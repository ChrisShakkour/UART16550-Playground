
// synopsys translate_off
`include "timescale.v"
// synopsys translate_on

// synopsys translate_off
`include "uart_defines.v"
// synopsys translate_on

`timescale 1ns/1ns

interface wishbone
  #(parameter integer ADDR_W=3,
    parameter integer DATA_W=8,
    parameter integer SELECT_W=4);   
   logic                        clk;
   logic   		        rstn;
   logic [ADDR_W-1:0] 		address;
   logic [DATA_W-1:0] 		data_in;
   logic [DATA_W-1:0] 		data_out;
   logic 			we;
   logic 			stb;
   logic 			cyc;
   logic 			ack;
   logic [SELECT_W-1:0] 	sel;
   
   // master side
   modport master
     (
      input  ack, data_in,
      output clk, rstn, address, data_out, we, stb, cyc, sel
      );

   // slave side
   modport slave
     (
      output ack, data_in,
      input  clk, rstn, address, data_out, we, stb, cyc, sel
      );
   
endinterface // wishbone

	       
module uart_top_tb;
   
   // CLK PARAMETERS 20MHz CLK
   localparam integer HALF_CLK=25; 
   localparam integer CLK_PERIOD=2*HALF_CLK;

   // UART PROTOCOL PARAMS
   localparam bit     LSB_FIRST=1;        //[0/1] 0: MSB first,   1: LSB first
   localparam bit     PARITY_EN=0;        //[0/1] 0: disable,     1: enable
   localparam bit     SINGLE_STOP_BIT=1;  //[0/1] 0: 2 stop bits, 1: single
   localparam integer N_DATA_BITS=8;      //[5:8] can be any number between 5 and 8
   localparam integer BUADRATE=9200;      //[] bits per sec
   localparam integer NANOSECOND=1e+9;
   localparam         UART_BIT_PERIOD=(NANOSECOND/BUADRATE);

   // wishbone params
   localparam integer  ADDR_W=3;
   localparam integer  DATA_W=8;
   localparam integer  SELECT_W=4;
      
   logic 	      clk_en;
   logic 	      interrupt;
   logic 	      uart_rts_n;
   logic 	      uart_dtr_n;
   logic 	      uart_master_tx; //pc side to uart
   logic 	      uart_master_rx; //pc side to uart
   
   wishbone #(.ADDR_W   (ADDR_W),
	      .DATA_W   (DATA_W),
	      .SELECT_W (SELECT_W)
	      ) wb();

   always #HALF_CLK
     wb.clk = (clk_en) ? ~wb.clk : 0;
   
   // UART DUT instance
   uart_top uart_top_DUT
     (
      // Wishbone signals
      .wb_clk_i   (wb.clk),
      .wb_rst_i   (~wb.rstn),
      .wb_adr_i   (wb.address),
      .wb_dat_i   (wb.data_out),
      .wb_dat_o   (wb.data_in),
      .wb_we_i    (wb.we),
      .wb_stb_i   (wb.stb),
      .wb_cyc_i   (wb.cyc),
      .wb_ack_o   (wb.ack),
      .wb_sel_i   (wb.sel),
      // interrupt request
      .int_o      (interrupt), 
      // uart serial input/output
      .stx_pad_o  (uart_master_rx),
      .srx_pad_i  (uart_master_tx),
      // modem signals
      .rts_pad_o  (uart_rts_n),
      .cts_pad_i  (uart_rts_n),
      .dtr_pad_o  (uart_dtr_n),
      .dsr_pad_i  (uart_dtr_n),
      .ri_pad_i   ('1),
      .dcd_pad_i  ('1)
`ifdef UART_HAS_BAUDRATE_OUTPUT
      ,.baud_o    ()
`endif
      );

   
   task print(string str);
      $display("-I- time=%0t[ns]: %s",
               $time, str);   
   endtask // print


   task uart_bit_wait(int bits);
      #(bits*UART_BIT_PERIOD);
   endtask // uart_bit_wait
   
   
   task delay(int cycles);
      #(cycles*CLK_PERIOD);
   endtask // delay

   
   task reset();
      print("Asserting reset");
      wb.rstn=1'b0;
      delay(100);
      wb.rstn=1'b1;
   endtask // reset


   task enable_clk();
      clk_en=1'b1;
   endtask // enable_clk
     
   
   // initial state
   task init();
      print("Initializing TB signals");
      // clock gating
      clk_en=0;
      // wishbone signals
      wb.clk=0;
      wb.rstn=1;
      wb.address='0;
      wb.data_out='0;
      wb.we='0;
      wb.stb='0;
      wb.cyc='0;
      wb.sel=4'd1;
      // UART HOST TO DEVICE TX line
      // held high when not transmitting
      uart_master_tx=1'b1;       
   endtask // init_signals

   
   task wishbone_write;
      input logic [ADDR_W-1:0] address;
      input logic [DATA_W-1:0] data;
      print($sformatf("Wishbone write 0x%x to address 0x%x", data, address));
      @(negedge wb.clk);
      wb.address=address;
      wb.data_out=data;
      wb.we=1'b1;   //write
      wb.stb=1'b1;  //chipselect
      wb.cyc=1'b1;  //valid cycle    
      delay(1);
      wb.stb=1'b0;  //chipselect
      wb.cyc=1'b0;  //valid cycle    
   endtask // wishbone_write

   
   task wishbone_read;
      input logic [ADDR_W-1:0] address;
      print($sformatf("Wishbone read req from address 0x%x", address));
      @(negedge wb.clk);
      wb.address=address;
      wb.we=1'b0;   //read
      wb.stb=1'b1;  //chipselect
      wb.cyc=1'b1;  //valid cycle    
      delay(1);
      wb.stb=1'b0;  //chipselect
      wb.cyc=1'b0;  //valid cycle    
   endtask // wishbone_read

   
   task config_uart();
      logic [ADDR_W-1:0] address;
      print("Configuring UART engine");
      @(posedge wb.clk);
      for(int i=0; i<8; i++) begin
	 address = $random();
	 //wishbone_write(address, $random());
	 //delay(3);
	 wishbone_read(i);
	 delay(3);
      end
   endtask // config_uart
      

   //uart host to device transmit
   task UART_H2D_transmit;
      input logic [N_DATA_BITS-1:0] data;      
      print($sformatf("UART transmiting Host to Device, %b", data));
      // start bit
      uart_master_tx=1'b0;
      uart_bit_wait(1);       
      // data bits
      for(int i=0; i<N_DATA_BITS; i++) begin
	 uart_master_tx = data[i]; 
	 uart_bit_wait(1);
      end		     
      // parity
      if(PARITY_EN) begin 
	 uart_master_tx = ^data;
	 uart_bit_wait(1);
      end
      // end bits
      uart_master_tx = 1'b1;
      uart_bit_wait((SINGLE_STOP_BIT) ? 1 : 2);
   endtask // UART_H2D_transmit
   
   
   initial begin
      $display("UART playground testbench");      
      delay(10); init();
      delay(10); reset();
      delay(10); enable_clk();
      delay(10); config_uart();
      delay(10);
      
      fork
	 // THREAD 1
	 // Async UART transmissions
	 // from host to device 
	 begin/*
	    for(int i=0; i<10; i++) begin
	       uart_bit_wait(10); 
	       UART_H2D_transmit($random());
	    end
	 */end
	 
	 // THREAD 2
	 // waiting for interrupts 
	 // form uart 
	 begin
	 end

      join
      $finish(1);
   end
   
endmodule // uart_top_tb
