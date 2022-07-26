

`include "../src/uart_defines.v"

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
   localparam integer BUADRATE=9600;      //[] bits per sec
   localparam integer NANOSECOND=1e+9;
   localparam         UART_BIT_PERIOD=(NANOSECOND/BUADRATE);
   localparam integer UART_ACK_TIMEOUT=5;
   
   
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


   task wait_for_ack();
      fork
	 // THREAD 1
	 // assert ack recieved
	 begin
	    while(!wb.ack)
	      @(posedge wb.clk);
	    print("wishbone Ack recieved");
	 end
	 // THREAD 2
	 // Ack timeout
	 begin
	    for(int i=0; i<UART_ACK_TIMEOUT; i++)
	      @(posedge wb.clk);
	    print("wishbone Ack timeout");
	 end
      join_any;
      disable fork;
   endtask // wishbone_write
   
   
   
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
      wb.we=1'b0;   //write
      wb.stb=1'b0;  //chipselect
      wb.cyc=1'b0;  //valid cycle
      wait_for_ack();
      delay(1);
   endtask // wishbone_write

   
   task wishbone_read;
      input logic [ADDR_W-1:0] address;
      output logic [DATA_W-1:0] data;      
      print($sformatf("Wishbone read req from address 0x%x", address));
      @(negedge wb.clk);
      wb.address=address;
      wb.we=1'b0;   //read
      wb.stb=1'b1;  //chipselect
      wb.cyc=1'b1;  //valid cycle    
      delay(1);
      wb.stb=1'b0;  //chipselect
      wb.cyc=1'b0;  //valid cycle
      wait_for_ack();
      data = wb.data_in;
      print($sformatf("Wishbone data recieved 0x%x", data));
      delay(1);
   endtask // wishbone_read

   
   task config_uart();
      logic [DATA_W-1:0] data;
      int 		 devisor;
      print("Configuring UART engine");
      @(posedge wb.clk);

      // set the line control register bit 7 to high
      // to allow access to the devider latches
      wishbone_read( `UART_REG_LC, data ); // Line Control
      data |= 8'b10000000;
      wishbone_write( `UART_REG_LC, data ); // Line Control
      wishbone_read( `UART_REG_LC, data ); // Line Control

      // set devisor latches MSB first
      // devisor = (Sys clock / (16*BaudRate))
      wishbone_read( `UART_REG_DL2, data );
      wishbone_read( `UART_REG_DL1, data );
      devisor = (0.02)*(NANOSECOND)/(16*BUADRATE);
      print($sformatf("devosor value set to 0x%x", devisor));
      wishbone_write( `UART_REG_DL2, devisor & 32'hFF00 );	 
      wishbone_write( `UART_REG_DL1, devisor & 32'h00FF ); // Divisor latch bytes (1-2
      wishbone_read( `UART_REG_DL2, data );
      wishbone_read( `UART_REG_DL1, data );     
      
      // set the line control register bit 7 to low
      // to deny access to the devider latches
      wishbone_read( `UART_REG_LC, data ); // Line Control
      data &= 8'b01111111;
      wishbone_write( `UART_REG_LC, data ); // Line Control
      wishbone_read( `UART_REG_LC, data ); // Line Control

      // set the FIFO control register bits [7:6] to low
      // to trigger interrupt
      wishbone_write( `UART_REG_FC, 8'b00000000 ); // FIFO control
      
      // enable interrupts
      wishbone_read( `UART_REG_IE, data ); // Line Control
      data |= 8'b00000001;
      wishbone_write( `UART_REG_IE, data ); // Line Control
      wishbone_read( `UART_REG_IE, data ); // Line Control      
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
	 begin
	    for(int i=10; i!=0; i--) begin
	       logic [DATA_W-1:0] data;
	       uart_bit_wait(3); 
	       UART_H2D_transmit($random());
	       uart_bit_wait(1);
	       wishbone_read( `UART_REG_RB, data ); // Line Control
	    end
	 end
	 
	 // THREAD 2
	 // waiting for interrupts 
	 // form uart 
	 begin
	 end
      join
      $finish(1);
   end
   
endmodule // uart_top_tb


/*	 
	 wishbone_write( `UART_REG_RB, 0 ); // receiver buffer
	 wishbone_write( `UART_REG_TR, 0 ); // transmitter
	 wishbone_write( `UART_REG_IE, 0 ); // Interrupt enable
	 wishbone_write( `UART_REG_II, 0 ); // Interrupt identification
	 wishbone_write( `UART_REG_FC, 0 ); // FIFO control
	 wishbone_write( `UART_REG_LC, 0 ); // Line Control
	 wishbone_write( `UART_REG_MC, 0 ); // Modem control
	 wishbone_write( `UART_REG_LS, 0 ); // Line status
	 wishbone_write( `UART_REG_MS, 0 ); // Modem status
	 wishbone_write( `UART_REG_SR, 0 ); // Scratch register
	 wishbone_write( `UART_REG_DL1, 0 ); // Divisor latch bytes (1-2
	 wishbone_write( `UART_REG_DL2, 0 );	 
	 wishbone_write(address, $random());
	 wishbone_read(i);
 */
