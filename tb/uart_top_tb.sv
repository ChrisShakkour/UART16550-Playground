
// synopsys translate_off
`include "timescale.v"
// synopsys translate_on

// synopsys translate_off
`include "uart_defines.v"
// synopsys translate_on

`timescale 1ns/1ps

interface wishbone;   
   logic                        clk;
   logic   		        rstn;
   logic [4:0] address;
   logic [31:0] data_in;
   logic [31:0] data_out;
   logic 			we;
   logic 			stb;
   logic 			cyc;
   logic 			ack;
   logic 			sel;
   
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
   
      
   logic 	      clk_en;
   logic 	      interrupt;
   logic 	      uart_master_tx; //pc side to uart
   logic 	      uart_master_rx; //pc side to uart
   wishbone           wb();
   

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
      .rts_pad_o  (),
      .cts_pad_i  (),
      .dtr_pad_o  (),
      .dsr_pad_i  (),
      .ri_pad_i   (),
      .dcd_pad_i  ()
`ifdef UART_HAS_BAUDRATE_OUTPUT
      ,.baud_o    ()
`endif
      );


   task uart_bit_wait(int bits);
      #(bits*UART_BIT_PERIOD);
   endtask // uart_bit_wait
   
   
   task delay(int cycles);
      #(cycles*CLK_PERIOD);
   endtask // delay
   

   // initial state
   task init_signals;
      // clock gating
      clk_en=0;
      // wishbone signals
      wb.clk=0;
      wb.rstn=1;
      wb.address='0;
      wb.data_out='0;
      wb.we='0;
      wb.tb='0;
      wb.cyc='0;
      wb.sel='0;
      // UART HOST TO DEVICE TX line
      // held high when not transmitting
      uart_master_tx=1'b1; 
      
   endtask // init_signals
   

   task wishbone_write();
   endtask // wishbone_write

   task wishbone_read();
   endtask // wishbone_read


   //uart host to device transmit
   task UART_H2D_transmit;
      input logic [N_DATA_BITS-1:0] data;      
      // start bit
      uart_master_tx=1'b0;
      uart_bit_wait(1);       
      // data bits
      for(int i=0; i<N_DATA_BITS; i++)
	uart_master_tx = data[i]; uart_bit_wait(1);		     
      // parity
      if(PARITY_EN) 
	uart_master_tx = ^data; uart_bit_wait(1);
      // end bits
      uart_master_tx = 1'b1;
      uart_bit_wait((SINGLE_STOP_BIT) ? 1 : 2);
   endtask // UART_H2D_transmit
   
   
   initial begin
      $display("UART playground testbench");      
      $finish(1);
   end
   
endmodule // uart_top_tb
