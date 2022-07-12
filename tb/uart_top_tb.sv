

`timescale 1ns/10ps

module uart_top_tb;

   uart_top uart_top_DUT();
   

   initial begin
      $display("UART playground testbench");      
      $finish(1);
   end
   
endmodule // uart_top_tb
