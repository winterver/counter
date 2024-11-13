`timescale 1ps / 1ps
module risc8_tb;
  initial begin
    $dumpfile("risc8_tb.vcd");
    $dumpvars(0, risc8_tb);
  end

  reg    rst;
  reg    clk;
  core DUT(.rst(rst),
           .clk(clk));

  initial begin
    clk = 1'b0;
    #150
    repeat(99) begin
      clk = 1'b1;
      #50 clk = 1'b0;
      #50;
    end
    clk = 1'b1;
    #50;
  end


  initial begin
    rst = 1'b0;
    #100
    rst = 1'b1;
    #9000;
  end

  initial begin
    #20000 $finish;
  end
endmodule
