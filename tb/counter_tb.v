module counter_tb;
  initial begin
    $dumpfile("counter_tb.vcd");
    $dumpvars(0, counter_tb);
  end

  parameter SIZE = 8;
  reg             clk;
  reg             rst;
  wire [SIZE-1:0] ctr;

  counter#(
    .SIZE(SIZE)
  )
  counter_inst(
    .clk(clk),
    .rst(rst),
    .ctr(ctr)
  );

  initial begin
    rst = 0;
    clk = 0;
    #10 rst = 1;
    #40 rst = 0;
    #1  rst = 1;
    #50 $finish;
  end

  always #1 clk = !clk;
endmodule
