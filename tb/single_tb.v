// modified from https://www.slideshare.net/slideshow/8-bit-single-cycle-processor/30891706
module single_tb;
  initial begin
    $dumpfile("single_tb.vcd");
    $dumpvars(0, single_tb);
  end

  reg clk;
  reg rst;
  wire [7:0] im_in;
  wire [7:0] dm_out;
  // for inspection
  wire rd_mem;
  wire wr_mem;
  wire [4:0] im_addr;
  wire [4:0] dm_addr;
  wire [7:0] dm_in;
  wire [7:0] ac_out;
  wire [7:0] alu_out;
  wire [2:0] op;

  CPU cpu(
    .clk(clk),
    .rst(rst),
    .rd_mem(rd_mem),
    .wr_mem(wr_mem),
    .im_addr(im_addr),
    .dm_addr(dm_addr),
    .im_in(im_in),
    .dm_in(dm_in),
    .dm_out(dm_out),
    .ac_out(ac_out),
    .alu_out(alu_out),
    .op(op)
  );
  codemem im(
    .addr(im_addr), 
    .out(im_in)
  );
  datamem dm(
    .rd(rd_mem),
    .wr(wr_mem),
    .addr(dm_addr), 
    .in(dm_in), 
    .out(dm_out)
  );

  initial begin
    clk = 0;
    rst = 1;
    #20 rst = 0;
    #500 $finish;
  end
  always #10 clk = ~clk;
endmodule
