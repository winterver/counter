module counter#(
  parameter SIZE = 8
)(
  input                 clk,
  input                 rst,
  output reg [SIZE-1:0] ctr
);

  always @(posedge clk or negedge rst) begin
    if (!rst) begin
      ctr <= 1'b0;
    end else begin
      ctr <= ctr + 1'b1;
    end
  end
endmodule
