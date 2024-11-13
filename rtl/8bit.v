module pc(
  input         clk,
  input         rst,
  input         en,
  input         oe,
  input         ld,
  input   [3:0] in,
  output  [3:0] out
);
  reg     [3:0] val;
  assign out = oe ? val : 4'hz;
  always @(posedge clk, posedge rst) begin
    if (rst) val <= 0;
    else if (ld) val <= in;
    else if (en) val <= val+1;
  end
endmodule

module reg8(
  input         clk,
  input         rst,
  input         en,
  input         ld,
  input   [7:0] in,
  output  [7:0] out
);
  reg     [7:0] val;
  assign out = en ? val : 4'hz;
  always @(posedge clk, posedge rst) begin
    if (rst) val <= 4'hz;
    else if (ld) val <= in;
  end
endmodule

module alu(
  input         en,
  input         op,
  input   [7:0] a,
  input   [7:0] b,
  output  [7:0] r,
  output        C,
  output        Z
);
  reg     [8:0] res;
  assign C = res[8];
  assign Z = res[7:0] == 0;
  assign r = en ? res[7:0] : 8'hzz;
  always @* begin
    if (op) res <= a - b;
    else res <= a + b;
  end
endmodule

module mar(
  input         clk,
  input         rst,
  input         ld,
  input   [3:0] in,
  output  [3:0] out
);
  reg     [3:0] val;
  assign out = val;
  always @(posedge clk, posedge rst) begin
    if (rst) val <= 4'hz;
    else if (ld) val <= in;
  end
endmodule

module mem(
  input         clk,
  input         ld,
  input         oe,
  input   [3:0] addr,
  input   [7:0] in,
  output  [7:0] out
);
  reg     [7:0] mem[0:15];
  assign out = oe ? mem[addr] : 8'hzz;
  always @(posedge clk) begin
    if (ld) mem[addr] <= in;
  end
endmodule

module controller(
  input               clk,
  input               rst,
  input       [3:0]   op,
  output reg  [16:0]  out
);
  reg     [3:0]   ctr;
  always @(posedge clk, posedge rst) begin
    if (rst)
      ctr <= 4'b0000;
    else if (ctr == 4'b0110)
      ctr <= 4'b0000;
    else
      ctr <= ctr + 1;
  end
  always @* begin
    case({ctr,op})
      {4'b0000,4'bzzzz}: out = 17'b00000001100000000;
      {4'b0001,4'bzzzz}: out = 17'b00000100001000010;
      // LDA
      {4'b0010,4'b0000}: out = 17'b00000000000000001;
      {4'b0011,4'b0000}: out = 17'b00000000100000001;
      {4'b0100,4'b0000}: out = 17'b00000000001100000;
      {4'b0101,4'b0000}: out = 17'b00000000000000000;
      {4'b0110,4'b0000}: out = 17'b00000000000000000;
      // instruction register => bus
      {4'b0010,4'b0001}: out = 17'b00000000000000001;
      {4'b0011,4'b0001}: out = 17'b00000000100000001;
      {4'b0100,4'b0001}: out = 17'b00000000010010000;
      {4'b0101,4'b0001}: out = 17'b00000000000000000;
      {4'b0110,4'b0001}: out = 17'b00000000000000000;
      // ADD
      {4'b0010,4'b0010}: out = 17'b00000000000000001;
      {4'b0011,4'b0010}: out = 17'b00000000100000001;
      {4'b0100,4'b0010}: out = 17'b00000000001001000;
      {4'b0101,4'b0010}: out = 17'b00001000000000000;
      {4'b0110,4'b0010}: out = 17'b00001000000100000;
      // SUB
      {4'b0010,4'b0011}: out = 17'b00000000000000001;
      {4'b0011,4'b0011}: out = 17'b00000000100000001;
      {4'b0100,4'b0011}: out = 17'b00000000001001000;
      {4'b0101,4'b0011}: out = 17'b00001000000000000;
      {4'b0110,4'b0011}: out = 17'b00001100000100000;
      // JMP
      {4'b0010,4'b0100}: out = 17'b00000000000000001;
      {4'b0011,4'b0100}: out = 17'b00000010000000001;
      {4'b0100,4'b0100}: out = 17'b00000000000000000;
      {4'b0101,4'b0100}: out = 17'b00000000000000000;
      {4'b0110,4'b0100}: out = 17'b00000000000000000;
      // OUT
      {4'b0010,4'b0101}: out = 17'b00000000000000001;
      {4'b0011,4'b0101}: out = 17'b00000000100000001;
      {4'b0100,4'b0101}: out = 17'b00000000001000000;
      {4'b0101,4'b0101}: out = 17'b01000000001000000;
      {4'b0110,4'b0101}: out = 17'b00100000000000000;
      // HLT
      {4'b0010,4'b0110}: out = 17'b10000000000000000;
      {4'b0011,4'b0110}: out = 17'b10000010000000000;
      {4'b0100,4'b0110}: out = 17'b00000000000000000;
      {4'b0101,4'b0110}: out = 17'b00000000000000000;
      {4'b0110,4'b0110}: out = 17'b00000000000000000;
    endcase
  end
endmodule
