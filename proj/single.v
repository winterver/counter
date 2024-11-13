module pc(
  input             clk,
  input             rst,
  input       [4:0] in,
  output reg  [4:0] out
);
  always @(posedge clk) begin
    if (rst) out <= 5'b0;
    else out <= in;
  end
endmodule

module acc(
  input             clk,
  input             rst,
  input             ld,
  input       [7:0] in,
  output reg  [7:0] out
);
  always @(posedge clk) begin
    if (rst) out = 8'b0;
    else if (ld) out <= in;
  end
endmodule

module alu(
  input       [2:0] op,
  input       [7:0] a,
  input       [7:0] b,
  output reg  [7:0] out
);
  always @* begin
    case(op)
      3'b000: out = a + b;
      3'b001: out = a - b;
      3'b010: out = a & b;
      3'b011: out = a | b;
      3'b100: out = ~ b;
      3'b101: out = a ^ b;
      3'b110: out = a ~^ b;
      default:out = 8'b0;
    endcase
  end
endmodule

module adder(
  input   [4:0] a,
  input   [4:0] b,
  output  [4:0] out
);
  assign out = a + b;
endmodule

module mux2to1#(
  parameter BITS = 8
)(
  input               sel,
  input   [BITS-1:0]  in1,
  input   [BITS-1:0]  in2,
  output  [BITS-1:0]  out
);
  assign out = sel ? in2 : in1;
endmodule

module controller(
  input  [2:0]  op,
  output reg    rd_mem,
  output reg    wr_mem,
  output reg    ld_ac,
  output reg    ac_src,
  output reg    pc_src,
  output reg    branch
);
  always @* begin
    rd_mem = 1'b0;
    wr_mem = 1'b0;
    ld_ac  = 1'b0;
    ac_src = 1'b0;
    pc_src = 1'b0;
    branch = 1'b0;
    case(op)
      3'b000: begin // LDA [addr]
        rd_mem = 1'b1;
        wr_mem = 1'b0;
        ld_ac  = 1'b1;
        ac_src = 1'b0;
      end
      3'b001: begin // SUB [addr]
        rd_mem = 1'b1;
        wr_mem = 1'b0;
        ld_ac  = 1'b1;
        ac_src = 1'b0;
      end
      3'b010: begin // AND [addr]
        rd_mem = 1'b1;
        wr_mem = 1'b0;
        ld_ac  = 1'b1;
        ac_src = 1'b0;
      end
      3'b011: begin // ORA [addr]
        rd_mem = 1'b1;
        wr_mem = 1'b0;
        ld_ac  = 1'b1;
        ac_src = 1'b0;
      end
      3'b100: begin // NOT
        rd_mem = 1'b1;
        wr_mem = 1'b0;
        ld_ac  = 1'b1;
        ac_src = 1'b0;
      end
      3'b101: begin // XOR [addr]
        rd_mem = 1'b1;
        wr_mem = 1'b0;
        ld_ac  = 1'b1;
        ac_src = 1'b0;
      end
      3'b110: begin // XNOR [addr]
        rd_mem = 1'b1;
        wr_mem = 1'b0;
        ld_ac  = 1'b1;
        ac_src = 1'b0;
      end
      3'b111: begin // JMP [addr]
        rd_mem = 1'b0;
        wr_mem = 1'b0;
        ld_ac  = 1'b0;
        ac_src = 1'b0;
        pc_src = 1'b1;
        branch = 1'b1;
      end
      default: begin
        rd_mem = 1'b0;
        wr_mem = 1'b0;
        ld_ac  = 1'b0;
        ac_src = 1'b0;
        pc_src = 1'b0;   
      end
    endcase
  end
endmodule

module datamem(
  input             rd,
  input             wr,
  input       [4:0] addr,
  input       [7:0] in,
  output reg  [7:0] out
);
  reg         [7:0] mem[0:31];
  always @(rd, addr)  if (rd) out = mem[addr];
  always @(wr, in)    if (wr) mem[addr] = in;
  initial begin
    mem[0] = 8'd1;
    mem[1] = 8'd2;
    mem[2] = 8'd3;
    mem[3] = 8'd4;
    mem[4] = 8'd5;
  end
endmodule

module codemem(
  input       [4:0] addr,
  output reg  [7:0] out
);
  reg         [7:0] ins[0:31];
  always @(addr) out = ins[addr];
  initial begin
    ins[0] = 8'h00;   // ADD [0]
    ins[1] = 8'h21;   // SUB [1]
    ins[2] = 8'h42;   // AND [2]
    ins[3] = 8'h63;   // OR  [3]
    ins[4] = 8'h84;   // NOT [4]
    ins[5] = 8'hA4;   // XOR [4]
    ins[6] = 8'hC4;   // XNOR[4]
    ins[7] = 8'hEA;   // JMP 10
    ins[10] = 8'h00;  // ADD [0]
    ins[11] = 8'hE0;  // JMP 0
  end
endmodule

module datapath(
  input         clk,
  input         rst,
  input         ld_ac,
  input         ac_src,
  input         pc_src,
  output [2:0]  op,
  output [4:0]  im_addr,
  input  [7:0]  im_in,
  output [4:0]  dm_addr,
  output [7:0]  dm_in,
  input  [7:0]  dm_out,
  output [7:0]  ac_out,
  output [7:0]  alu_out
);
  wire   [7:0]  mux2_out;
  wire   [4:0]  mux1_out;
  wire   [4:0]  adder_out;
  wire   [4:0]  pc_out;

  pc pc0(
    .clk(clk),
    .rst(rst),
    .in(mux1_out),
    .out(pc_out)
  );
  adder adder0(
    .a(pc_out),
    .b(5'b00001),
    .out(adder_out)
  );
  mux2to1#(.BITS(5)) mux1(
    .sel(pc_src),
    .in1(adder_out),
    .in2(im_in[4:0]),
    .out(mux1_out)
  );
  acc acc0(
    .clk(clk),
    .rst(rst),
    .ld(ld_ac),
    .in(mux2_out),
    .out(ac_out)
  );
  alu alu0(
    .op(op),
    .a(ac_out),
    .b(dm_out),
    .out(alu_out)
  );
  mux2to1#(.BITS(8)) mux2(
    .sel(ac_src),
    .in1(alu_out),
    .in2(dm_out),
    .out(mux2_out)
  );
  assign im_addr = pc_out;
  assign op = im_in[7:5];
  assign dm_addr = im_in[4:0];
  assign dm_in = ac_out;
endmodule

module CPU(
  input         clk,
  input         rst,
  output        rd_mem,
  output        wr_mem,
  output  [4:0] im_addr,
  output  [4:0] dm_addr,
  input   [7:0] im_in,
  output  [7:0] dm_in,
  input   [7:0] dm_out,
  output  [7:0] ac_out,
  output  [7:0] alu_out,
  output  [2:0] op
);
  wire ld_ac;
  wire ac_src;
  wire pc_src;
  wire branch;
  datapath dpu(
    .clk(clk),
    .rst(rst),
    .ld_ac(ld_ac),
    .ac_src(ac_src),
    .pc_src(pc_src),
    .op(op),
    .im_addr(im_addr),
    .dm_addr(dm_addr),
    .im_in(im_in),
    .dm_in(dm_in),
    .dm_out(dm_out),
    .ac_out(ac_out),
    .alu_out(alu_out)
  );
  controller ctl(
    .op(op),
    .rd_mem(rd_mem),
    .wr_mem(wr_mem),
    .ld_ac(ld_ac),
    .ac_src(ac_src),
    .pc_src(pc_src),
    .branch(branch)
  );
endmodule

module CPU2(
  input         clk,
  input         rst,
  output        rd_mem,
  output        wr_mem,
  output  [4:0] im_addr,
  output  [4:0] dm_addr,
  input   [7:0] im_in,
  output  [7:0] dm_in,
  input   [7:0] dm_out
);
  wire ld_ac;
  wire ac_src;
  wire pc_src;
  wire branch;
  wire [7:0] ac_out;
  wire [7:0] alu_out;
  wire [2:0] op;
  datapath dpu(
    .clk(clk),
    .rst(rst),
    .ld_ac(ld_ac),
    .ac_src(ac_src),
    .pc_src(pc_src),
    .op(op),
    .im_addr(im_addr),
    .dm_addr(dm_addr),
    .im_in(im_in),
    .dm_in(dm_in),
    .dm_out(dm_out),
    .ac_out(ac_out),
    .alu_out(alu_out)
  );
  controller ctl(
    .op(op),
    .rd_mem(rd_mem),
    .wr_mem(wr_mem),
    .ld_ac(ld_ac),
    .ac_src(ac_src),
    .pc_src(pc_src),
    .branch(branch)
  );
endmodule