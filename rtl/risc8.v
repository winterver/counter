module rom(data, addr, read, en);
  input read, en;
  input [7:0] addr;
  output [7:0] data;

  reg [7:0] mem[255:0];

  assign data = read && en ? mem[addr] : 8'hzz;

  initial begin
    mem[0] = 8'b000_00000;   //NOP
    mem[1] = 8'b001_00001;   //LDO s1
    mem[2] = 8'b010_00001;   //rom(65)   //end, reg[1]<-rom[65]
    mem[3] = 8'b001_00010;   //LDO s2
    mem[4] = 8'b010_00010;   //rom(66)   //end, reg[2]<-rom[66]
    mem[5] = 8'b001_00011;   //LDO s3
    mem[6] = 8'b010_00011;   //rom(67)   //end, reg[3]<-rom[67]
    mem[7] = 8'b100_00001;   //PRE s1
    mem[8] = 8'b101_00010;   //ADD s2
    mem[9] = 8'b110_00001;   //LDM s1  // REG[1] <- REG[1]+REG[2]
    mem[10] = 8'b011_00001;  //STO s1
    mem[11] = 8'b000_00001;  //ram(1)  // RAM[1] <- REG[1]
    mem[12] = 8'b010_00010;  //LDA s2
    mem[13] = 8'b000_00001;  //ram(1)  // REG[2] <- RAM[1]
    mem[14] = 8'b100_00011;  //PRE s3
    mem[15] = 8'b101_00010;  //ADD s2
    mem[16] = 8'b110_00011;  //LDM s3  // REG[3] <- REG[2]+REG[3]
    mem[17] = 8'b011_00011;  //STO s3
    mem[18] = 8'b000_00010;  //ram(2)   //REG[3] -> ram[2]
    mem[19] = 8'b111_00000;  //HLT
    mem[65] = 8'b001_00101;  //37
    mem[66] = 8'b010_11001;  //89
    mem[67] = 8'b001_10101;  //53
  end
endmodule

module ram(data, addr, en, read, write);
  input en, read, write;
  input [7:0] addr;
  inout [7:0] data;

  reg [7:0] mem[255:0];

  assign data = read && en ? mem[addr] : 8'hzz;
  always @(posedge write) mem[addr] <= data;
endmodule

module pc(pc_addr, clk, rst, en);
  input clk, rst, en;
  output reg [7:0] pc_addr;

  always @(posedge clk or negedge rst) begin
      if(!rst) pc_addr <= 8'd0;
      else begin
          if(en) pc_addr <= pc_addr+1;
          else pc_addr <= pc_addr;
      end
  end
endmodule

module acc(in, out, en, clk, rst);
  input clk, rst, en;
  input [7:0] in;
  output reg [7:0] out;

  always @(posedge clk or negedge rst) begin
      if(!rst)    out <= 8'd0;
      else begin
          if(en)  out <= in;
          else    out <= out;
      end
  end
endmodule

module addr_mux(addr, sel, ir_ad, pc_ad);
  input sel;
  input [7:0] ir_ad, pc_ad;
  output [7:0] addr;
  assign addr = sel ? ir_ad : pc_ad;
endmodule

module alu(alu_out, alu_in, acc, op);
  input [2:0] op;
  input [7:0] alu_in, acc;
  output reg [7:0] alu_out;

  parameter   NOP=3'b000,
              LDO=3'b001,
              LDA=3'b010,
              STO=3'b011,
              PRE=3'b100,
              ADD=3'b101,
              LDM=3'b110,
              HLT=3'b111;

  always @* begin
    casez(op)
      NOP:    alu_out = acc;
      HLT:    alu_out = acc;
      LDO:    alu_out = alu_in;
      LDA:    alu_out = alu_in;
      STO:    alu_out = acc;
      PRE:    alu_out = alu_in;
      ADD:    alu_out = acc + alu_in;
      LDM:    alu_out = acc;
      default:alu_out = 8'hzz;
    endcase
  end
endmodule

module reg32(in, data, write, read, addr, clk);
  input write, read, clk;
  input [7:0] in;
  input [7:0] addr;
  output [7:0] data;

  reg [7:0] R[31:0];
  wire [4:0] r_addr;

  assign r_addr = addr[4:0];
  assign data = read ? R[r_addr] : 8'hzz;

  always @(posedge clk) begin
    if(write) R[r_addr] <= in;
  end
endmodule

module ir(data, fetch, clk, rst, ins, ad1, ad2);
  input clk, rst;
  input [1:0] fetch;
  input [7:0] data;
  output [2:0] ins;
  output [4:0] ad1;
  output [7:0] ad2;

  reg [7:0] ins_p1, ins_p2;
  reg [2:0] state;

  assign ins = ins_p1[7:5]; //hign 3 bits, instructions
  assign ad1 = ins_p1[4:0]; //low 5 bits, register address
  assign ad2 = ins_p2;

  always @(posedge clk or negedge rst) begin
    if(!rst) begin
      ins_p1 <= 8'd0;
      ins_p2 <= 8'd0;
    end
    else begin
      if(fetch==2'b01) begin
        ins_p1 <= data;
        ins_p2 <= ins_p2;
      end
      else if(fetch==2'b10) begin
        ins_p1 <= ins_p1;
        ins_p2 <= data;
      end
      else begin
        ins_p1 <= ins_p1;
        ins_p2 <= ins_p2;
      end
    end
  end
endmodule

module controller(ins, clk, rst, write_r, read_r, PC_en, fetch, acc_en, ram_en, rom_en,ram_write, ram_read, rom_read, ad_sel);

  input clk, rst;
  input [2:0] ins;

  output reg write_r, read_r, PC_en, acc_en, ram_en, rom_en;
  output reg ram_write, ram_read, rom_read, ad_sel;
  output reg [1:0] fetch;

  reg [3:0] state;        // current state
  reg [3:0] next_state;   // next state

  parameter   NOP=3'b000, // no operation
              LDO=3'b001, // load ROM to register
              LDA=3'b010, // load RAM to register
              STO=3'b011, // Store intermediate results to acculator
              PRE=3'b100, // Prefetch Data from Address
              ADD=3'b101, // Adds the contents of the mem address or integer to the accumulator
              LDM=3'b110, // Load Multiple
              HLT=3'b111; // Halt

  // state code
  parameter Sidle=4'hf,
               S0=4'd0,
               S1=4'd1,
               S2=4'd2,
               S3=4'd3,
               S4=4'd4,
               S5=4'd5,
               S6=4'd6,
               S7=4'd7,
               S8=4'd8,
               S9=4'd9,
               S10=4'd10,
               S11=4'd11,
               S12=4'd12;

  //PART A: D flip latch; State register
  always @(posedge clk or negedge rst)
  begin
      if(!rst) state<=Sidle;
          //current_state <= Sidle;
      else state<=next_state;
          //current_state <= next_state;
  end

  //PART B: Next-state combinational logic
  always @*
  begin
  case(state)
  S1:     begin
              if (ins==NOP) next_state=S0;
              else if (ins==HLT)  next_state=S2;
              else if (ins==PRE | ins==ADD) next_state=S9;
              else if (ins==LDM) next_state=S11;
              else next_state=S3;
          end

  S4:     begin
              if (ins==LDA | ins==LDO) next_state=S5;
              //else if (ins==STO) next_state=S7;
              else next_state=S7; // ---Note: there are only 3 long instrucions. So, all the cases included. if (counter_A==2*b11)
          end
  Sidle:  next_state=S0;
  S0:     next_state=S1;
  S2:     next_state=S2;
  S3:     next_state=S4;
  S5:     next_state=S6;
  S6:     next_state=S0;
  S7:     next_state=S8;
  S8:     next_state=S0;
  S9:     next_state=S10;
  S10:    next_state=S0;
  S11:    next_state=S12;
  S12:    next_state=S0;
  default: next_state=Sidle;
  endcase
  end

  // another style
  //PART C: Output combinational logic
  always@*
  begin
  case(state)
  // --Note: for each statement, we concentrate on the current state, not next_state
  // because it is combinational logic.
    Sidle: begin
           write_r=1'b0;
           read_r=1'b0;
           PC_en=1'b0;
           acc_en=1'b0;
           ram_en=1'b0;
           rom_en=1'b0;
           ram_write=1'b0;
           ram_read=1'b0;
           rom_read=1'b0;
           ad_sel=1'b0;
           fetch=2'b00;
           end
       S0: begin // load IR
           write_r=0;
           read_r=0;
           PC_en=0;
   acc_en=0;
           ram_en=0;
           rom_en=1;
           ram_write=0;
           ram_read=0;
           rom_read=1;
           ad_sel=0;
           fetch=2'b01;
           end
       S1: begin
           write_r=0;
           read_r=0;
           PC_en=1;
           acc_en=0;
           ram_en=0;
           ram_write=0;
           ram_read=0;
           rom_en=1;
           rom_read=1;
           ad_sel=0;
           fetch=2'b00;
           end
       S2: begin
           write_r=0;
           read_r=0;
           PC_en=0;
           acc_en=0;
           ram_en=0;
           rom_en=0;
           ram_write=0;
           ram_read=0;
           rom_read=0;
           ad_sel=0;
           fetch=2'b00;
           end
       S3: begin
           write_r=0;
           read_r=0;
           PC_en=0;
           acc_en=1;
           ram_en=0;
           rom_en=1;
           ram_write=0;
           ram_read=0;
           rom_read=1;
           ad_sel=0;
           fetch=2'b10;
           end
  S4: begin
           write_r=0;
           read_r=0;
           PC_en=1;
           acc_en=1;
           ram_en=0;
           ram_write=0;
           ram_read=0;
           rom_en=1;
           rom_read=1;
           ad_sel=0;
           fetch=2'b10;
           end
       S5: begin
           if (ins==LDO)
           begin
           write_r=1;
           read_r=0;
           PC_en=0;
           acc_en=1;
           ram_en=0;
           ram_write=0;
           ram_read=0;
           rom_en=1;
           rom_read=1;
           ad_sel=1;
           fetch=2'b01;
           end
           else
           begin
           write_r=1;
           read_r=0;
           PC_en=0;
           acc_en=1;
           ram_en=1;
           ram_write=0;
           ram_read=1;
           rom_en=0;
           rom_read=0;
           ad_sel=1;
           fetch=2'b01;
           end
           end
       S6: begin

       write_r=1'b0;
           read_r=1'b0;
           PC_en=1'b0; //** not so sure, log: change 1 to 0
           acc_en=1'b0;
           ram_en=1'b0;
           rom_en=1'b0;
           ram_write=1'b0;
           ram_read=1'b0;
           rom_read=1'b0;
           ad_sel=1'b0;
           fetch=2'b00;
      end

       S7: begin // STO, reg->ram. step1. read REG
           write_r=0;
           read_r=1;
           PC_en=0;
           acc_en=0;
           ram_en=0;
           rom_en=0;
           ram_write=0;
           ram_read=0;
           rom_read=0;
           ad_sel=0;
           fetch=2'b00;
           end
       S8: begin // STO, step2, write RAM
           write_r=0;
           read_r=1;
           PC_en=0;
           acc_en=0;
           rom_read=0;
           rom_en=0;

           ram_en=1;
           ram_write=1;
           ram_read=0;

           ad_sel=1;
           fetch=2'b00; //fetch=2'b10, ram_en=1, ram_write=1, ad_sel=1;
           end
       S9: begin
           if (ins==PRE) // REG->ACCUM
           begin
           write_r=0;
           read_r=1;
           PC_en=0;
           acc_en=1;
           ram_en=0;
           rom_en=0;
           ram_write=0;
           ram_read=0;
           rom_read=0;
           ad_sel=0;
           fetch=2'b00;
           end
           else
           begin
           write_r=0;
           read_r=1;
           PC_en=0;
           acc_en=1;
           ram_en=0;
           rom_en=0;
           ram_write=0;
           ram_read=0;
           rom_read=0;
           ad_sel=0;

           fetch=2'b00;
           end
           end
      S10: begin
           write_r=0;
           read_r=1;
           PC_en=0;
           acc_en=0;
           ram_en=0;
           rom_en=0;
           ram_write=0;
           ram_read=0;
           rom_read=0;
           ad_sel=0;
           fetch=2'b00;
           end
      S11: begin // LDM, step1, write reg
           write_r=1;
           read_r=0;
           PC_en=0;
           acc_en=1;
           ram_en=0;

           ram_write=0;
           ram_read=0;
           rom_en=1;
           rom_read=1;
           ad_sel=0;
           fetch=2'b00;

           end
      S12: begin
           write_r=0;
           read_r=0;
           PC_en=0;
           acc_en=0;
           ram_en=0;
           rom_en=0;
           ram_write=0;
           ram_read=0;
           rom_read=0;
           ad_sel=0;
           fetch=2'b00;
           end
  default: begin
           write_r=0;
           read_r=0;
           PC_en=0;
           acc_en=0;
           ram_en=0;
           rom_en=0;
           ram_write=0;
           ram_read=0;
           rom_read=0;
           ad_sel=0;
           fetch=2'b00;
           end
  endcase
  end
endmodule

module core(clk, rst);
  input clk, rst;

  wire write_r, read_r, PC_en, acc_en, ram_en, rom_en;
  wire ram_write, ram_read, rom_read, ad_sel;

  wire [1:0] fetch;
  wire [7:0] data, addr;
  wire [7:0] acc_out, alu_out;
  wire [7:0] ir_ad, pc_ad;
  wire [4:0] reg_ad;
  wire [2:0] ins;

  ram RAM1(.data(data),
           .addr(addr),
           .en(ram_en),
           .read(ram_read),
           .write(ram_write));

  rom ROM1(.data(data),
           .addr(addr),
           .en(rom_en),
           .read(rom_read));

  addr_mux MUX1(.addr(addr),
                .sel(ad_sel),
                .ir_ad(ir_ad),
                .pc_ad(pc_ad));

  pc PC1(.pc_addr(pc_ad),
         .clk(clk),
         .rst(rst),
         .en(PC_en));

  acc ACCUM1(.out(acc_out),
             .in(alu_out),
             .en(acc_en),
             .clk(clk),
             .rst(rst));

  alu ALU1(.alu_out(alu_out),
           .alu_in(data),
           .acc(acc_out),
           .op(ins));

  reg32 REG1(.in(alu_out),
              .data(data),
              .write(write_r),
              .read(read_r),
              .addr({ins,reg_ad}),
              .clk(clk));

  ir IR1(.data(data),
         .fetch(fetch),
         .clk(clk),
         .rst(rst),
         .ins(ins),
         .ad1(reg_ad),
         .ad2(ir_ad));

  controller CONTROLLER1(.ins(ins),
                         .clk(clk),
                         .rst(rst),
                         .write_r(write_r),
                         .read_r(read_r),
                         .PC_en(PC_en),
                         .fetch(fetch),
                         .acc_en(acc_en),
                         .ram_en(ram_en),
                         .rom_en(rom_en),
                         .ram_write(ram_write),
                         .ram_read(ram_read),
                         .rom_read(rom_read),
                         .ad_sel(ad_sel));
endmodule
