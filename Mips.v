//mux
module mux (In1, In2, Sel, Out);
   input [31:0] In1,In2;
   input Sel;
   output reg [31:0] Out;
   always @(*) begin
      case (Sel)
         0: Out <=In1;
         default: Out <=In2;
       endcase
   end
endmodule

//program counter
module PC (Clk, Reset, PCin, PCout);
   input Clk, Reset;
   input [31:0] PCin;
   output reg [31:0] PCout;
   always @(posedge Clk or posedge Reset) begin
      if (Reset)
         PCout <= 32'h00000000;
      else
         PCout <= PCin + 32'h00000004;
   end
endmodule

//MIPS 4K Instruction Memory 
module IMemory (Address, Instruction);
   //4096 bits of memory (1024 32-bit instructions)
   input [9:0] Address;
   output [31:0] Instruction;
   reg [31:0] IMem [0:1023];
   assign Instruction = IMem[Address];
endmodule

//MIPS Instruction Register
module IRegister (Instruction, IR);
   input [31:0] Instruction;
   output [31:0] IR;
   assign IR = Instruction;
endmodule

//MIPS 16K Data Memory
module DMemory (Address, WriteData, ReadData, MemWrite, MemRead);
   //16384 bits of memory (4096 32-bit words)
   input [11:0] Address;
   input [31:0] WriteData;
   output [31:0] ReadData;
   input MemWrite, MemRead;
   reg [31:0] DataMem [0:4095];
   assign ReadData = MemRead ? DataMem[Address] : 0;
   always @(MemWrite)
      if (MemWrite)
         DataMem[Address] = WriteData;
endmodule

//MIPS Register file
module registerfile (
   input RegWrite, Clock,
   input [4:0] Read1, Read2, WriteReg,
   input [31:0] WriteData,
   output [31:0] Data1, Data2
);
   reg [31:0] RF [31:0];
   assign Data1 = RF[Read1];
   assign Data2 = RF[Read2];
   always @(posedge Clock)
      if (RegWrite)
         RF[WriteReg] = WriteData;
endmodule

//MIPS ALU Control
module ALUControl (ALUOp, Funct, ALUctl);
   input [1:0] ALUOp;
   input [5:0] Funct;
   output reg ALUctl;

   //if (ALUOp=0)->add  (ALUOp=1)->sub
   always @(ALUOp, Funct) begin
      case (ALUOp)
         00: ALUctl <= 1'b0;  
         01: ALUctl <= 1'b1;    
         10: begin // R-type instructions
                case(Funct)
                    6'b100000: ALUctl <= 1'b0;  // ADD
                    6'b100010: ALUctl <= 1'b1;  // SUB
                    default: ALUctl <= 1'b0;
                endcase
            end
         default: ALUctl <= 1'b0;
      endcase
   end
endmodule

//MIPS ALU
module MIPSALU(ALUctl, A, B, ALUOut, zero_flag);
   input ALUctl;
   input [31:0] A, B;
   output reg [31:0] ALUOut;
   output zero_flag;
   assign zero_flag = (ALUOut == 0);
   //if (ALUctl=0)->add  (ALUctl=1)->sub
   always @(ALUctl, A, B) begin
      case (ALUctl)
         0: ALUOut <= A + B;
         1: ALUOut <= A - B;
         default: ALUOut <= 0;
      endcase
   end
endmodule


//MIPS Control Unit
module ControlUnit (OpCode, RegDst, Branch, branch_not_equal, MemRead, MemtoReg, ALUOp, MemWrite, ALUSrc, RegWrite);
   input [5:0] OpCode;
   output reg [1:0] ALUOp; 
   output RegDst, branch_not_equal, Branch, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite;
   //35=6'b100011, 43=6'b101011, 4=6'b000100, 5=6'b000101
   assign RegDst = (OpCode == 6'b000000) ? 1 : 0;
   assign MemtoReg = (OpCode == 6'b100011) ? 1 : 0;
   assign MemRead = (OpCode == 6'b100011) ? 1 : 0;
   assign MemWrite = (OpCode == 6'b101011) ? 1 : 0; // 1:add , 0:sub 
   assign Branch = (OpCode == 6'b000100) ? 1 : 0;
   assign branch_not_equal = (OpCode == 6'b000101) ? 1 : 0; 
   //ALUSrc
   assign ALUSrc = (OpCode == 6'b100011 || OpCode == 6'b101011) ? 1 : 0;
   //RegWrite
   assign RegWrite = (OpCode == 6'b000000 || OpCode == 6'b100011) ? 1 : 0;
   //ALUOp
   always @(*) begin
        case(OpCode)
            6'b000000: ALUOp = 2'b10; // R-type instructions
            6'b100011: ALUOp = 2'b00; // LW
            6'b101011: ALUOp = 2'b00; // SW
            6'b000100: ALUOp = 2'b01; // beq
            6'b000101: ALUOp = 2'b01; // bne
            default: ALUOp = 2'b00;
        endcase
    end
  
endmodule

//MIPS Sign Extend
module SignExtend (In, Out);
   input [15:0] In;
   output [31:0] Out;
   assign Out = {{16{In[15]}}, In}; 
endmodule 

//adder
module Adder (A, B, Sum);
   input [31:0] A, B;
   output [31:0] Sum;
   assign Sum = A + B;
endmodule

//MIPS Shift Left 2
module ShiftLeft2 (In, Out);
   input [31:0] In;
   output [31:0] Out;
   assign Out = In << 2;
endmodule

