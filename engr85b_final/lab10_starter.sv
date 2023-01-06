// lab10_starter.sv


///////////////////////////////////////////////////////////////
// testbench
//
// Expect simulator to print "Simulation succeeded"
// when the value 25 (0x19) is written to address 100 (0x64)
///////////////////////////////////////////////////////////////

module testbench();

  logic        clk;
  logic        reset;

  logic [31:0] WriteData, DataAdr;
  logic        MemWrite;
  logic [31:0] hash;

  // instantiate device to be tested
  top dut(clk, reset, WriteData, DataAdr, MemWrite);
  
  // initialize test
  initial
    begin
      hash <= 0;
      reset <= 1; # 22; reset <= 0;
    end

  // generate clock to sequence tests
  always
    begin
      clk <= 1; # 5; clk <= 0; # 5;
    end

  // check results
  always @(negedge clk)
    begin
      if(MemWrite) begin
        //if(DataAdr === 100 & WriteData === 25) begin
        if(DataAdr === 44 & WriteData === 8) begin
          $display("Simulation succeeded");
 	   	  $display("hash = %h", hash);
          $stop;
        end else if (DataAdr !== 96) begin
          $display("Simulation failed");
          $stop;
        end
      end
    end

  // Make 32-bit hash of instruction, PC, ALU
  always @(negedge clk)
    if (~reset) begin
      hash = hash ^ dut.rvmulti.dp.Instr ^ dut.rvmulti.dp.PC;
      if (MemWrite) hash = hash ^ WriteData;
      hash = {hash[30:0], hash[9] ^ hash[29] ^ hash[30] ^ hash[31]};
    end

endmodule

///////////////////////////////////////////////////////////////
// top
//
// Instantiates multicycle RISC-V processor and memory
///////////////////////////////////////////////////////////////

module top(input  logic        clk, reset, 
           output logic [31:0] WriteData, DataAdr, 
           output logic        MemWrite);

  logic [31:0] ReadData;
  
  // instantiate processor and memories
  riscvmulti rvmulti(clk, reset, MemWrite, DataAdr, 
                     WriteData, ReadData);
  mem mem(clk, MemWrite, DataAdr, WriteData, ReadData);
endmodule

///////////////////////////////////////////////////////////////
// mem
//
// Single-ported RAM with read and write ports
// Initialized with machine language program
///////////////////////////////////////////////////////////////

module mem(input  logic        clk, we,
           input  logic [31:0] a, wd,
           output logic [31:0] rd);

  logic [31:0] RAM[63:0];
  
  initial
  begin
      //$readmemh("riscvtest.txt",RAM);
      $readmemh("riscvtest_final.txt",RAM);
  end


  assign rd = RAM[a[31:2]]; // word aligned

  always_ff @(posedge clk)
    if (we) RAM[a[31:2]] <= wd;
endmodule

///////////////////////////////////////////////////////////////
// riscvmulti
//
// Multicycle RISC-V microprocessor
///////////////////////////////////////////////////////////////

module riscvmulti(input  logic        clk, reset,
                  output logic        MemWrite,
                  output logic [31:0] Adr, WriteData,
                  input  logic [31:0] ReadData);

  // Your code goes here
  // Instantiate controller (from lab 5) and datapath (new for this lab)
  logic [6:0] op_in;
  logic [2:0] funct3;
  logic       funct7b5;
  logic       Zero;
  logic [1:0] ImmSrc;
  logic [1:0] ALUSrcA, ALUSrcB;
  logic [1:0] ResultSrc;
  logic       AdrSrc;
  logic [2:0] ALUControl;
  logic       IRWrite, PCWrite;
  logic       RegWrite;

  logic [31:0] Instr;

  assign op_in     = Instr[6:0];
  assign funct3    = Instr[14:12];
  assign funct7b5  = Instr[30];

  controller m_con (
         .clk        (clk       )
        ,.reset      (reset     )
        ,.op_in      (op_in     )
        ,.funct3     (funct3    )
        ,.funct7b5   (funct7b5  )
        ,.Zero       (Zero      )
        ,.ImmSrc     (ImmSrc    )
        ,.ALUSrcA    (ALUSrcA   )
        ,.ALUSrcB    (ALUSrcB   )
        ,.ResultSrc  (ResultSrc )
        ,.AdrSrc     (AdrSrc    )
        ,.ALUControl (ALUControl)
        ,.IRWrite    (IRWrite   )
        ,.PCWrite    (PCWrite   )
        ,.RegWrite   (RegWrite  )
        ,.MemWrite   (MemWrite  )
        );
  // Can use either datapath_dataflow or datapath_struct in this design
  // datapath_dataflow is the initial implementation, later converted to
  // structural design in datapath_struct
  datapath_struct dp (
         .clk        (clk       )
        ,.reset      (reset     )
        ,.PCWrite    (PCWrite   )
        ,.AdrSrc     (AdrSrc    )
        ,.MemWrite   (MemWrite  )
        ,.IRWrite    (IRWrite   )
        ,.ResultSrc  (ResultSrc )
        ,.ALUControl (ALUControl)
        ,.ALUSrcA    (ALUSrcA   )
        ,.ALUSrcB    (ALUSrcB   )
        ,.ImmSrc     (ImmSrc    )
        ,.RegWrite   (RegWrite  )

        ,.Zero       (Zero      )

        ,.WriteData  (WriteData )
        ,.ReadData   (ReadData  )
        ,.DataAdr    (Adr       )
        ,.Instr      (Instr     )
  );
endmodule

///////////////////////////////////////////////////////////////
// Your modules go here
///////////////////////////////////////////////////////////////

// controller.sv
//
// This file is for HMC E85A Lab 5.
// Place controller.tv in same computer directory as this file to test your multicycle controller.
//
// Starter code last updated by Ben Bracker (bbracker@hmc.edu) 1/14/21
// - added opcodetype enum
// - updated testbench and hash generator to accomodate don't cares as expected outputs
// Solution code by Nguyen   (________) ________

module controller(input  logic       clk,
                  input  logic       reset,  
                  input  logic[6:0]  op_in,
                  input  logic [2:0] funct3,
                  input  logic       funct7b5,
                  input  logic       Zero,
                  output logic [1:0] ImmSrc,
                  output logic [1:0] ALUSrcA, ALUSrcB,
                  output logic [1:0] ResultSrc, 
                  output logic       AdrSrc,
                  output logic [2:0] ALUControl,
                  output logic       IRWrite, PCWrite, 
                  output logic       RegWrite, MemWrite);

typedef enum logic[6:0] {r_type_op=7'b0110011, i_type_alu_op=7'b0010011, lw_op=7'b0000011, sw_op=7'b0100011, beq_op=7'b1100011, jal_op=7'b1101111} opcodetype;

  typedef enum { 
    S0_Fetch
    ,S1_Decode

    ,S2_MemAdr ,S3_MemRead ,S4_MemWB

    ,S5_MemWrite ,S6_ExecuteR ,S7_ALUWB ,S8_ExecuteI
    ,S9_JAL ,S10_BEQ
  } stateEnum;
  stateEnum stateR;

  opcodetype op;

  assign op = opcodetype'(op_in);
  logic PCUpdate, Branch;

  logic [1:0] ALUOp;


  logic branch_hit;
  assign branch_hit = funct3 == 'b000 ? Zero :
                      funct3 == 'b001 ? ~Zero :
                      1'b0
                      ;
  assign PCWrite = PCUpdate | Branch & branch_hit;


  aludecoder dec (.ALUOp(ALUOp)
                  ,.funct3(funct3)
                  ,.op_5(op_in[5])
                  ,.funct7_5(funct7b5)
                  ,.ALUControl(ALUControl));

  always_comb begin
    case(op)
        lw_op:     ImmSrc <= 'b00;
        sw_op:     ImmSrc <= 'b01;
        r_type_op: ImmSrc <= 'b00;
        beq_op:    ImmSrc <= 'b10;
        i_type_alu_op:  ImmSrc <= 'b00;
        jal_op:    ImmSrc <= 'b11;
        default : ImmSrc <= 'x;
    endcase
  end

  always_comb begin
      ALUSrcA      = '0;
      ALUSrcB      = '0;
      ResultSrc    = '0;
      AdrSrc       = '0;

      IRWrite      = '0;
      PCUpdate     = '0;
      Branch       = '0;
      RegWrite     = '0;
      MemWrite     = '0;
      case(stateR)
        S0_Fetch: begin
          AdrSrc     = 0;
          IRWrite     = 1;
          ALUSrcA     = 'b00;
          ALUSrcB     = 'b10;
          ALUOp       = 'b00;
          ResultSrc   = 'b10;
          PCUpdate    = 1;
        end
        S1_Decode: begin
          ALUSrcA     = 'b01;
          ALUSrcB     = 'b01;
          ALUOp       = 'b00;
        end
        S2_MemAdr: begin
          ALUSrcA     = 'b10;
          ALUSrcB     = 'b01;
          ALUOp       = 'b00;
        end
        S3_MemRead: begin
          ResultSrc   = 'b00;
          AdrSrc      = 1;
        end
        S4_MemWB: begin
          ResultSrc   = 'b01;
          RegWrite    = 1;
        end
        S5_MemWrite: begin
          ResultSrc   = 'b00;
          AdrSrc      = 1;
          MemWrite    = 1;
        end
        S6_ExecuteR: begin
          ALUSrcA     = 'b10;
          ALUSrcB     = 'b00;
          ALUOp       = 'b10;
        end
        S7_ALUWB: begin
          ResultSrc   = 'b00;
          RegWrite    = 1;
        end
        S8_ExecuteI: begin
          ALUSrcA     = 'b10;
          ALUSrcB     = 'b01;
          ALUOp       = 'b10;
        end
        S9_JAL: begin
          ALUSrcA     = 'b01;
          ALUSrcB     = 'b10;
          ALUOp       = 'b00;

          ResultSrc   = 'b00;
          PCUpdate    = 1;
        end
        S10_BEQ: begin
          ALUSrcA     = 'b10;
          ALUSrcB     = 'b00;
          ALUOp       = 'b01;
          ResultSrc   = 'b00;
          Branch      = 1;
        end
      endcase
  end
  always_ff @(posedge clk)
  begin
    if(reset) begin
      stateR  <= S0_Fetch;
    end
    else begin
      case(stateR)
        S0_Fetch: begin
          stateR <= S1_Decode;
        end
        S1_Decode: begin
          if(op == lw_op || op == sw_op)
            stateR <= S2_MemAdr;
          else if(op == r_type_op)
            stateR <= S6_ExecuteR;
          else if(op == i_type_alu_op)
            stateR <= S8_ExecuteI;
          else if(op == jal_op)
            stateR <= S9_JAL;
          else if(op == beq_op)
            stateR <= S10_BEQ;
        end
        S2_MemAdr: begin
          if(op == lw_op)
            stateR <= S3_MemRead;
          else if(op == sw_op)
            stateR <= S5_MemWrite;
        end
        S3_MemRead: begin
          stateR <= S4_MemWB;
        end
        S4_MemWB: begin
          stateR <= S0_Fetch;
        end
        S5_MemWrite: begin
          stateR <= S0_Fetch;
        end
        S6_ExecuteR: begin
          stateR <= S7_ALUWB;
        end
        S7_ALUWB: begin
          stateR <= S0_Fetch;
        end
        S8_ExecuteI: begin
          stateR <= S7_ALUWB;
        end
        S9_JAL: begin
          stateR <= S7_ALUWB;
        end
        S10_BEQ: begin
          stateR <= S0_Fetch;
        end
        default: begin
          stateR <= S0_Fetch;
        end
      endcase
    end
  end
endmodule

module aludecoder(input  logic [1:0] ALUOp,
                  input  logic [2:0] funct3,
                  input  logic op_5, funct7_5,
                  output logic [2:0] ALUControl);
              
    // For Lab 2, write a structural Verilog model 
    // use and, or, not
    // do not use assign statements, always blocks, or other behavioral Verilog
    // Example syntax to access bits to make ALUControl[0] = ~funct3[0]
    //  not g1(ALUControl[0], funct3[0]);
    // This is just an example; replace this with correct logic!

    logic [2:0] ALUControl_helper;
    assign ALUControl_helper = funct3 == 'b000 ?
                                   ({op_5, funct7_5} != 'b11 ? 3'b000 : 3'b001) :
                               funct3 == 'b010 ? 3'b101 :
                               funct3 == 'b110 ? 3'b011 :
                               funct3 == 'b111 ? 3'b010 :
                               3'bxxx;

    assign ALUControl = ALUOp == 'b00 ? 3'b000 :
                        ALUOp == 'b01 ? 3'b001 :
                        ALUOp == 'b10 ? ALUControl_helper :
                        3'bxxx;
endmodule

module datapath_struct(
                input  logic        clk, reset,

                input  logic        PCWrite, AdrSrc, MemWrite, IRWrite,

                input  logic [1:0]  ResultSrc, 
                input  logic [2:0]  ALUControl,
                input  logic [1:0]  ALUSrcA, ALUSrcB,
                input  logic [1:0]  ImmSrc,
                input  logic        RegWrite,

                output logic        Zero,
                output logic [31:0] WriteData,
                input  logic [31:0] ReadData,

                output logic [31:0]  DataAdr,
                output logic [31:0]  Instr
                );

  logic [31:0] PC, PCNext;
  logic [31:0] ImmExt;
  logic [31:0] SrcA, SrcB;
  logic [31:0] Result;

  logic [31:0] ALUResult, ALUOut;

  logic [31:0] OldPC, A, RD1, RD2;
  logic [31:0] Data;

  // next PC logic
  // flopr_ld port list (clk, reset, load, d, q);
  flopr_ld #(32) pcReg    (clk, reset, PCWrite, PCNext, PC); 
  flopr_ld #(32) instrReg (clk, reset, IRWrite, ReadData, Instr); 
  flopr_ld #(32) oldPCReg (clk, reset, IRWrite, PC, OldPC); 

  // flopr port list (clk, reset, d, q);
  flopr #(32) aReg          (clk, reset, RD1, A); 
  flopr #(32) writeDataReg  (clk, reset, RD2, WriteData); 
  flopr #(32) aluOutReg     (clk, reset, ALUResult, ALUOut); 
  flopr #(32) dataReg       (clk, reset, ReadData, Data); 

  // mux2 port list (d0, d1, s, y)
  mux2 #(32) dataAdrMux (PC, Result, AdrSrc, DataAdr);

  // register file logic
  regfile     rf(clk, RegWrite, Instr[19:15], Instr[24:20], 
                 Instr[11:7], Result, RD1, RD2);
  extend      ext(Instr[31:7], ImmSrc, ImmExt);

  // ALU logic
  alu         alu(SrcA, SrcB, ALUControl, ALUResult, Zero);

  // mux3 port list (d0, d1, d2, s, y)
  mux3 #(32) srcAMux (PC, OldPC, A, ALUSrcA, SrcA);
  mux3 #(32) srcBMux (WriteData, ImmExt, 32'h4, ALUSrcB, SrcB);
  mux3 #(32) resultMux (ALUOut, Data, ALUResult, ResultSrc, Result);

  assign PCNext = Result;
endmodule

module datapath_dataflow(input  logic        clk, reset,

                input  logic        PCWrite, AdrSrc, MemWrite, IRWrite,

                input  logic [1:0]  ResultSrc, 
                input  logic [2:0]  ALUControl,
                input  logic [1:0]  ALUSrcA, ALUSrcB,
                input  logic [1:0]  ImmSrc,
                input  logic        RegWrite,

                output logic        Zero,
                output logic [31:0] WriteData,
                input  logic [31:0] ReadData,

                output logic [31:0]  DataAdr,
                output logic [31:0]  Instr
                );

  logic [31:0] PC, PCNext;
  logic [31:0] ImmExt;
  logic [31:0] SrcA, SrcB;
  logic [31:0] Result;

  logic [31:0] ALUResult, ALUOut;

  logic [31:0] OldPC, A, RD1, RD2;
  logic [31:0] Data;

  // next PC logic
  //flopr #(32) pcreg(clk, reset, PCNext, PC); 
  always_ff @(posedge clk or posedge reset)
  begin
    if(reset) begin
      PC <= '0;
      Instr <= '0;
      OldPC <= '0;
    end
    else begin
      if(PCWrite)
        PC <= PCNext;
      if(IRWrite) begin
        Instr <= ReadData;
        OldPC <= PC;
      end
    end
  end

  always_ff @(posedge clk) begin
    A <= RD1;
    WriteData <= RD2;
    ALUOut <= ALUResult;
    Data <= ReadData;
  end

  assign DataAdr = AdrSrc ? Result : PC;

  // register file logic
  regfile     rf(clk, RegWrite, Instr[19:15], Instr[24:20], 
                 Instr[11:7], Result, RD1, RD2);
  extend      ext(Instr[31:7], ImmSrc, ImmExt);

  // ALU logic
  alu         alu(SrcA, SrcB, ALUControl, ALUResult, Zero);

  assign SrcA = ALUSrcA == 'b00 ? PC :
                ALUSrcA == 'b01 ? OldPC :
                A
                ;
  assign SrcB = ALUSrcB == 'b00 ? WriteData :
                ALUSrcB == 'b01 ? ImmExt :
                32'h4
                ;
  assign Result = ResultSrc == 'b00 ? ALUOut :
                  ResultSrc == 'b01 ? Data   :
                  ALUResult
                  ;
  assign PCNext = Result;
endmodule

module regfile(input  logic        clk, 
               input  logic        we3, 
               input  logic [ 4:0] a1, a2, a3, 
               input  logic [31:0] wd3, 
               output logic [31:0] rd1, rd2);

  logic [31:0] rf[31:0];

  // three ported register file
  // read two ports combinationally (A1/RD1, A2/RD2)
  // write third port on rising edge of clock (A3/WD3/WE3)
  // register 0 hardwired to 0

  always_ff @(posedge clk)
    if (we3) rf[a3] <= wd3;	

  assign rd1 = (a1 != 0) ? rf[a1] : 0;
  assign rd2 = (a2 != 0) ? rf[a2] : 0;
endmodule

module adder(input  [31:0] a, b,
             output [31:0] y);

  assign y = a + b;
endmodule

module extend(input  logic [31:7] instr,
              input  logic [1:0]  immsrc,
              output logic [31:0] immext);
 
  always_comb
    case(immsrc) 
               // I-type 
      2'b00:   immext = {{20{instr[31]}}, instr[31:20]};  
               // S-type (stores)
      2'b01:   immext = {{20{instr[31]}}, instr[31:25], instr[11:7]}; 
               // B-type (branches)
      2'b10:   immext = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0}; 
               // J-type (jal)
      2'b11:   immext = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0}; 
      default: immext = 32'bx; // undefined
    endcase             
endmodule

module flopr #(parameter WIDTH = 8)
              (input  logic             clk, reset,
               input  logic [WIDTH-1:0] d, 
               output logic [WIDTH-1:0] q);

  always_ff @(posedge clk, posedge reset)
    if (reset) q <= 0;
    else       q <= d;
endmodule

module flopr_ld #(parameter WIDTH = 8)
              (input  logic             clk, reset,
               input  logic load,
               input  logic [WIDTH-1:0] d, 
               output logic [WIDTH-1:0] q);

  always_ff @(posedge clk, posedge reset)
    if (reset) q <= 0;
    else if(load) q <= d;
endmodule

module mux2 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, 
              input  logic             s, 
              output logic [WIDTH-1:0] y);

  assign y = s ? d1 : d0; 
endmodule

module mux3 #(parameter WIDTH = 8)
             (input  logic [WIDTH-1:0] d0, d1, d2,
              input  logic [1:0]       s, 
              output logic [WIDTH-1:0] y);

  assign y = s[1] ? d2 : (s[0] ? d1 : d0); 
endmodule

module alu(input  logic [31:0] a, b,
           input  logic [2:0]  alucontrol,
           output logic [31:0] result,
           output logic        zero);

  logic [31:0] condinvb, sum;

  assign condinvb = alucontrol[0] ? ~b : b;
  assign sum = a + condinvb + alucontrol[0];

  always_comb
    case (alucontrol)
      3'b000:  result = sum;       // add
      3'b001:  result = sum;       // subtract
      3'b010:  result = a & b;     // and
      3'b011:  result = a | b;     // or
      3'b100:  result = a << b[4:0]; // sll
      3'b101:  result = sum[31];   // slt
      default: result = 32'bx;
    endcase

  assign zero = (result == 32'b0);
endmodule

// Describe your non-leaf cells structurally
// Describe your lef cells (mux, flop, alu, etc.) behaviorally
// Exactly follow the multicycle processor diagram
// Feel free to cut and paste from riscvsingle.sv where applicable
// Remember to declare internal signals
// Be consistent with spelling and capitalization
// Be consistent with order of signals in module declarations and instantiations
// Have fun!
