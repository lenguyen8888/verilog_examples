// controller.sv
//
// This file is for HMC E85A Lab 5.
// Place controller.tv in same computer directory as this file to test your multicycle controller.
//
// Starter code last updated by Ben Bracker (bbracker@hmc.edu) 1/14/21
// - added opcodetype enum
// - updated testbench and hash generator to accomodate don't cares as expected outputs
// Solution code by ________ (________) ________

typedef enum logic [6:0] {
  r_type_op = 7'b0110011,
  i_type_alu_op = 7'b0010011,
  lw_op = 7'b0000011,
  sw_op = 7'b0100011,
  beq_op = 7'b1100011,
  jal_op = 7'b1101111
} opcodetype;

module controller (
    input  logic            clk,
    input  logic            reset,
    input  opcodetype       op,
    input  logic      [2:0] funct3,
    input  logic            funct7b5,
    input  logic            Zero,
    output logic      [1:0] ImmSrc,
    output logic      [1:0] ALUSrcA, ALUSrcB,
    output logic      [1:0] ResultSrc,
    output logic            AdrSrc,
    output logic      [2:0] ALUControl,
    output logic            IRWrite, PCWrite,
    output logic            RegWrite, MemWrite
);
  typedef enum {
    S0_Fetch,  S1_Decode,
    S2_MemAdr, S6_ExecuteR,
    S8_ExecuteI,
    S9_JAL,
    S10_BEQ,
    S3_MemRead,
    S5_MemWrite,
    S7_ALUWB,
    S4_MemWB
  } State_enum;
  State_enum stateVar;

  always_ff @(clk) begin : stateDecode
    if (reset)
      stateVar <= S0_Fetch;
    else
    case (stateVar)
      S0_Fetch: begin
        stateVar <= S1_Decode;
      end
      S1_Decode: begin
        if(op == lw_op || op == sw_op)
          stateVar <= S2_MemAdr;
        else if(op == r_type_op)
          stateVar <= S6_ExecuteR;
        else if(op == i_type_alu_op)
          stateVar <= S8_ExecuteI;
        else if(op == jal_op)
          stateVar <= S9_JAL;
        else if(op == beq_op)
          stateVar <= S10_BEQ;
      end 
      S2_MemAdr: begin
        if(op == lw_op)
          stateVar <= S3_MemRead;
        else if(op == sw_op)
          stateVar <= S5_MemWrite;
      end
      S6_ExecuteR, S8_ExecuteI, S9_JAL: begin
        stateVar <= S7_ALUWB;
      end
      S10_BEQ, S4_MemWB, S5_MemWrite, S7_ALUWB: begin
        stateVar <= S0_Fetch;
      end
      S3_MemRead: begin
        stateVar <= S4_MemWB;
      end
      default: begin
        stateVar <= S0_Fetch;
      end
    endcase
  end

  logic Branch, PCUpdate;
  assign PCWrite = PCUpdate | Zero & Branch;

  logic [1:0] ALUOp;

  always_comb begin : outputDecode
    ImmSrc    = '0;
    ALUSrcA   = '0;
    ALUSrcB   = '0;
    ResultSrc = '0;
    AdrSrc    = '0;
    ALUOp     = '0;
    IRWrite   = '0;
    RegWrite  = '0;
    MemWrite  = '0;
    case (stateVar)
      S0_Fetch: begin
        AdrSrc = '0;
        IRWrite = 1;
        ALUSrcA = 'b00;
        ALUSrcB = 'b10;
        ALUOp   = 'b00;
      end
      S1_Decode: begin
        ALUSrcA = 'b01;
        ALUSrcB = 'b01;
        ALUOp   = 'b00;
      end
      default: begin
        
      end
    endcase
  end
endmodule

module testbench ();

  logic            clk;
  logic            reset;

  opcodetype       op;
  logic      [2:0] funct3;
  logic            funct7b5;
  logic            Zero;
  logic      [1:0] ImmSrc;
  logic [1:0] ALUSrcA, ALUSrcB;
  logic [1:0] ResultSrc;
  logic       AdrSrc;
  logic [2:0] ALUControl;
  logic IRWrite, PCWrite;
  logic RegWrite, S5_MemWrite;

  logic [31:0] vectornum, errors;
  logic [39:0] testvectors[10000:0];

  logic        new_error;
  logic [15:0] expected;
  logic [ 6:0] hash;


  // instantiate device to be tested
  controller dut (
      clk,
      reset,
      op,
      funct3,
      funct7b5,
      Zero,
      ImmSrc,
      ALUSrcA,
      ALUSrcB,
      ResultSrc,
      AdrSrc,
      ALUControl,
      IRWrite,
      PCWrite,
      RegWrite,
      S5_MemWrite
  );

  // generate clock
  always begin
    clk = 1;
    #5;
    clk = 0;
    #5;
  end

  // at start of test, load vectors and pulse reset
  initial begin
    $readmemb("controller.tv", testvectors);
    vectornum = 0;
    errors = 0;
    hash = 0;
    reset = 1;
    #22;
    reset = 0;
  end

  // apply test vectors on rising edge of clk
  always @(posedge clk) begin
    #1;
    {op, funct3, funct7b5, Zero, expected} = testvectors[vectornum];
  end

  // check results on falling edge of clk
  always @(negedge clk)
    if (~reset) begin  // skip cycles during reset
      new_error = 0;

      if ((ImmSrc !== expected[15:14]) && (expected[15:14] !== 2'bxx)) begin
        $display("   ImmSrc = %b      Expected %b", ImmSrc, expected[15:14]);
        new_error = 1;
      end
      if ((ALUSrcA !== expected[13:12]) && (expected[13:12] !== 2'bxx)) begin
        $display("   ALUSrcA = %b     Expected %b", ALUSrcA, expected[13:12]);
        new_error = 1;
      end
      if ((ALUSrcB !== expected[11:10]) && (expected[11:10] !== 2'bxx)) begin
        $display("   ALUSrcB = %b     Expected %b", ALUSrcB, expected[11:10]);
        new_error = 1;
      end
      if ((ResultSrc !== expected[9:8]) && (expected[9:8] !== 2'bxx)) begin
        $display("   ResultSrc = %b   Expected %b", ResultSrc, expected[9:8]);
        new_error = 1;
      end
      if ((AdrSrc !== expected[7]) && (expected[7] !== 1'bx)) begin
        $display("   AdrSrc = %b       Expected %b", AdrSrc, expected[7]);
        new_error = 1;
      end
      if ((ALUControl !== expected[6:4]) && (expected[6:4] !== 3'bxxx)) begin
        $display("   ALUControl = %b Expected %b", ALUControl, expected[6:4]);
        new_error = 1;
      end
      if ((IRWrite !== expected[3]) && (expected[3] !== 1'bx)) begin
        $display("   IRWrite = %b      Expected %b", IRWrite, expected[3]);
        new_error = 1;
      end
      if ((PCWrite !== expected[2]) && (expected[2] !== 1'bx)) begin
        $display("   PCWrite = %b      Expected %b", PCWrite, expected[2]);
        new_error = 1;
      end
      if ((RegWrite !== expected[1]) && (expected[1] !== 1'bx)) begin
        $display("   RegWrite = %b     Expected %b", RegWrite, expected[1]);
        new_error = 1;
      end
      if ((S5_MemWrite !== expected[0]) && (expected[0] !== 1'bx)) begin
        $display("   S5_MemWrite = %b     Expected %b", S5_MemWrite, expected[0]);
        new_error = 1;
      end

      if (new_error) begin
        $display("Error on vector %d: inputs: op = %h funct3 = %h funct7b5 = %h", vectornum, op,
                 funct3, funct7b5);
        errors = errors + 1;
      end
      vectornum = vectornum + 1;
      hash = hash ^ {ImmSrc&{2{expected[15:14]!==2'bxx}}, ALUSrcA&{2{expected[13:12]!==2'bxx}}} ^ {ALUSrcB&{2{expected[11:10]!==2'bxx}}, ResultSrc&{2{expected[9:8]!==2'bxx}}} ^ {AdrSrc&{expected[7]!==1'bx}, ALUControl&{3{expected[6:4]!==3'bxxx}}} ^ {IRWrite&{expected[3]!==1'bx}, PCWrite&{expected[2]!==1'bx}, RegWrite&{expected[1]!==1'bx}, S5_MemWrite&{expected[0]!==1'bx}};
      hash = {hash[5:0], hash[6] ^ hash[5]};
      if (testvectors[vectornum] === 40'bx) begin
        $display("%d tests completed with %d errors", vectornum, errors);
        $display("hash = %h", hash);
        $stop;
      end
    end
endmodule

