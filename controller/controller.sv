// controller.sv
//
// This file is for HMC E85A Lab 5.
// Place controller.tv in same computer directory as this file to test your multicycle controller.
//
// Starter code last updated by Ben Bracker (bbracker@hmc.edu) 1/14/21
// - added opcodetype enum
// - updated testbench and hash generator to accomodate don't cares as expected outputs
// Solution code by Nguyen   (________) ________

typedef enum logic[6:0] {r_type_op=7'b0110011, i_type_alu_op=7'b0010011, lw_op=7'b0000011, sw_op=7'b0100011, beq_op=7'b1100011, jal_op=7'b1101111} opcodetype;

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

  assign PCWrite = PCUpdate | Branch & Zero;

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

module testbench();

  logic        clk;
  logic        reset;
  
  opcodetype  op;
  logic [2:0] funct3;
  logic       funct7b5;
  logic       Zero;
  logic [1:0] ImmSrc;
  logic [1:0] ALUSrcA, ALUSrcB;
  logic [1:0] ResultSrc;
  logic       AdrSrc;
  logic [2:0] ALUControl;
  logic       IRWrite, PCWrite;
  logic       RegWrite, MemWrite;
  
  logic [31:0] vectornum, errors;
  logic [39:0] testvectors[10000:0];
  
  logic        new_error;
  logic [15:0] expected;
  logic [6:0]  hash;


  // instantiate device to be tested
  controller dut(clk, reset, op, funct3, funct7b5, Zero,
                 ImmSrc, ALUSrcA, ALUSrcB, ResultSrc, AdrSrc, ALUControl, IRWrite, PCWrite, RegWrite, MemWrite);
  
  // generate clock
  always 
    begin
      clk = 1; #5; clk = 0; #5;
    end

  // at start of test, load vectors and pulse reset
  initial
    begin
      $readmemb("controller.tv", testvectors);
      vectornum = 0; errors = 0; hash = 0;
      reset = 1; #22; reset = 0;
    end
	 
  // apply test vectors on rising edge of clk
  always @(posedge clk)
    begin
      #1; {op, funct3, funct7b5, Zero, expected} = testvectors[vectornum];
    end

  // check results on falling edge of clk
  always @(negedge clk)
    if (~reset) begin // skip cycles during reset
      new_error=0; 

      if ((ImmSrc!==expected[15:14])&&(expected[15:14]!==2'bxx))  begin
        $display("   ImmSrc = %b      Expected %b", ImmSrc,     expected[15:14]);
        new_error=1;
      end
      if ((ALUSrcA!==expected[13:12])&&(expected[13:12]!==2'bxx)) begin
        $display("   ALUSrcA = %b     Expected %b", ALUSrcA,    expected[13:12]);
        new_error=1;
      end
      if ((ALUSrcB!==expected[11:10])&&(expected[11:10]!==2'bxx)) begin
        $display("   ALUSrcB = %b     Expected %b", ALUSrcB,    expected[11:10]);
        new_error=1;
      end
      if ((ResultSrc!==expected[9:8])&&(expected[9:8]!==2'bxx))   begin
        $display("   ResultSrc = %b   Expected %b", ResultSrc,  expected[9:8]);
        new_error=1;
      end
      if ((AdrSrc!==expected[7])&&(expected[7]!==1'bx))           begin
        $display("   AdrSrc = %b       Expected %b", AdrSrc,     expected[7]);
        new_error=1;
      end
      if ((ALUControl!==expected[6:4])&&(expected[6:4]!==3'bxxx)) begin
        $display("   ALUControl = %b Expected %b", ALUControl, expected[6:4]);
        new_error=1;
      end
      if ((IRWrite!==expected[3])&&(expected[3]!==1'bx))          begin
        $display("   IRWrite = %b      Expected %b", IRWrite,    expected[3]);
        new_error=1;
      end
      if ((PCWrite!==expected[2])&&(expected[2]!==1'bx))          begin
        $display("   PCWrite = %b      Expected %b", PCWrite,    expected[2]);
        new_error=1;
      end
      if ((RegWrite!==expected[1])&&(expected[1]!==1'bx))         begin
        $display("   RegWrite = %b     Expected %b", RegWrite,   expected[1]);
        new_error=1;
      end
      if ((MemWrite!==expected[0])&&(expected[0]!==1'bx))         begin
        $display("   MemWrite = %b     Expected %b", MemWrite,   expected[0]);
        new_error=1;
      end

      if (new_error) begin
        $display("Error on vector %d: inputs: op = %h funct3 = %h funct7b5 = %h", vectornum, op, funct3, funct7b5);
        errors = errors + 1;
      end
      vectornum = vectornum + 1;
      hash = hash ^ {ImmSrc&{2{expected[15:14]!==2'bxx}}, ALUSrcA&{2{expected[13:12]!==2'bxx}}} ^ {ALUSrcB&{2{expected[11:10]!==2'bxx}}, ResultSrc&{2{expected[9:8]!==2'bxx}}} ^ {AdrSrc&{expected[7]!==1'bx}, ALUControl&{3{expected[6:4]!==3'bxxx}}} ^ {IRWrite&{expected[3]!==1'bx}, PCWrite&{expected[2]!==1'bx}, RegWrite&{expected[1]!==1'bx}, MemWrite&{expected[0]!==1'bx}};
      hash = {hash[5:0], hash[6] ^ hash[5]};
      if (testvectors[vectornum] === 40'bx) begin 
        $display("%d tests completed with %d errors", vectornum, errors);
	      $display("hash = %h", hash);
        $stop;
      end
    end
endmodule

