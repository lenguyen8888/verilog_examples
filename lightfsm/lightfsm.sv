module lightfsm(input  logic clk,
                input  logic reset,
                input  logic left, right,
                output logic la, lb, lc, ra, rb, rc);

  // put your logic here

  typedef logic [2:0] stateVal_t;
  stateVal_t state;

  const stateVal_t LEFT_START  = 1;
  const stateVal_t LEFT_END    = 3;
  const stateVal_t RIGHT_START = 4;
  const stateVal_t RIGHT_END   = 6;

  always_ff @(posedge clk)
  begin
    if(reset)
      state <= 0;
    else
      case(state)
        0: begin
           if(left & ~right)
             state <= LEFT_START;
           else if(right & !left)
             state <= RIGHT_START;
        end
        1, 2:
          state <= state + 1;
        3:
          state <= 0;
        4, 5:
          state <= state + 1;
        6:
          state <= 0;
        default:
          state <= 0;
      endcase
  end

  always_comb
  begin
    case(state)
        0: begin
            {lc, lb, la} = 'b000;
            {rc, rb, ra} = 'b000;
        end
        1: begin
            {lc, lb, la} = 'b001;
            {rc, rb, ra} = 'b000;
        end
        2: begin
            {lc, lb, la} = 'b011;
            {rc, rb, ra} = 'b000;
        end
        3: begin
            {lc, lb, la} = 'b111;
            {rc, rb, ra} = 'b000;
        end
        4: begin
            {lc, lb, la} = 'b000;
            {rc, rb, ra} = 'b001;
        end
        5: begin
            {lc, lb, la} = 'b000;
            {rc, rb, ra} = 'b011;
        end
        6: begin
            {lc, lb, la} = 'b000;
            {rc, rb, ra} = 'b111;
        end
        default: begin
            {lc, lb, la} = 'b000;
            {rc, rb, ra} = 'b000;
        end
    endcase
  end
  
endmodule

module testbench(); 
  logic        clk, reset;
  logic        left, right, la, lb, lc, ra, rb, rc;
  logic [5:0]  expected;
  logic [6:0]  hash;
  logic [31:0] vectornum, errors;
  logic [7:0]  testvectors[10000:0];

  // instantiate device under test 
  lightfsm dut(clk, reset, left, right, la, lb, lc, ra, rb, rc); 

  // generate clock 
  always 
    begin
      clk=1; #5; clk=0; #5; 
    end 

  // at start of test, load vectors and pulse reset
  initial 
    begin
      $readmemb("lightfsm.tv", testvectors); 
      vectornum = 0; errors = 0; hash = 0; reset = 1; #22; reset = 0; 
    end 

  // apply test vectors on rising edge of clk 
  always @(posedge clk) 
    begin
      #1; {left, right, expected} = testvectors[vectornum]; 
    end 

  // check results on falling edge of clk 
  always @(negedge clk) 
    if (~reset) begin    // skip during reset
      if ({la, lb, lc, ra, rb, rc} !== expected) begin // check result 
        $display("Error: inputs = %b", {left, right});
        $display(" outputs = %b %b %b %b %b %b (%b expected)", 
          la, lb, lc, ra, rb, rc, expected); 
        errors = errors + 1; 
      end
      vectornum = vectornum + 1;
      hash = hash ^ {la, lb, lc, ra, rb, rc};
      hash = {hash[5:0], hash[6] ^ hash[5]};
      if (testvectors[vectornum] === 8'bx) begin 
        $display("%d tests completed with %d errors", vectornum, errors); 
        $display("Hash: %h", hash);
        $stop; 
      end 
    end 
endmodule 
 
