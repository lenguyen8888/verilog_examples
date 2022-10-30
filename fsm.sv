module fsm(input  logic clk, reset,
           input  logic a,
           output logic q);
           
    // your code goes here
    logic [1:0] state;
    always_ff @(posedge clk)
    begin
        if(reset)
            state <= '0;
        else begin
            state[1] <= ~a & state[1] | ~a & ~state[0] | state[1] & ~state[0];
            state[0] <= ~state[0] & ~state[1] | (a & ~(state[0] & state[1]));
        end
    end
    assign q = state[1] ^ state[0];
endmodule


module testbench(); 
  logic        clk, reset;
  logic        a, q, qexpected;
  logic [6:0]  hash;
  logic [31:0] vectornum, errors;
  logic [1:0]  testvectors[10000:0];

  // instantiate device under test 
  fsm dut(clk, reset, a, q);

  // generate clock 
  always 
    begin
      clk=1; #5; clk=0; #5; 
    end 

  // at start of test, load vectors and pulse reset
  initial 
    begin
      $readmemb("fsm.tv", testvectors); 
      vectornum = 0; errors = 0; hash = 0; reset = 1; #22; reset = 0;
    end 

  // apply test vectors on rising edge of clk 
  always @(posedge clk) 
    begin
      #1; {a, qexpected} = testvectors[vectornum]; 
    end 

  // check results on falling edge of clk 
  always @(negedge clk) begin
    if (!reset) begin
//		if (q !== qexpected) begin // check result 
//		  $display("Error: a = %b", a);
//		  $display(" q = %b (%b expected)", q, qexpected);
//		  errors = errors + 1; 
//		end
		vectornum = vectornum + 1;
		hash = hash ^ q;
		hash = {hash[5:0], hash[6] ^ hash[5]};
	end
    if (testvectors[vectornum] === 2'bx) begin 
//      $display("%d tests completed with %d errors", vectornum, errors); 
      $display("Hash: %h", hash);
      $stop; 
    end 
  end 
endmodule 
 
