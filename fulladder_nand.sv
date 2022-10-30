// fulladder_nand.sv
// David_Harris@hmc.edu 27 August 2020

// full adder module using only nand gates
module fulladder(input  logic a, b, cin,
                 output logic cout, s);
              
  logic n1, n2, n3;
  logic x1, x2, x3, x4, x5, x6, x7;
    
  // logic for carry out: cout = a&b | a&cin | b&cin
  // Use DeMorgan's Law to replace AND-OR with NAND-NAND
  nand g1(n1, a, b);
  nand g2(n2, a, cin);
  nand g3(n3, b, cin);
  nand g4(cout, n1, n2, n3);
  
  // logic for sum: s = a ^ b ^ cin
  
  // these four nand gates compute x4 = a ^ b
  nand gx1(x1, a, b);
  nand gx2(x2, a, x1);
  nand gx3(x3, b, x1);
  nand gx4(x4, x2, x3);
  
  // These four nand gates compute s = x4 ^ cin = (a ^ b) ^ cin
  nand gx5(x5, x4, cin);
  nand gx6(x6, x4, x5);
  nand gx7(x7, cin, x5);
  nand gx8(s, x6, x7);
endmodule

module testbench #(parameter VECTORSIZE=5);
  logic                   clk;
  logic                   a, b, cin, cout, s, coutexpected, sexpected;
  logic [6:0]             hash;
  logic [31:0]            vectornum, errors;
  logic [VECTORSIZE-1:0]  testvectors[1000:0];
  logic [VECTORSIZE-1:0]  DONE = 'bx;
  
  // instantiate device under test
  fulladder dut(a, b, cin, cout, s);
  
  // generate clock
  always begin
   clk = 1; #5; clk = 0; #5; 
  end
  
  // at start of test, load vectors and pulse reset
  initial begin
    $readmemb("fulladder.tv", testvectors);
    vectornum = 0; errors = 0;
    hash = 0;
  end
    
  // apply test vectors on rising edge of clk
  always @(posedge clk) begin
    #1; {a, b, cin, coutexpected, sexpected} = testvectors[vectornum];
  end
  
  // Check results on falling edge of clock.
  always @(negedge clk)begin
     if (s !== sexpected || cout !== coutexpected) begin // result is bad
      $display("%t Error: inputs=%b", $time, {a, b, cin});
      $display(" outputs = %b (%b expected)", {cout, s}, {coutexpected, sexpected});
      errors = errors+1;
    end
    // In any event, increment the count of vectors and compute hash
    vectornum = vectornum + 1;
    hash = hash ^ {cout, s};
    hash = {hash[5:0], hash[6] ^ hash[5]};
    if (testvectors[vectornum] === DONE) begin
      #2;
      $display("%d tests completed with %d errors", vectornum, errors);
      $display("Hash: %h", hash);
      $stop;
    end
  end
endmodule

