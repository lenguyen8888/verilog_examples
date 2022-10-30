
module smadd(input  logic [3:0] a, b,
             output logic [3:0] y);
             
  logic [2:0] ma, mb, my, mr, mbn, mb2, mrb, mrb1;
  logic       sa, sb, sy;
  logic       cin, cout, cinb, sameoutsign;
  
  // extract magnitudes and signs of inputs
  assign {sa, ma} = a; 
  assign {sb, mb} = b;
  
  // See if signs are different and add or subtract accordingly
  xor    gxcin(cin, sa, sb);
  xor_3  xmb2(cin, mb, mb2);
  add_3  addmr(ma, mb2, cin, mr, cout);
  
  // Invert the sign of the result if necessary
  not_3  nmrb1(mr, mrb1);
  incr   incmrb(mrb1, mrb);
  
  // Select the appropriate answer
  not    gncinb(cinb, cin);
  or     gs(sameoutsign, cinb, cout);
  mux2_3 mymux(sameoutsign, mrb, mr, my);
  mux2   symux(sameoutsign, sb, sa, sy);
  initial
    $display("%m size == %d", 9 + 2 + 5);
  
  // reassemble output
  assign y = {sy, my};
endmodule

module not_3(input  logic [2:0] a,
             output logic [2:0] y);

  not n0(y[0], a[0]);
  not n1(y[1], a[1]);
  not n2(y[2], a[2]);
  initial
    $display("%m size == %d", 2*3);
endmodule

module xor_3(input  logic       a,
             input  logic [2:0] b,
             output logic [2:0] y);

  xor x0(y[0], a, b[0]);
  xor x1(y[1], a, b[1]);
  xor x2(y[2], a, b[2]);
  initial
    $display("%m size == %d", 9 * 3);
endmodule

module mux2_3(input  logic       s,
              input  logic [2:0] d0, d1,
              output logic [2:0] y);
    
  mux2 m0(s, d0[0], d1[0], y[0]);
  mux2 m1(s, d0[1], d1[1], y[1]);
  mux2 m2(s, d0[2], d1[2], y[2]);
endmodule

module mux2(input  logic s,
            input  logic d0, d1,
            output logic y);
    
  logic sb, a0, a1;
  
  not gn1(sb, s);
  and ga0(a0, sb, d0);
  and ga1(a1, s, d1);
  or  go1(y, a0, a1);
  initial
    $display("%m size == %d", 2 + 2*5 + 5);
endmodule

module add_3(input  logic [2:0] a, b,
             input  logic       cin,
             output logic [2:0] s,
             output logic       cout);
             
  logic [1:0] c;

  fulladder fa0(a[0], b[0], cin, s[0], c[0]);
  fulladder fa1(a[1], b[1], c[0], s[1], c[1]);
  fulladder fa2(a[2], b[2], c[1], s[2], cout);
endmodule

module fulladder(input  logic a, b, cin,
                 output logic s, cout);
                 
  logic n1, n2, n3, x1;
  
  // sum
  xor gx1(x1, a, b);
  xor gx2(s, x1, cin);
  
  // carry
  and ga1(n1, a, b);
  and ga2(n2, a, cin);
  and ga3(n3, b, cin);
  or  go1(cout, n1, n2, n3);

  initial
    $display("%m size == %d", 9 * 2 + 3 * 5 + 6);
endmodule

module incr(input  logic [2:0] a,
            output logic [2:0] s);
              
  logic     c1;

  not       han0(s[0], a[0]);
  xor       hax1(s[1], a[1], a[0]);
  and       haa1(c1, a[1], a[0]);
  xor       hax2(s[2], a[2], c1);
  initial
    $display("%m size == %d", 2 + 9 * 2 + 5);
endmodule

module testbench(); 
  logic        clk;
  logic [3:0]  a, b, y, yexpected;
  logic [6:0]  hash;
  logic [31:0] vectornum, errors;
  logic [11:0]  testvectors[10000:0];

  // instantiate device under test 
  smadd dut(a, b, y);

  // generate clock 
  always 
    begin
      clk=1; #5; clk=0; #5; 
    end 

  // at start of test, load vectors and pulse reset
  initial 
    begin
      $readmemb("smadd.tv", testvectors); 
      vectornum = 0; errors = 0; hash = 0; 
    end 

  // apply test vectors on rising edge of clk 
  always @(posedge clk) 
    begin
      #1; {a, b, yexpected} = testvectors[vectornum]; 
    end 

  // check results on falling edge of clk 
  always @(negedge clk) begin
    if (y !== yexpected) begin // check result 
      $display("Error: inputs = %b %b", a, b);
      $display(" output = %b (%b expected)", y, yexpected);
      errors = errors + 1; 
    end
    vectornum = vectornum + 1;
    hash = hash ^ y;
    hash = {hash[5:0], hash[6] ^ hash[5]};
    if (testvectors[vectornum] === 12'bx) begin 
      $display("%d tests completed with %d errors", vectornum, errors); 
      $display("Hash: %h", hash);
      $stop; 
    end 
  end 
endmodule 
 
