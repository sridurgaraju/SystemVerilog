module testbench;

  parameter N = 2;

  // Instantiate the interface with parameterized addr width
  mesi_if #(2) mesi_intf();
  

  // DUT instance (connect through interface)
  top_system #(N) dut (
    .clk         (mesi_intf.clk),
    .rst         (mesi_intf.rst),
    .read_req    (mesi_intf.read_req),
    .write_req   (mesi_intf.write_req),
    .addr        (mesi_intf.addr),
    .mem_read    (mesi_intf.mem_read),
    .mem_write   (mesi_intf.mem_write),
    .state       (mesi_intf.state)
  );

  // Clock generation
  initial begin
    mesi_intf.clk = 0;
  end
  always #5 mesi_intf.clk = ~mesi_intf.clk;

    // Stimulus
initial begin
    mesi_intf.rst = 1;
    mesi_intf.read_req = 0;
    mesi_intf.write_req = 0;
    mesi_intf.addr[0] = 32'h0;
    mesi_intf.addr[1] = 32'h0;
    #15 mesi_intf.rst = 0;

    // --- Scenario 1: Cache 0 reads (should go to Exclusive or Shared)
    mesi_intf.addr[0] = 32'h1000;
    mesi_intf.read_req[0] = 1;
    #10 mesi_intf.read_req[0] = 0;
    #10;

    // --- Scenario 2: Cache 1 reads same address (should downgrade 0 to Shared)
    mesi_intf.addr[1] = 32'h1000;
    mesi_intf.read_req[1] = 1;
    #10 mesi_intf.read_req[1] = 0;
    #10;

    // --- Scenario 3: Cache 1 writes to 0x1000 (should invalidate Cache 0)
    mesi_intf.write_req[1] = 1;
    #10 mesi_intf.write_req[1] = 0;
    #10;

    // --- Scenario 4: Cache 0 writes to new address (Exclusive to Modified)
    mesi_intf.addr[0] = 32'h2000;
    mesi_intf.write_req[0] = 1;
    #10 mesi_intf.write_req[0] = 0;
    #10;

    // --- Scenario 5: Cache 1 reads 0x2000 (should force flush from Cache 0)
    mesi_intf.addr[1] = 32'h2000;
    mesi_intf.read_req[1] = 1;
    #10 mesi_intf.read_req[1] = 0;
    #10;

    // --- Scenario 6: Cache 1 writes to new address
    mesi_intf.addr[1] = 32'h3000;
    mesi_intf.write_req[1] = 1;
    #10 mesi_intf.write_req[1] = 0;
    #10;

    // --- Scenario 7: Cache 0 reads same address (should flush & go Shared)
    mesi_intf.addr[0] = 32'h3000;
    mesi_intf.read_req[0] = 1;
    #10 mesi_intf.read_req[0] = 0;
    #10;

    // --- Scenario 8: Cache 0 reads another fresh address
    mesi_intf.addr[0] = 32'h4000;
    mesi_intf.read_req[0] = 1;
    #10 mesi_intf.read_req[0] = 0;
    #10;

    // --- Scenario 9: Cache 1 writes to same address
    mesi_intf.addr[1] = 32'h4000;
    mesi_intf.write_req[1] = 1;
    #10 mesi_intf.write_req[1] = 0;
    #10;

    // --- Scenario 10: Back-to-back writes from both caches to same address
    mesi_intf.addr[0] = 32'h5000;
    mesi_intf.addr[1] = 32'h5000;
    mesi_intf.write_req = 2'b11; // both attempt write
    #10 mesi_intf.write_req = 2'b00;
    #10;
  
    
    // --- Wrap up
    #50 $finish;
end

     initial begin
$dumpfile("dump.vcd");
$dumpvars;
end
  
function string state_to_str(input logic [1:0] state);
    case (state)
        2'b00: return "I";
        2'b01: return "S";
        2'b10: return "E";
        2'b11: return "M";
        default: return "U"; // undefined
    endcase
endfunction

initial begin
    $monitor("Time: %0t | Cache0=%s | Cache1=%s | MemRead0=%b | MemWrite1=%b",
        $time, state_to_str(mesi_intf.state[0]), state_to_str(mesi_intf.state[1]),
        mesi_intf.mem_read[0], mesi_intf.mem_write[1]);
end
  
  logic [1:0] prev_state[N];    // MESI states: I=00, S=01, E=10, M=11
  logic [31:0] prev_addr[N];    // Previous address accessed by each cache


  covergroup mesi_transitions(int cid) @(posedge mesi_intf.clk);
    option.per_instance = 1;

    cp_state : coverpoint mesi_intf.state[cid] {
      bins I_to_E = (2'b00 => 2'b10); // Invalid to Exclusive
      bins I_to_S = (2'b00 => 2'b01); // Invalid to Shared
      bins I_to_M = (2'b00 => 2'b11);
      bins S_to_M = (2'b01 => 2'b11); // Shared to Modified
      bins E_to_M = (2'b10 => 2'b11); // Exclusive to Modified
      bins M_to_I = (2'b11 => 2'b00); // Modified to Invalid
      bins M_to_S = (2'b11 => 2'b01);
      bins M_to_E = (2'b11 => 2'b10);
      bins E_to_S = (2'b10 => 2'b01); // Exclusive to Shared
      bins E_to_I = (2'b10 => 2'b00);
      bins S_to_I = (2'b01 => 2'b00); // Shared to Invalid
      bins S_to_E = (2'b01 => 2'b10);
    }
  endgroup

  // Create covergroup instances
  mesi_transitions mesi_cov_inst[N];

  initial begin
    for (int i = 0; i < N; i++) begin
      mesi_cov_inst[i] = new(i);
    end
  end
  
  always_ff @(posedge mesi_intf.clk) begin
  if (!mesi_intf.rst) begin
    for (int i = 0; i < N; i++) begin
      // Only sample if state changed and address is same
      if (mesi_intf.state[i] !== prev_state[i] &&
          mesi_intf.addr[i] == prev_addr[i]) begin
        mesi_cov_inst[i].sample();
      end

      // Update tracking for next cycle
      prev_state[i] <= mesi_intf.state[i];
      prev_addr[i]  <= mesi_intf.addr[i];
    end
  end
  end

endmodule
