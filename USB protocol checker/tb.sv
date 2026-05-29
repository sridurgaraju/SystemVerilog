//=============================================================================
// USB Packet Parser - Testbench (Step 1)
// Drives: IN token, DATA0 packet, ACK handshake (a complete bulk-IN transaction)
// Also injects a PID complement error to verify error detection
// Run in Questa: vlog usb_packet_parser.sv tb_usb_packet_parser.sv
//               vsim tb_usb_packet_parser -do "run -all"
//=============================================================================

`timescale 1ns/1ps

module tb_usb_packet_parser;
  import usb_pkg::*;

  //--- Clock and reset ---
  logic clk, rst_n;
  always #5 clk = ~clk; // 100 MHz clock

  //--- DUT signals ---
  logic       bit_in, bit_valid, pkt_start, pkt_end;
  logic [7:0] pid_byte;
  logic [3:0] pid;
  pkt_type_e  pkt_type;
  logic       pid_valid, pid_error;
  logic [6:0] token_addr;
  logic [3:0] token_endp;
  logic [10:0]sof_frame_num;
  logic       token_pkt_valid;
  logic [7:0] data_payload[0:63];
  logic [5:0] data_len;
  logic       data_pkt_valid;
  logic       hs_pkt_valid;
  logic       parse_error;

  //--- DUT instantiation ---
  usb_packet_parser dut (.*);

  //=========================================================================
  // Task: send one byte serially (LSB first), with pkt_start on first byte
  //=========================================================================
task automatic send_byte(input logic [7:0] data, input logic is_first);
  if (is_first) begin
    // Cycle 1: pulse pkt_start, hold bit_in=0 (don't drive bit 0 yet)
    @(posedge clk); #1;
    pkt_start = 1'b1;
    bit_valid = 1'b0;        // no valid bit this cycle
    @(posedge clk); #1;
    pkt_start = 1'b0;
    // Cycle 2 onwards: drive all 8 bits normally
    for (int i = 0; i < 8; i++) begin
      bit_in    = data[i];
      bit_valid = 1'b1;
      @(posedge clk); #1;
    end
  end else begin
    for (int i = 0; i < 8; i++) begin
      bit_in    = data[i];
      bit_valid = 1'b1;
      @(posedge clk); #1;
    end
  end
endtask

  //=========================================================================
  // Task: send EOP (end of packet)
  //=========================================================================
  task automatic send_eop();
    @(posedge clk); #1;
    bit_valid = 1'b0;
    pkt_end   = 1'b1;
    @(posedge clk); #1;
    pkt_end   = 1'b0;
    repeat(2) @(posedge clk);
  endtask

  //=========================================================================
  // Task: send a token packet (PID + ADDR + ENDP packed into 2 bytes)
  // token_data[10:0] = {endp[3:0], addr[6:0]}
  //=========================================================================
  task automatic send_token_pkt(
    input logic [7:0] pid_val,    // full 8-bit PID (upper nibble = ~lower)
    input logic [6:0] addr,
    input logic [3:0] endp
  );
    logic [15:0] token_field;
    logic [7:0]  byte1, byte2;

    // Pack: byte1 = addr[6:0] | endp[0], byte2 = endp[3:1] | CRC5(dummy=0)
    token_field = {5'b00000, endp, addr}; // CRC5 = 0 for now (Step 3 adds real CRC)
    byte1 = token_field[7:0];
    byte2 = token_field[15:8];

    $display("\n[TB] Sending TOKEN pid=0x%02X addr=%0d endp=%0d", pid_val, addr, endp);
    send_byte(pid_val, 1);   // PID byte, first byte of packet
    send_byte(byte1,  0);
    send_byte(byte2,  0);
    send_eop();
  endtask

  //=========================================================================
  // Task: send a data packet (PID + payload bytes + 2 dummy CRC bytes)
  //=========================================================================
  task automatic send_data_pkt(
    input logic [7:0] pid_val,
    input logic [7:0] payload[],
    input int         len
  );
    $display("[TB] Sending DATA pid=0x%02X len=%0d", pid_val, len);
    send_byte(pid_val, 1);
    for (int i = 0; i < len; i++)
      send_byte(payload[i], 0);
    // 2 dummy CRC16 bytes (real CRC added in Step 3)
    send_byte(8'hAA, 0);
    send_byte(8'h55, 0);
    send_eop();
  endtask

  //=========================================================================
  // Task: send handshake packet (PID only, no payload)
  //=========================================================================
  task automatic send_handshake_pkt(input logic [7:0] pid_val);
    $display("[TB] Sending HANDSHAKE pid=0x%02X", pid_val);
    send_byte(pid_val, 1);
    send_eop();
  endtask

  //=========================================================================
  // Main test
  //=========================================================================
  initial begin
    // Init
    clk       = 0;
    rst_n     = 0;
    bit_in    = 0;
    bit_valid = 0;
    pkt_start = 0;
    pkt_end   = 0;

    repeat(4) @(posedge clk);
    rst_n = 1;
    repeat(2) @(posedge clk);

    $display("=================================================");
    $display(" USB Packet Parser Testbench - Step 1");
    $display("=================================================");

    //----------------------------------------------------------------------
    // Test 1: Complete Bulk-IN transaction
    //   Host sends IN token → Device sends DATA0 → Host sends ACK
    //----------------------------------------------------------------------
    $display("\n--- Test 1: Bulk-IN transaction (IN + DATA0 + ACK) ---");

    // IN token: addr=5, endp=1
    // PID_IN = 4'b1001, full byte = {~1001, 1001} = {0110, 1001} = 8'h69
    send_token_pkt(8'h69, 7'd5, 4'd1);
    repeat(3) @(posedge clk);

    // DATA0 payload: 4 bytes
    // PID_DATA0 = 4'b0011, full byte = {~0011, 0011} = {1100, 0011} = 8'hC3
    begin
      logic [7:0] payload[4];
      payload[0] = 8'hDE;
      payload[1] = 8'hAD;
      payload[2] = 8'hBE;
      payload[3] = 8'hEF;
      send_data_pkt(8'hC3, payload, 4);
    end
    repeat(3) @(posedge clk);

    // ACK handshake
    // PID_ACK = 4'b0010, full byte = {~0010, 0010} = {1101, 0010} = 8'hD2
    send_handshake_pkt(8'hD2);
    repeat(3) @(posedge clk);

    //----------------------------------------------------------------------
    // Test 2: SETUP token (control transfer start)
    //----------------------------------------------------------------------
    $display("\n--- Test 2: SETUP token addr=0 endp=0 ---");
    // PID_SETUP = 4'b1101, full byte = {~1101, 1101} = {0010, 1101} = 8'h2D
    send_token_pkt(8'h2D, 7'd0, 4'd0);
    repeat(3) @(posedge clk);

    //----------------------------------------------------------------------
    // Test 3: NAK handshake
    //----------------------------------------------------------------------
    $display("\n--- Test 3: NAK handshake ---");
    // PID_NAK = 4'b1010, full byte = {~1010, 1010} = {0101, 1010} = 8'h5A
    send_handshake_pkt(8'h5A);
    repeat(3) @(posedge clk);

    //----------------------------------------------------------------------
    // Test 4: Bad PID - complement check should fail
    //----------------------------------------------------------------------
    $display("\n--- Test 4: Bad PID (complement check must FAIL) ---");
    // Deliberately corrupt: upper nibble does NOT match ~lower nibble
    send_byte(8'hFF, 1); // 8'hFF: upper=1111, lower=1111, ~lower=0000 → MISMATCH
    send_eop();
    repeat(3) @(posedge clk);

    //----------------------------------------------------------------------
    // Test 5: SOF packet
    //----------------------------------------------------------------------
    $display("\n--- Test 5: SOF packet frame=0x001 ---");
    // PID_SOF = 4'b0101, full byte = {~0101, 0101} = {1010, 0101} = 8'hA5
    send_token_pkt(8'hA5, 7'd0, 4'd0); // frame num in addr/endp fields for SOF
    repeat(3) @(posedge clk);

    //----------------------------------------------------------------------
    // Done
    //----------------------------------------------------------------------
    $display("\n=================================================");
    $display(" All tests complete");
    $display("=================================================");
    $finish;
  end

  //=========================================================================
  // Monitors: print when valid outputs appear
  //=========================================================================
  always @(posedge clk) begin
    if (token_pkt_valid)
      $display("[MONITOR] Token valid - addr=%0d endp=%0d pid=%s",
        token_addr, token_endp, dut.pid_name(pid));

    if (data_pkt_valid)
      $display("[MONITOR] Data valid  - pid=%s payload_len=%0d",
        dut.pid_name(pid), data_len);

    if (hs_pkt_valid)
      $display("[MONITOR] Handshake   - %s", dut.pid_name(pid));

    if (pid_error)
      $display("[MONITOR] *** PID ERROR detected ***");

    if (parse_error && !pid_error)
      $display("[MONITOR] *** PARSE ERROR ***");
  end

endmodule
