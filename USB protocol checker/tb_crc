//=============================================================================
// USB Packet Parser Testbench - Step 2
// Adds: real CRC5 in token packets, real CRC16 in data packets,
//       crc_ok/crc_error monitor lines, Test 6 (bad CRC injection)
//=============================================================================

`timescale 1ns/1ps

module tb_usb_packet_parser;
  import usb_pkg::*;

  logic clk, rst_n;
  always #5 clk = ~clk;

  //--- DUT signals ---
  logic        bit_in, bit_valid, pkt_start, pkt_end;
  logic [7:0]  pid_byte;
  logic [3:0]  pid;
  pkt_type_e   pkt_type;
  logic        pid_valid, pid_error;
  logic [6:0]  token_addr;
  logic [3:0]  token_endp;
  logic [10:0] sof_frame_num;
  logic        token_pkt_valid;
  logic [7:0]  data_payload[0:63];
  logic [5:0]  data_len;
  logic        data_pkt_valid;
  logic        hs_pkt_valid;
  logic        crc_ok, crc_error;   // NEW Step 2
  logic        parse_error;

  usb_packet_parser dut (.*);

  //=========================================================================
  // CRC5 software model
  // Computes CRC5 over 11-bit value {endp[3:0], addr[6:0]}, returns 5 bits.
  // Used by send_token_pkt to insert real CRC into the packet.
  //=========================================================================
  function automatic logic [4:0] compute_crc5(
    input logic [6:0] addr,
    input logic [3:0] endp
  );
    logic [4:0] crc = 5'b11111;
    logic [10:0] data = {endp, addr};  // {endp[3:0], addr[6:0]}, LSB first
    logic fb;
    for (int i = 0; i < 11; i++) begin
      fb    = data[i] ^ crc[4];
      crc[4] = crc[3];
      crc[3] = crc[2];
      crc[2] = crc[1] ^ fb;
      crc[1] = crc[0];
      crc[0] = fb;
    end
    return ~crc;  // USB: invert before appending
  endfunction

  //=========================================================================
  // CRC16 software model
  // Computes CRC16 over a byte array, returns 16 bits.
  //=========================================================================
  function automatic logic [15:0] compute_crc16(
  input logic [7:0] data[],
  input int         len
  );
  logic [15:0] crc = 16'hFFFF;
  logic fb;
  for (int b = 0; b < len; b++) begin
    for (int i = 0; i < 8; i++) begin
      fb  = data[b][i] ^ crc[0];   // LSB-first
      crc >>= 1;
      if (fb) crc ^= 16'hA001;
    end
  end
  return ~crc;   // invert before appending
endfunction

  //=========================================================================
  // send_byte: drive one byte serially, LSB first
  // pkt_start pulses on its own cycle before bit 0 (Step 1 fix retained)
  //=========================================================================
  task automatic send_byte(input logic [7:0] data, input logic is_first);
    if (is_first) begin
      @(posedge clk); #1;
      pkt_start = 1'b1;
      bit_valid = 1'b0;
      @(posedge clk); #1;
      pkt_start = 1'b0;
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
  // send_eop: end-of-packet signal
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
  // send_token_pkt: sends PID + 2 token bytes with REAL CRC5
  //=========================================================================
  task automatic send_token_pkt(
    input logic [7:0] pid_val,
    input logic [6:0] addr,
    input logic [3:0] endp
  );
    logic [4:0]  crc5;
    logic [15:0] token_field;
    logic [7:0]  byte1, byte2;

    crc5         = compute_crc5(addr, endp);
    // Pack: [15:11]=CRC5, [10:7]=endp, [6:0]=addr
    token_field  = {crc5, endp, addr};
    byte1        = token_field[7:0];
    byte2        = token_field[15:8];

    $display("\n[TB] Sending TOKEN pid=0x%02X addr=%0d endp=%0d CRC5=0x%02X",
             pid_val, addr, endp, crc5);
    send_byte(pid_val, 1);
    send_byte(byte1,   0);
    send_byte(byte2,   0);
    send_eop();
  endtask

  //=========================================================================
  // send_data_pkt: sends PID + payload + REAL CRC16
  //=========================================================================
  task automatic send_data_pkt(
    input logic [7:0] pid_val,
    input logic [7:0] payload[],
    input int         len
  );
    logic [15:0] crc16;
    logic [7:0]  crc_lo, crc_hi;

    crc16  = compute_crc16(payload, len);
    crc_lo = crc16[7:0];    // sent LSB byte first
    crc_hi = crc16[15:8];

    $display("[TB] Sending DATA pid=0x%02X len=%0d CRC16=0x%04X",
             pid_val, len, crc16);
    send_byte(pid_val, 1);
    for (int i = 0; i < len; i++)
      send_byte(payload[i], 0);
    send_byte(crc_lo, 0);
    send_byte(crc_hi, 0);
    send_eop();
  endtask

  //=========================================================================
  // send_data_pkt_bad_crc: injects wrong CRC to test crc_error detection
  //=========================================================================
  task automatic send_data_pkt_bad_crc(
    input logic [7:0] pid_val,
    input logic [7:0] payload[],
    input int         len
  );
    logic [15:0] crc16;
    logic [15:0] bad_crc;

    crc16   = compute_crc16(payload, len);
    bad_crc = crc16 ^ 16'h0001;   // deliberate inversion - will not match residue

    $display("[TB] Sending DATA (BAD CRC) pid=0x%02X len=%0d good=0x%04X bad=0x%04X",
             pid_val, len, crc16, bad_crc);
    send_byte(pid_val, 1);
    for (int i = 0; i < len; i++)
      send_byte(payload[i], 0);
    send_byte(bad_crc[7:0],  0);
    send_byte(bad_crc[15:8], 0);
    send_eop();
  endtask

  //=========================================================================
  // send_handshake_pkt (unchanged)
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
    clk = 0; rst_n = 0;
    bit_in = 0; bit_valid = 0; pkt_start = 0; pkt_end = 0;

    repeat(4) @(posedge clk);
    rst_n = 1;
    repeat(2) @(posedge clk);

    $display("=================================================");
    $display(" USB Packet Parser Testbench - Step 2");
    $display("=================================================");

    //----------------------------------------------------------------------
    // Test 1: Bulk-IN - real CRC5 on token, real CRC16 on data
    //----------------------------------------------------------------------
    $display("\n--- Test 1: Bulk-IN (real CRC5 + CRC16) ---");
    send_token_pkt(8'h69, 7'd5, 4'd1);   // IN, addr=5, endp=1
    repeat(3) @(posedge clk);

    begin
      logic [7:0] payload[4];
      payload[0] = 8'hDE; payload[1] = 8'hAD;
      payload[2] = 8'hBE; payload[3] = 8'hEF;
      send_data_pkt(8'hC3, payload, 4);  // DATA0
    end
    repeat(3) @(posedge clk);

    send_handshake_pkt(8'hD2);            // ACK
    repeat(3) @(posedge clk);

    //----------------------------------------------------------------------
    // Test 2: SETUP token
    //----------------------------------------------------------------------
    $display("\n--- Test 2: SETUP token addr=0 endp=0 ---");
    send_token_pkt(8'h2D, 7'd0, 4'd0);
    repeat(3) @(posedge clk);

    //----------------------------------------------------------------------
    // Test 3: NAK
    //----------------------------------------------------------------------
    $display("\n--- Test 3: NAK handshake ---");
    send_handshake_pkt(8'h5A);
    repeat(3) @(posedge clk);

    //----------------------------------------------------------------------
    // Test 4: Bad PID
    //----------------------------------------------------------------------
    $display("\n--- Test 4: Bad PID (complement check must FAIL) ---");
    send_byte(8'hFF, 1);
    send_eop();
    repeat(3) @(posedge clk);

    //----------------------------------------------------------------------
    // Test 5: SOF
    //----------------------------------------------------------------------
    $display("\n--- Test 5: SOF packet ---");
    send_token_pkt(8'hA5, 7'd0, 4'd0);
    repeat(3) @(posedge clk);

    //----------------------------------------------------------------------
    // Test 6: Data packet with deliberately bad CRC16 (NEW)
    //----------------------------------------------------------------------
    $display("\n--- Test 6: DATA0 with bad CRC16 (crc_error must fire) ---");
    begin
      logic [7:0] payload[3];
      payload[0] = 8'h11; payload[1] = 8'h22; payload[2] = 8'h33;
      send_data_pkt_bad_crc(8'hC3, payload, 3);
    end
    repeat(3) @(posedge clk);

    $display("\n=================================================");
    $display(" All tests complete");
    $display("=================================================");
    $finish;
  end

  //=========================================================================
  // Monitor
  //=========================================================================
  always @(posedge clk) begin
    if (token_pkt_valid)
      $display("[MONITOR] Token valid  - addr=%0d endp=%0d pid=%s",
               token_addr, token_endp, dut.pid_name(pid));
    if (data_pkt_valid)
      $display("[MONITOR] Data valid   - pid=%s payload_len=%0d",
               dut.pid_name(pid), data_len);
    if (hs_pkt_valid)
      $display("[MONITOR] Handshake    - %s", dut.pid_name(pid));
    if (crc_ok)
      $display("[MONITOR] CRC OK");
    if (crc_error)
      $display("[MONITOR] *** CRC ERROR ***");
    if (pid_error)
      $display("[MONITOR] *** PID ERROR detected ***");
    if (parse_error && !pid_error)
      $display("[MONITOR] *** PARSE ERROR ***");
  end

endmodule
