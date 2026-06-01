//=============================================================================
// USB 2.0 Packet Parser - Step 2
// Adds: CRC5 checker (token packets), CRC16 checker (data packets)
// New outputs: crc_ok (pulse on pass), crc_error (pulse on fail)
//=============================================================================

`timescale 1ns/1ps

//-----------------------------------------------------------------------------
// Package: PID constants and packet type enum
//-----------------------------------------------------------------------------
package usb_pkg;

  parameter logic [3:0] PID_OUT   = 4'b0001;
  parameter logic [3:0] PID_IN    = 4'b1001;
  parameter logic [3:0] PID_SOF   = 4'b0101;
  parameter logic [3:0] PID_SETUP = 4'b1101;
  parameter logic [3:0] PID_DATA0 = 4'b0011;
  parameter logic [3:0] PID_DATA1 = 4'b1011;
  parameter logic [3:0] PID_ACK   = 4'b0010;
  parameter logic [3:0] PID_NAK   = 4'b1010;
  parameter logic [3:0] PID_STALL = 4'b1110;

  typedef enum logic [1:0] {
    PKT_TOKEN     = 2'b00,
    PKT_DATA      = 2'b01,
    PKT_HANDSHAKE = 2'b10,
    PKT_UNKNOWN   = 2'b11
  } pkt_type_e;

endpackage

//=============================================================================
// CRC5 checker
// Computes CRC5 over 11-bit token data {endp[3:0], addr[6:0]}
// Polynomial: x^5 + x^2 + 1
// Init: 5'b11111   Residue (pass): 5'b01100
//
// Interface:
//   start    - pulse to reset and begin a new computation
//   data_in  - 1 bit per clock, LSB first
//   check    - pulse to compare current register against residue
//   crc_ok   - pulses 1 cycle after check if register == residue
//   crc_error- pulses 1 cycle after check if register != residue
//=============================================================================
module crc5_checker (
  input  logic        clk,
  input  logic        rst_n,
  input  logic        check,        // pulse to trigger check
  input  logic [15:0] token_buf,    // full 16-bit assembled token field
  output logic        crc_ok,
  output logic        crc_error
);

  function automatic logic [4:0] compute_crc5(input logic [10:0] data);
    logic [4:0] crc = 5'b11111;
    logic fb;
    for (int i = 0; i < 11; i++) begin
      fb    = data[i] ^ crc[4];
      crc   = {crc[3], crc[2], crc[1] ^ fb, crc[0], fb};
    end
    return ~crc;
  endfunction

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      crc_ok    <= 1'b0;
      crc_error <= 1'b0;
    end else begin
      crc_ok    <= 1'b0;
      crc_error <= 1'b0;
      if (check) begin
        if (compute_crc5(token_buf[10:0]) == token_buf[15:11]) begin
          crc_ok    <= 1'b1;
          $display("[CRC5] PASS");
        end else begin
          crc_error <= 1'b1;
          $display("[CRC5] FAIL - computed=0x%02X received=0x%02X",
                   compute_crc5(token_buf[10:0]), token_buf[15:11]);
        end
      end
    end
  end
endmodule

//=============================================================================
// CRC16 checker
// Computes CRC16 over data payload bytes (fed byte-by-byte, LSB first)
// Polynomial: x^16 + x^15 + x^2 + 1
// Init: 16'hFFFF   Residue (pass): 16'hB001
//
// Same interface as crc5_checker but data_in feeds 8 bits per byte:
//   byte_in  - full byte (fed bit-by-bit internally over 8 valid pulses)
//   byte_valid - pulse for each incoming byte
//=============================================================================
module crc16_checker (
  input  logic        clk,
  input  logic        rst_n,
  input  logic        start,
  input  logic [7:0]  byte_in,
  input  logic        byte_valid,
  input  logic        check,
  output logic        crc_ok,
  output logic        crc_error
);
  localparam logic [15:0] CRC16_RESIDUE = 16'hB001;

  logic [15:0] crc_reg;

  // Process one byte: loop over 8 bits LSB first
  // Using a function to keep the always_ff clean
  function automatic logic [15:0] crc16_byte(
  input logic [15:0] crc,
  input logic [7:0]  data
);
  logic fb;
  logic [15:0] c = crc;
  for (int i = 0; i < 8; i++) begin
    fb  = data[i] ^ c[0];    // feedback from LSB (USB is LSB-first)
    c >>= 1;                  // shift right
    if (fb) c ^= 16'hA001;   // reflected polynomial of x^16+x^15+x^2+1
  end
  return c;
endfunction

  // NOTE on CRC16 tap positions:
  // Poly x^16 + x^15 + x^2 + 1:
  //   fb = data_bit XOR crc[15]
  //   crc shifts right one position
  //   XOR fb into crc[14] (x^15 tap) and crc[1] (x^2 tap)
  // The function above implements this correctly bit by bit.

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      crc_reg   <= 16'hFFFF;
      crc_ok    <= 1'b0;
      crc_error <= 1'b0;
    end else begin
      crc_ok    <= 1'b0;
      crc_error <= 1'b0;

      if (start) begin
        crc_reg <= 16'hFFFF;

      end else if (byte_valid) begin
        crc_reg <= crc16_byte(crc_reg, byte_in);

      end else if (check) begin
        if (crc_reg == CRC16_RESIDUE) begin
          crc_ok    <= 1'b1;
          $display("[CRC16] PASS (residue=0x%04X)", crc_reg);
        end else begin
          crc_error <= 1'b1;
          $display("[CRC16] FAIL - residue=0x%04X, expected=0x%04X",
                   crc_reg, CRC16_RESIDUE);
        end
      end
    end
  end
endmodule

//=============================================================================
// Top-level packet parser (Step 2)
// Added ports: crc_ok, crc_error
// Internal: crc5_checker and crc16_checker instances
//=============================================================================
module usb_packet_parser
  import usb_pkg::*;
(
  input  logic        clk,
  input  logic        rst_n,

  // Bit stream
  input  logic        bit_in,
  input  logic        bit_valid,

  // Framing
  input  logic        pkt_start,
  input  logic        pkt_end,

  // PID outputs
  output logic [7:0]  pid_byte,
  output logic [3:0]  pid,
  output pkt_type_e   pkt_type,
  output logic        pid_valid,
  output logic        pid_error,

  // Token outputs
  output logic [6:0]  token_addr,
  output logic [3:0]  token_endp,
  output logic [10:0] sof_frame_num,
  output logic        token_pkt_valid,

  // Data outputs
  output logic [7:0]  data_payload [0:63],
  output logic [5:0]  data_len,
  output logic        data_pkt_valid,

  // Handshake output
  output logic        hs_pkt_valid,

  // CRC outputs (NEW in Step 2)
  output logic        crc_ok,       // pulses when CRC check passes
  output logic        crc_error,    // pulses when CRC check fails

  // Error
  output logic        parse_error
);

  //-------------------------------------------------------------------------
  // Internal signals (same as Step 1)
  //-------------------------------------------------------------------------
  logic [7:0]  shift_reg;
  logic [2:0]  bit_count;
  logic [5:0]  byte_count;
  logic        byte_ready;
  logic [7:0]  current_byte;

  typedef enum logic [2:0] {
    S_IDLE, S_PID, S_TOKEN, S_DATA, S_HANDSHAKE, S_DONE, S_ERROR
  } state_e;
  state_e state, next_state;

  logic [15:0] token_buf;
  logic [3:0]  token_byte_cnt;
  logic [5:0]  data_byte_idx;

  //-------------------------------------------------------------------------
  // CRC5 control signals
  //-------------------------------------------------------------------------
  logic crc5_check;
  logic crc5_ok, crc5_error;

  //-------------------------------------------------------------------------
  // CRC16 control signals
  //-------------------------------------------------------------------------
  logic        crc16_start, crc16_byte_valid, crc16_check;
  logic [7:0]  crc16_byte_in;
  logic        crc16_ok, crc16_error;

  //-------------------------------------------------------------------------
  // Combine CRC results onto output ports
  //-------------------------------------------------------------------------
  assign crc_ok    = crc5_ok    | crc16_ok;
  assign crc_error = crc5_error | crc16_error;

  //-------------------------------------------------------------------------
  // CRC module instantiations
  //-------------------------------------------------------------------------
  crc5_checker u_crc5 (
  .clk      (clk),
  .rst_n    (rst_n),
  .check    (crc5_check),
  .token_buf(token_buf),
  .crc_ok   (crc5_ok),
  .crc_error(crc5_error)
);

  crc16_checker u_crc16 (
    .clk        (clk),
    .rst_n      (rst_n),
    .start      (crc16_start),
    .byte_in    (crc16_byte_in),
    .byte_valid (crc16_byte_valid),
    .check      (crc16_check),
    .crc_ok     (crc16_ok),
    .crc_error  (crc16_error)
  );

  //-------------------------------------------------------------------------
  // Shift register: bits → bytes (unchanged from Step 1)
  //-------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      shift_reg  <= 8'h00;
      bit_count  <= 3'd0;
      byte_ready <= 1'b0;
    end else begin
      byte_ready <= 1'b0;
      if (pkt_start) begin
        shift_reg <= 8'h00;
        bit_count <= 3'd0;
      end else if (bit_valid && (state != S_IDLE) && (state != S_DONE)) begin
        shift_reg <= {bit_in, shift_reg[7:1]};
        if (bit_count == 3'd7) begin
          bit_count  <= 3'd0;
          byte_ready <= 1'b1;
        end else
          bit_count <= bit_count + 1'b1;
      end
    end
  end
  assign current_byte = shift_reg;

  //-------------------------------------------------------------------------
  // FSM sequential
  //-------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) state <= S_IDLE;
    else        state <= next_state;
  end

  //-------------------------------------------------------------------------
  // FSM combinational (reads current_byte directly - Step 1 fix retained)
  //-------------------------------------------------------------------------
  always_comb begin
    next_state = state;
    case (state)
      S_IDLE:  if (pkt_start) next_state = S_PID;

      S_PID: begin
        if (byte_ready) begin
          if (current_byte[7:4] != ~current_byte[3:0])
            next_state = S_ERROR;
          else case (current_byte[3:0])
            PID_OUT, PID_IN,
            PID_SOF, PID_SETUP: next_state = S_TOKEN;
            PID_DATA0, PID_DATA1: next_state = S_DATA;
            PID_ACK, PID_NAK,
            PID_STALL:          next_state = S_HANDSHAKE;
            default:            next_state = S_ERROR;
          endcase
        end
      end

      S_TOKEN:     if (pkt_end) next_state = S_DONE;
      S_DATA:      if (pkt_end) next_state = S_DONE;
      S_HANDSHAKE:              next_state = S_DONE;
      S_DONE:                   next_state = S_IDLE;
      S_ERROR:     if (pkt_end) next_state = S_IDLE;
      default:                  next_state = S_IDLE;
    endcase
  end

  //-------------------------------------------------------------------------
  // PID decoder (unchanged from Step 1)
  //-------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      pid_byte  <= 8'h00; pid <= 4'h0;
      pkt_type  <= PKT_UNKNOWN;
      pid_valid <= 1'b0;  pid_error <= 1'b0;
    end else begin
      pid_valid <= 1'b0;
      pid_error <= 1'b0;
      if (state == S_PID && byte_ready) begin
        pid_byte <= current_byte;
        pid      <= current_byte[3:0];
        if (current_byte[7:4] != ~current_byte[3:0]) begin
          pid_error <= 1'b1;
          $display("[USB PARSER] ERROR: PID complement check FAILED. Byte=0x%02X",
                   current_byte);
        end else begin
          pid_valid <= 1'b1;
          case (current_byte[3:0])
            PID_OUT, PID_IN,
            PID_SOF, PID_SETUP:   pkt_type <= PKT_TOKEN;
            PID_DATA0, PID_DATA1: pkt_type <= PKT_DATA;
            PID_ACK, PID_NAK,
            PID_STALL:            pkt_type <= PKT_HANDSHAKE;
            default:              pkt_type <= PKT_UNKNOWN;
          endcase
          $display("[USB PARSER] PID decoded: 0x%X (%s)",
                   current_byte[3:0], pid_name(current_byte[3:0]));
        end
      end
    end
  end

  //-------------------------------------------------------------------------
  // Token extractor + CRC5 control  (Step 2 revised)
  //
  // Collects 2 payload bytes after the PID into token_buf[15:0]:
  //   token_buf[10:0]  = {endp[3:0], addr[6:0]}
  //   token_buf[15:11] = CRC5 transmitted by host
  //
  // On pkt_end: extracts fields, pulses token_pkt_valid, pulses crc5_check.
  // crc5_checker reads token_buf directly and compares computed vs received.
  //
  // Removed vs old version:
  //   - crc5_start, crc5_data_in, crc5_valid
  //   - crc5_feed_active, crc5_bit_idx, token_buf_latch (sequencer gone)
  //-------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      token_buf       <= 16'h0;
      token_byte_cnt  <= 4'd0;
      token_addr      <= 7'h00;
      token_endp      <= 4'h0;
      sof_frame_num   <= 11'h000;
      token_pkt_valid <= 1'b0;
      crc5_check      <= 1'b0;
    end else begin
      token_pkt_valid <= 1'b0;
      crc5_check      <= 1'b0;

      // Reset byte counter when a clean token PID arrives
      if (state == S_PID && byte_ready && !pid_error)
        token_byte_cnt <= 4'd0;

      if (state == S_TOKEN) begin
        // Shift each incoming byte into the upper half of token_buf
        if (byte_ready) begin
          token_buf      <= {current_byte, token_buf[15:8]};
          token_byte_cnt <= token_byte_cnt + 1'b1;
        end

        if (pkt_end) begin
          // Extract fields from assembled token_buf
          if (pid == PID_SOF) begin
            sof_frame_num <= token_buf[10:0];
            $display("[USB PARSER] SOF frame=0x%03X", token_buf[10:0]);
          end else begin
            token_addr <= token_buf[6:0];
            token_endp <= token_buf[10:7];
            $display("[USB PARSER] TOKEN addr=0x%02X endp=0x%X pid=%s",
                     token_buf[6:0], token_buf[10:7], pid_name(pid));
          end
          token_pkt_valid <= 1'b1;
          crc5_check      <= 1'b1;  // checker reads token_buf directly this cycle
        end
      end
    end
  end
  //-------------------------------------------------------------------------
  // Data collector + CRC16 control
  //
  // All bytes (payload + CRC16) are fed to the CRC16 engine as they arrive.
  // On pkt_end, crc16_check fires.
  // data_len = total_bytes - 2 (strip the two CRC bytes from the count).
  //-------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      data_len        <= 6'd0;
      data_byte_idx   <= 6'd0;
      data_pkt_valid  <= 1'b0;
      crc16_start     <= 1'b0;
      crc16_byte_valid<= 1'b0;
      crc16_byte_in   <= 8'h00;
      crc16_check     <= 1'b0;
      for (int i = 0; i < 64; i++) data_payload[i] <= 8'h00;
    end else begin
      data_pkt_valid   <= 1'b0;
      crc16_start      <= 1'b0;
      crc16_byte_valid <= 1'b0;
      crc16_check      <= 1'b0;

      if (state == S_PID && byte_ready && !pid_error) begin
        data_byte_idx <= 6'd0;
        crc16_start   <= 1'b1;   // init CRC16 on new data packet
      end

      if (state == S_DATA) begin
        if (byte_ready && data_byte_idx < 6'd63) begin
          data_payload[data_byte_idx] <= current_byte;
          data_byte_idx               <= data_byte_idx + 1'b1;
          // Feed every byte (payload + CRC) to checker
          crc16_byte_in               <= current_byte;
          crc16_byte_valid            <= 1'b1;
        end
        if (pkt_end) begin
          data_len       <= (data_byte_idx >= 2) ? data_byte_idx - 6'd2 : 6'd0;
          data_pkt_valid <= 1'b1;
          crc16_check    <= 1'b1;
          $display("[USB PARSER] DATA%0d payload_bytes=%0d (excl CRC16)",
                   (pid == PID_DATA1) ? 1 : 0,
                   (data_byte_idx >= 2) ? data_byte_idx - 6'd2 : 0);
        end
      end
    end
  end

  //-------------------------------------------------------------------------
  // Handshake (unchanged)
  //-------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) hs_pkt_valid <= 1'b0;
    else begin
      hs_pkt_valid <= 1'b0;
      if (state == S_HANDSHAKE) begin
        hs_pkt_valid <= 1'b1;
        $display("[USB PARSER] HANDSHAKE: %s", pid_name(pid));
      end
    end
  end

  //-------------------------------------------------------------------------
  // Parse error
  //-------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) parse_error <= 1'b0;
    else        parse_error <= (state == S_ERROR) || pid_error;
  end

  //-------------------------------------------------------------------------
  // PID name helper
  //-------------------------------------------------------------------------
  function automatic string pid_name(input logic [3:0] p);
    case (p)
      PID_OUT:   return "OUT";
      PID_IN:    return "IN";
      PID_SOF:   return "SOF";
      PID_SETUP: return "SETUP";
      PID_DATA0: return "DATA0";
      PID_DATA1: return "DATA1";
      PID_ACK:   return "ACK";
      PID_NAK:   return "NAK";
      PID_STALL: return "STALL";
      default:   return "UNKNOWN";
    endcase
  endfunction

endmodule
