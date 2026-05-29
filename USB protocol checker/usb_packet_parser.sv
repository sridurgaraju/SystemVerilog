`timescale 1ns / 1ps
//=============================================================================
// USB 2.0 Packet Parser - Step 1
// Decodes: Token (IN/OUT/SETUP/SOF), Data (DATA0/DATA1), Handshake (ACK/NAK/STALL)
// Includes: PID complement check
// Next steps: CRC checker, SVA assertions, BFM
//=============================================================================

//-----------------------------------------------------------------------------
// PID definitions - upper nibble, lower nibble must be complement
//-----------------------------------------------------------------------------
package usb_pkg;

  // Token PIDs
  parameter logic [3:0] PID_OUT   = 4'b0001;
  parameter logic [3:0] PID_IN    = 4'b1001;
  parameter logic [3:0] PID_SOF   = 4'b0101;
  parameter logic [3:0] PID_SETUP = 4'b1101;

  // Data PIDs
  parameter logic [3:0] PID_DATA0 = 4'b0011;
  parameter logic [3:0] PID_DATA1 = 4'b1011;

  // Handshake PIDs
  parameter logic [3:0] PID_ACK   = 4'b0010;
  parameter logic [3:0] PID_NAK   = 4'b1010;
  parameter logic [3:0] PID_STALL = 4'b1110;

  // Packet type classification
  typedef enum logic [1:0] {
    PKT_TOKEN     = 2'b00,
    PKT_DATA      = 2'b01,
    PKT_HANDSHAKE = 2'b10,
    PKT_UNKNOWN   = 2'b11
  } pkt_type_e;

endpackage

//-----------------------------------------------------------------------------
// Top-level packet parser
// Input:  serial bit stream (1 bit per clock, LSB first)
// Output: decoded packet fields, valid strobes, error flags
//-----------------------------------------------------------------------------
module usb_packet_parser
  import usb_pkg::*;
(
  input  logic        clk,
  input  logic        rst_n,

  // Serial bit stream input (from NRZI decoder / BFM)
  input  logic        bit_in,       // incoming bit, sampled on posedge clk
  input  logic        bit_valid,    // high when bit_in is valid

  // Packet framing (driven by BFM or upstream framer)
  input  logic        pkt_start,    // pulses high on first bit after SYNC
  input  logic        pkt_end,      // pulses high on EOP

  //--- Decoded outputs ---

  // PID
  output logic [7:0]  pid_byte,     // raw 8-bit PID byte
  output logic [3:0]  pid,          // lower nibble (actual PID)
  output pkt_type_e   pkt_type,     // TOKEN / DATA / HANDSHAKE / UNKNOWN
  output logic        pid_valid,    // PID decoded and complement check passed
  output logic        pid_error,    // complement check FAILED

  // Token packet fields (valid when token_pkt_valid pulses)
  output logic [6:0]  token_addr,   // 7-bit device address
  output logic [3:0]  token_endp,   // 4-bit endpoint
  output logic [10:0] sof_frame_num,// frame number (SOF packets only)
  output logic        token_pkt_valid,

  // Data packet fields (valid when data_pkt_valid pulses)
  output logic [7:0]  data_payload [0:63], // up to 64 bytes (FS max)
  output logic [5:0]  data_len,     // number of payload bytes received
  output logic        data_pkt_valid,

  // Handshake (valid when hs_pkt_valid pulses)
  output logic        hs_pkt_valid,

  // General error flag
  output logic        parse_error
);

  //-------------------------------------------------------------------------
  // Internal signals
  //-------------------------------------------------------------------------
  logic [7:0]  shift_reg;       // 8-bit shift register, fills LSB first
  logic [2:0]  bit_count;       // counts 0..7 within a byte
  logic [5:0]  byte_count;      // which byte in the packet
  logic        byte_ready;      // pulses when a full byte has been shifted in

  logic [7:0]  current_byte;    // latched byte when byte_ready

  // Parser FSM
  typedef enum logic [2:0] {
    S_IDLE,       // waiting for pkt_start
    S_PID,        // collecting PID byte (byte 0)
    S_TOKEN,      // collecting token payload (ADDR+ENDP or frame number)
    S_DATA,       // collecting data payload bytes
    S_HANDSHAKE,  // handshake has no payload, go straight to done
    S_DONE,       // packet complete, output valid strobes
    S_ERROR       // parse error
  } state_e;

  state_e state, next_state;

  // Token shift buffer: 16 bits (ADDR[6:0] + ENDP[3:0] + CRC5[4:0])
  logic [15:0] token_buf;
  logic [3:0]  token_byte_cnt;

  // Data buffer byte counter
  logic [5:0]  data_byte_idx;

  //-------------------------------------------------------------------------
  // Bit → byte shift register
  // USB sends LSB first, so shift into MSB and let byte fill from right
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
        // LSB first: shift right, incoming bit enters at MSB position [7]
        shift_reg <= {bit_in, shift_reg[7:1]};
        if (bit_count == 3'd7) begin
          bit_count  <= 3'd0;
          byte_ready <= 1'b1;
        end else begin
          bit_count <= bit_count + 1'b1;
        end
      end
    end
  end

  assign current_byte = shift_reg; // valid when byte_ready pulses

  //-------------------------------------------------------------------------
  // Parser FSM - sequential
  //-------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      state <= S_IDLE;
    else
      state <= next_state;
  end

  //-------------------------------------------------------------------------
  // Parser FSM - combinational next state
  //-------------------------------------------------------------------------
  always_comb begin
    next_state = state;
    case (state)
      S_IDLE:      if (pkt_start)  next_state = S_PID;

      S_PID: begin
              if (byte_ready) begin
                if (current_byte[7:4] != ~current_byte[3:0]) begin
                   next_state = S_ERROR;
                end else begin
                   case (current_byte[3:0])
                     PID_OUT, PID_IN, PID_SOF, PID_SETUP: next_state = S_TOKEN;
                     PID_DATA0, PID_DATA1:                next_state = S_DATA;
                     PID_ACK, PID_NAK, PID_STALL:         next_state = S_HANDSHAKE;
                     default:                              next_state = S_ERROR;
                   endcase
                end
              end
             end

      S_TOKEN:     if (pkt_end)    next_state = S_DONE;
      S_DATA:      if (pkt_end)    next_state = S_DONE;
      S_HANDSHAKE:                 next_state = S_DONE;
      S_DONE:                      next_state = S_IDLE;
      S_ERROR:     if (pkt_end)    next_state = S_IDLE;
      default:                     next_state = S_IDLE;
    endcase
  end

  //-------------------------------------------------------------------------
  // PID decode and complement check
  //-------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      pid_byte  <= 8'h00;
      pid       <= 4'h0;
      pkt_type  <= PKT_UNKNOWN;
      pid_valid <= 1'b0;
      pid_error <= 1'b0;
    end else begin
      pid_valid <= 1'b0;
      pid_error <= 1'b0;

      if (state == S_PID && byte_ready) begin
        pid_byte <= current_byte;
        pid      <= current_byte[3:0];

        // Complement check: upper nibble must be bitwise inverse of lower nibble
        if (current_byte[7:4] != ~current_byte[3:0]) begin
          pid_error <= 1'b1;
          $display("[USB PARSER] ERROR: PID complement check FAILED. Byte=0x%02X", current_byte);
        end else begin
          pid_valid <= 1'b1;
          // Classify packet type from lower nibble
          case (current_byte[3:0])
            PID_OUT, PID_IN, PID_SOF, PID_SETUP: pkt_type <= PKT_TOKEN;
            PID_DATA0, PID_DATA1:                 pkt_type <= PKT_DATA;
            PID_ACK, PID_NAK, PID_STALL:          pkt_type <= PKT_HANDSHAKE;
            default:                               pkt_type <= PKT_UNKNOWN;
          endcase
          $display("[USB PARSER] PID decoded: 0x%X (%s)",
            current_byte[3:0], pid_name(current_byte[3:0]));
        end
      end
    end
  end

  //-------------------------------------------------------------------------
  // Token packet: collect 2 bytes (ADDR[6:0] + ENDP[3:0] + CRC5[4:0])
  //   Byte 1: ADDR[6:0] + ENDP[0]
  //   Byte 2: ENDP[3:1] + CRC5[4:0]
  // SOF token: 11-bit frame number + CRC5
  //-------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      token_buf       <= 16'h0;
      token_byte_cnt  <= 4'd0;
      token_addr      <= 7'h00;
      token_endp      <= 4'h0;
      sof_frame_num   <= 11'h000;
      token_pkt_valid <= 1'b0;
    end else begin
      token_pkt_valid <= 1'b0;

      if (state == S_PID && byte_ready && !pid_error)
        token_byte_cnt <= 4'd0;

      if (state == S_TOKEN) begin
        if (byte_ready) begin
          token_buf      <= {current_byte, token_buf[15:8]}; // shift in
          token_byte_cnt <= token_byte_cnt + 1'b1;
        end
        if (pkt_end) begin
          // Extract fields from the 16-bit buffer
          // token_buf[10:0] = ADDR[6:0] | ENDP[3:0] (bits 10:7) | CRC5 upper bits
          if (pid == PID_SOF) begin
            sof_frame_num   <= token_buf[10:0];
            $display("[USB PARSER] SOF frame=0x%03X", token_buf[10:0]);
          end else begin
            token_addr <= token_buf[6:0];
            token_endp <= token_buf[10:7];
            $display("[USB PARSER] TOKEN addr=0x%02X endp=0x%X pid=%s",
              token_buf[6:0], token_buf[10:7], pid_name(pid));
          end
          token_pkt_valid <= 1'b1;
        end
      end
    end
  end

  //-------------------------------------------------------------------------
  // Data packet: collect payload bytes until EOP
  //-------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      data_len       <= 6'd0;
      data_byte_idx  <= 6'd0;
      data_pkt_valid <= 1'b0;
      for (int i = 0; i < 64; i++) data_payload[i] <= 8'h00;
    end else begin
      data_pkt_valid <= 1'b0;

      if (state == S_PID && byte_ready && !pid_error)
        data_byte_idx <= 6'd0;

      if (state == S_DATA) begin
        if (byte_ready && data_byte_idx < 6'd62) begin
          // Last 2 bytes are CRC16 - we collect them but mark them separately
          // CRC checking will be added in Step 3
          data_payload[data_byte_idx] <= current_byte;
          data_byte_idx <= data_byte_idx + 1'b1;
        end
        if (pkt_end) begin
          // data_byte_idx includes CRC16 (2 bytes); payload = total - 2
          data_len       <= (data_byte_idx >= 2) ? data_byte_idx - 6'd2 : 6'd0;
          data_pkt_valid <= 1'b1;
          $display("[USB PARSER] DATA%0d payload_bytes=%0d (excl CRC16)",
            (pid == PID_DATA1) ? 1 : 0,
            (data_byte_idx >= 2) ? data_byte_idx - 6'd2 : 0);
        end
      end
    end
  end

  //-------------------------------------------------------------------------
  // Handshake packet: no payload - just flag it
  //-------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      hs_pkt_valid <= 1'b0;
    end else begin
      hs_pkt_valid <= 1'b0;
      if (state == S_HANDSHAKE) begin
        hs_pkt_valid <= 1'b1;
        $display("[USB PARSER] HANDSHAKE: %s", pid_name(pid));
      end
    end
  end

  //-------------------------------------------------------------------------
  // General parse error output
  //-------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      parse_error <= 1'b0;
    else
      parse_error <= (state == S_ERROR) || pid_error;
  end

  //-------------------------------------------------------------------------
  // Helper function: PID name for display
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
