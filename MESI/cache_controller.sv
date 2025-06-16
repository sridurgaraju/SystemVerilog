module cache_controller (
    input  logic clk,
    input  logic rst,
    input  logic read_req,
    input  logic write_req,
    input  logic [31:0] addr,
    input  logic snoop_read,
    input  logic snoop_read_excl,
    input  logic snoop_invalidate,
    input logic shared_hit,
    output logic mem_read,
    output logic mem_write,
    output logic [1:0] state
);

    typedef enum logic [1:0] {
        INVALID   = 2'b00,
        SHARED    = 2'b01,
        EXCLUSIVE = 2'b10,
        MODIFIED  = 2'b11
    } mesi_state_t;

    mesi_state_t current_state, next_state;

    // FSM sequential block
    always_ff @(posedge clk or posedge rst) begin
        if (rst)
            current_state <= INVALID;
        else
            current_state <= next_state;
    end

    // FSM combinational block
    always_comb begin
        // Default assignments
        next_state = current_state;
        mem_read   = 0;
        mem_write  = 0;

        case (current_state)
            INVALID: begin
                if (read_req) begin
                    // Decide between E or S based on snoop activity
                  if (shared_hit)
                        next_state = SHARED;
                    else
                        next_state = EXCLUSIVE;
                    mem_read = 1;
                end else if (write_req) begin
                    next_state = MODIFIED;
                    mem_read = 1;
                end
            end

            SHARED: begin
                if (write_req) begin
                    next_state = MODIFIED;
                    mem_write = 1;
                end else if (snoop_invalidate) begin
                    next_state = INVALID;
                end
            end

            EXCLUSIVE: begin
                if (write_req) begin
                    next_state = MODIFIED;
                end else if (snoop_read || snoop_read_excl) begin
                    next_state = (snoop_read) ? SHARED : INVALID;
                    if (snoop_read || snoop_read_excl)
                        mem_write = 1; // Optional: to simulate flush if needed
                end
            end

            MODIFIED: begin
                if (snoop_read) begin
                    mem_write  = 1;
                    next_state = SHARED;
                end else if (snoop_read_excl) begin
                    mem_write  = 1;
                    next_state = INVALID;
                end
            end


            default: next_state = INVALID;
        endcase
    end

    assign state = current_state;

endmodule
