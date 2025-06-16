module top_system #(parameter N = 2) (
    input  logic clk,
    input  logic rst,
    input  logic [N-1:0] read_req,
    input  logic [N-1:0] write_req,
    input  logic [31:0] addr [N],

    output logic [N-1:0] mem_read,
    output logic [N-1:0] mem_write,
    output logic [1:0] state [N]
);

  logic [N-1:0] snoop_read, snoop_read_excl, snoop_invalidate, shared_hit;

    genvar i;
    generate
        for (i = 0; i < N; i = i + 1) begin : cache_inst
            cache_controller cache (
                .clk(clk),
                .rst(rst),
                .read_req(read_req[i]),
                .write_req(write_req[i]),
                .addr(addr[i]),
                .snoop_read(snoop_read[i]),
                .snoop_read_excl(snoop_read_excl[i]),
                .snoop_invalidate(snoop_invalidate[i]),
                .shared_hit(shared_hit[i]),
                .mem_read(mem_read[i]),
                .mem_write(mem_write[i]),
                .state(state[i])
            );
        end
    endgenerate

    snoopy_bus #(N) bus (
        .clk(clk),
        .rst(rst),
        .req_read(read_req),
        .req_write(write_req),
        .addr(addr),
        .state(state),
        .snoop_read(snoop_read),
        .snoop_read_excl(snoop_read_excl),
      .snoop_invalidate(snoop_invalidate),
      .shared_hit(shared_hit)
    );

endmodule
