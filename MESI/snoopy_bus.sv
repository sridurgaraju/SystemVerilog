module snoopy_bus #(
    parameter N = 2  // Number of cache controllers
)(
    input  logic clk,
    input  logic rst,

    // Request inputs from each cache
    input  logic [N-1:0] req_read,
    input  logic [N-1:0] req_write,
    input  logic [31:0] addr     [N],
    input  logic [1:0] state [N],
    
    // Snoop signals to each cache
    output logic [N-1:0] snoop_read,
    output logic [N-1:0] snoop_read_excl,
    output logic [N-1:0] snoop_invalidate,
    output logic [N-1:0] shared_hit
);

    integer i, j;
    
  
always_comb begin
    // Default all snoop outputs to 0
    for (int i = 0; i < N; i++) begin
        snoop_read[i]        = 0;
        snoop_read_excl[i]   = 0;
        snoop_invalidate[i]  = 0;
        shared_hit[i]        = 0;
    end

    // Loop over all cache pairs (i = observing cache, j = requesting cache)
    for (int i = 0; i < N; i++) begin
        for (int j = 0; j < N; j++) begin
            if (i != j && addr[i] == addr[j]) begin
                // Case: j wants to read a block that i has
                if (req_read[j]) begin
                    snoop_read[i] = 1;
                    shared_hit[j] = 1;
                end

                // Case: j wants to write a block that i has
                if (req_write[j]) begin
                    snoop_read_excl[i] = 1;
                    snoop_invalidate[i] = 1;
                    shared_hit[j] = 1;
                end
            end
        end
    end

    // Handle simultaneous write conflict (both caches writing same address)
    for (int i = 0; i < N; i++) begin
        for (int j = i + 1; j < N; j++) begin
            if (addr[i] == addr[j] && req_write[i] && req_write[j]) begin
                snoop_read_excl[i]   = 1;
                snoop_invalidate[i]  = 1;
                snoop_read_excl[j]   = 1;
                snoop_invalidate[j]  = 1;
                shared_hit[i]        = 1;
                shared_hit[j]        = 1;
            end
        end
    end
end




    // Debug logging on clock edge
    always_ff @(posedge clk) begin
        if (!rst) begin
            $display("SNOOPY_BUS: Time=%0t", $time);
            for (int k = 0; k < N; k++) begin
                $display("  Cache%0d -> snoop_read=%b, snoop_read_excl=%b, snoop_invalidate=%b | addr=0x%08h | req_read=%b req_write=%b",k, snoop_read[k], snoop_read_excl[k], snoop_invalidate[k], addr[k], req_read[k], req_write[k]);
            end
        end
    end

endmodule
