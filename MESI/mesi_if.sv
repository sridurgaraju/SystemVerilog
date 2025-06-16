interface mesi_if ();
  
  parameter N = 2;

  logic        clk;
  logic        rst;

  // For N=2 caches
  logic [31:0] addr[N];
  logic [N-1:0]  read_req, write_req;
  logic [N-1:0]  mem_read, mem_write;
  logic [1:0]    state[N];
  
  // Define cache state encoding
  parameter I = 2'b00;
  parameter S = 2'b01;
  parameter E = 2'b10;
  parameter M = 2'b11;

  // 1. Read Miss in Invalid → Mem Read & Proper State
  genvar i;
  generate
    for (i = 0; i < N; i++) begin : gen_assert_read_invalid
      assert property (@(posedge clk)
                       disable iff (!rst)
        read_req[i] && state[i] == I |=> mem_read[i] ##1 (state[i] == E || state[i] == S)
      )
      else $error("Cache %0d: Read miss in I did not lead to mem read + valid state", i);
    end
  endgenerate

  // 2. Shared State Requires Another Sharer (for N = 2)
  assert property (@(posedge clk)
                   disable iff (!rst)
    state[0] == S |-> (state[1] == S || state[1] == M) && (addr[0] == addr[1])
  )
  else $error("Cache 0: In Shared state without matching sharer/modifier");

  // i = 1, j = 0
  assert property (@(posedge clk)
                   disable iff (!rst)
    state[1] == S |-> (state[0] == S || state[0] == M) && (addr[1] == addr[0])
  )
  else $error("Cache 1: In Shared state without matching sharer/modifier");

  // 3. Modified is Unique (for N = 2)
  assert property (@(posedge clk)
                   disable iff (!rst)
    state[0] == M |-> !(state[1] == M && addr[0] == addr[1])
  )
  else $error("Cache 0 & 1 both in M state for same address!");

  assert property (@(posedge clk)
                   disable iff (!rst)
    state[1] == M |-> !(state[0] == M && addr[1] == addr[0])
  )
  else $error("Cache 1 & 0 both in M state for same address!");
    
genvar j;
generate
  for (i = 0; i < N; i++) begin : gen_i
    for (j = 0; j < N; j++) begin : gen_j
      if (i != j) begin : gen_asserts

        // I ➜ E on read if no other cache has the line
        property I_to_E_transition;
          @(posedge clk)
          disable iff (!rst)
          (state[i] == I && read_req[i] && !(state[j] != I && addr[i] == addr[j])) |=> state[i] == E;
        endproperty
        assert property (I_to_E_transition)
          else $error("Cache %0d: I➜E transition failed", i);

        // I ➜ S on read if other cache has the line
        property I_to_S_transition;
          @(posedge clk)
          disable iff (!rst)
          (state[i] == I && read_req[i] && (state[j] != I && addr[i] == addr[j])) |=> state[i] == S;
        endproperty
        assert property (I_to_S_transition)
          else $error("Cache %0d: I➜S transition failed", i);

        // S ➜ M on write should trigger invalidate
        property S_to_M_transition;
          @(posedge clk)
          disable iff (!rst)
          (state[i] == S && write_req[i] && addr[i] == addr[j]) |=> state[i] == M;
        endproperty
        assert property (S_to_M_transition)
          else $error("Cache %0d: S➜M transition failed", i);

        // E ➜ M on write (exclusive owner modifies)
        property E_to_M_transition;
          @(posedge clk)
          disable iff (!rst)
          (state[i] == E && write_req[i]) |=> state[i] == M;
        endproperty
        assert property (E_to_M_transition)
          else $error("Cache %0d: E➜M transition failed", i);

        // M ➜ I on snoop invalidate
        property M_to_I_on_snoop;
          @(posedge clk)
          disable iff (!rst)
          (state[i] == M && write_req[j] && addr[i] == addr[j]) |=> state[i] == I;
        endproperty
        assert property (M_to_I_on_snoop)
          else $error("Cache %0d: M➜I transition on snoop failed", i);

        // E ➜ S on snoop read
        property E_to_S_on_snoop;
          @(posedge clk)
          disable iff (!rst)
          (state[i] == E && read_req[j] && addr[i] == addr[j]) |=> state[i] == S;
        endproperty
        assert property (E_to_S_on_snoop)
          else $error("Cache %0d: E➜S transition on snoop failed", i);

        // S ➜ I on snoop write
        property S_to_I_on_snoop;
          @(posedge clk)
          disable iff (!rst)
          (state[i] == S && write_req[j] && addr[i] == addr[j]) |=> state[i] == I;
        endproperty
        assert property (S_to_I_on_snoop)
          else $error("Cache %0d: S➜I transition on snoop failed", i);

      end
    end
  end
endgenerate


endinterface
