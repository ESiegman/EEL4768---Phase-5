`default_nettype none

// A set associative cache. `hart` instantiates two of them, one as the
// instruction cache and one as the data cache, and passes the same five
// parameters to both. The replacement, write, and prefetch rules are in
// `documentation/phase_5.pdf`.
module cache #(
    // Replacement policy: 0 = LRU, 1 = PLRU.
    parameter EVICT_POLICY = 0,
    // Lines per set: 1 = direct mapped, the number of lines = fully
    // associative.
    parameter NUM_WAYS     = 1,
    // Data capacity in KB. Tags, valid bits and dirty bits are not counted.
    parameter CACHE_SIZE   = 16,
    // Bytes per line.
    parameter BLOCK_SIZE   = 128,
    // Next-line prefetch on a load miss: 1 = on, 0 = off.
    parameter PREFETCH_EN  = 0
) (
    // Global clock.
    input  wire                    i_clk,
    // Synchronous active-high reset.
    input  wire                    i_rst,
    // CPU side: the phase 4 data memory port, plus `o_ready` to stall the
    // pipeline while the cache waits on main memory. How the cache tells the
    // pipeline to stall is up to you, so you can change these ports.
    input  wire                    i_ren,
    input  wire                    i_wen,
    input  wire [31:0]             i_addr,
    input  wire [ 3:0]             i_mask,
    input  wire [31:0]             i_wdata,
    output wire                    o_ready,
    output wire [31:0]             o_rdata,
    // Memory side: one whole line (BLOCK_SIZE bytes) per transfer.
    //
    // Line read request, high for one cycle per request.
    output wire                    o_mem_rd,
    // Line-aligned address of the line to read.
    output wire [31:0]             o_mem_rd_addr,
    // High for one cycle when a requested line arrives. Memory answers
    // requests in order, after a delay set by the testbench: wait for this
    // signal instead of counting cycles.
    input  wire                    i_mem_valid,
    // The whole line, valid while `i_mem_valid` is high.
    input  wire [BLOCK_SIZE*8-1:0] i_mem_rd_data,
    // Line write, high for one cycle per write. Memory applies it on that
    // clock edge.
    output wire                    o_mem_wr,
    // Line-aligned address of the write.
    output wire [31:0]             o_mem_wr_addr,
    // Write data, in place within the line.
    output wire [BLOCK_SIZE*8-1:0] o_mem_wr_data,
    // One bit per byte: all ones for a dirty write-back, only the stored
    // bytes for a write-through.
    output wire [BLOCK_SIZE-1:0]   o_mem_wr_be
);

    localparam OFFSET_BITS = $clog2(BLOCK_SIZE);
    localparam NUM_SETS = (CACHE_SIZE * 1024) / (BLOCK_SIZE * NUM_WAYS);
    localparam SET_BITS = $clog2((NUM_SETS > 1) ? NUM_SETS : 2);
    localparam WAY_BITS = $clog2((NUM_WAYS > 1) ? NUM_WAYS : 2);
    localparam [31:0] BLOCK_BYTES = BLOCK_SIZE;

    // BASE-CACHE INTEGRATION BOUNDARY
    // The starting cache.v has no tag/data arrays or demand/write controller.
    // Replace these tie-offs with that implementation's signals. They keep
    // the feature logic inert until connected, rather than inventing hits.
    // See documentation/plru_prefetch.md for the timing contract.
    wire base_load_miss = 1'b0;
    wire [31:0] base_miss_addr = 32'b0;
    wire base_ready = 1'b0;
    wire [31:0] base_rdata = 32'b0;
    wire base_mem_wr = 1'b0;
    wire [31:0] base_mem_wr_addr = 32'b0;
    wire [BLOCK_SIZE*8-1:0] base_mem_wr_data = {BLOCK_SIZE*8{1'b0}};
    wire [BLOCK_SIZE-1:0] base_mem_wr_be = {BLOCK_SIZE{1'b0}};
    wire [NUM_SETS*NUM_WAYS-1:0] base_valid = {NUM_SETS*NUM_WAYS{1'b0}};
    wire base_touch = 1'b0;
    wire [SET_BITS-1:0] base_touch_set = {SET_BITS{1'b0}};
    wire [WAY_BITS-1:0] base_touch_way = {WAY_BITS{1'b0}};
    wire [SET_BITS-1:0] base_replacement_set = {SET_BITS{1'b0}};
    wire [WAY_BITS-1:0] base_lru_victim_way = {WAY_BITS{1'b0}};
    // Tag lookup results for prefetch_lookup_addr and prefetch_fill_addr.
    wire base_prefetch_cached = 1'b0;
    wire base_prefetch_fill_cached = 1'b0;
    // Old contents of (prefetch_fill_set, replacement_way), before filling.
    wire base_prefetch_victim_dirty = 1'b0;
    wire [31:0] base_prefetch_victim_addr = 32'b0;
    wire [BLOCK_SIZE*8-1:0] base_prefetch_victim_data = {BLOCK_SIZE*8{1'b0}};

    // Four in-flight records track the in-order memory replies. A demand
    // reserves another slot for its next-line request, so that prefetch can
    // always issue in the following cycle, even at arbitrary memory latency.
    localparam REQUEST_DEPTH = 4;
    localparam [2:0] REQUEST_LIMIT = REQUEST_DEPTH;
    localparam [2:0] DEMAND_LIMIT = REQUEST_DEPTH-1;
    reg [1:0] request_head;
    reg [1:0] request_tail;
    reg [2:0] request_count;
    wire [REQUEST_DEPTH-1:0] request_prefetch;
    wire [31:0] request_addr [0:REQUEST_DEPTH-1];
    reg prefetch_pending;
    reg [31:0] pending_addr;

    wire [31:0] demand_line_addr = {base_miss_addr[31:OFFSET_BITS], {OFFSET_BITS{1'b0}}};
    wire [31:0] next_line_addr = demand_line_addr + BLOCK_BYTES;
    wire [31:0] prefetch_lookup_addr = prefetch_pending ? pending_addr : next_line_addr;
    wire [REQUEST_DEPTH:0] demand_duplicate;
    wire [REQUEST_DEPTH:0] prefetch_duplicate;
    assign demand_duplicate[0] = 1'b0;
    assign prefetch_duplicate[0] = 1'b0;

    wire request_pop = ~i_rst & i_mem_valid & (request_count != 0);
    wire demand_return = request_pop & ~request_prefetch[request_head];
    wire [31:0] demand_return_addr = request_addr[request_head];
    wire [BLOCK_SIZE*8-1:0] demand_return_data = i_mem_rd_data;
    wire prefetch_return = request_pop & request_prefetch[request_head];
    wire [31:0] prefetch_fill_addr = request_addr[request_head];
    wire [BLOCK_SIZE*8-1:0] prefetch_fill_data = i_mem_rd_data;
    wire prefetch_fill = prefetch_return & ~base_prefetch_fill_cached;
    wire [SET_BITS-1:0] cpu_set;
    wire [SET_BITS-1:0] prefetch_fill_set;
    generate
        if (NUM_SETS == 1) begin : single_set
            assign cpu_set = {SET_BITS{1'b0}};
            assign prefetch_fill_set = {SET_BITS{1'b0}};
        end else begin : multiple_sets
            assign cpu_set = i_addr[OFFSET_BITS +: SET_BITS];
            assign prefetch_fill_set = prefetch_fill_addr[OFFSET_BITS +: SET_BITS];
        end
    endgenerate

    // Collision option B: install the prefetch first, then retry the CPU
    // access next cycle. Also serialize stores against a dirty write-back,
    // since there is only one memory write port, even for different sets.
    wire prefetch_cpu_stall = prefetch_fill & (i_ren | i_wen) &
                             ((cpu_set == prefetch_fill_set) |
                              (i_wen & base_prefetch_victim_dirty));
    wire prefetch_writeback = prefetch_fill & base_prefetch_victim_dirty;
    assign o_ready = base_ready & ~prefetch_cpu_stall & ~i_rst;
    assign o_rdata = base_rdata;
    assign o_mem_wr = ~i_rst & (prefetch_writeback | (base_mem_wr & ~prefetch_cpu_stall));
    assign o_mem_wr_addr = prefetch_writeback ? base_prefetch_victim_addr : base_mem_wr_addr;
    assign o_mem_wr_data = prefetch_writeback ? base_prefetch_victim_data : base_mem_wr_data;
    assign o_mem_wr_be = prefetch_writeback ? {BLOCK_SIZE{1'b1}} : base_mem_wr_be;

    // A pending prefetch has priority over the next demand. Tag lookup is
    // repeated at issue, and outstanding lines are not requested twice.
    wire prefetch_send = ~i_rst & prefetch_pending & ~base_prefetch_cached &
                         ~prefetch_duplicate[REQUEST_DEPTH];
    wire demand_room = (PREFETCH_EN == 0) ?
                       ((request_count < REQUEST_LIMIT) | request_pop) :
                       ((request_count < DEMAND_LIMIT) |
                        ((request_count == DEMAND_LIMIT) & request_pop));
    wire demand_accept = ~i_rst & base_load_miss & ~prefetch_cpu_stall &
                         ~prefetch_send & demand_room & ~demand_duplicate[REQUEST_DEPTH];
    assign o_mem_rd = prefetch_send | demand_accept;
    assign o_mem_rd_addr = prefetch_send ? pending_addr : demand_line_addr;

    always @(posedge i_clk) begin
        if (i_rst) begin
            request_head <= 2'b0;
            request_tail <= 2'b0;
            request_count <= 3'b0;
            prefetch_pending <= 1'b0;
            pending_addr <= 32'b0;
        end else begin
            if (o_mem_rd)
                request_tail <= request_tail + 2'd1;
            if (request_pop)
                request_head <= request_head + 2'd1;
            case ({o_mem_rd, request_pop})
                2'b10: request_count <= request_count + 3'd1;
                2'b01: request_count <= request_count - 3'd1;
                default: request_count <= request_count;
            endcase
            // A skipped pending prefetch is consumed just like a sent one.
            // Its reserved slot ensures no additional queue wait is needed.
            prefetch_pending <= (PREFETCH_EN != 0) & demand_accept;
            if (demand_accept)
                pending_addr <= next_line_addr;
        end
    end

    genvar slot;
    generate
        for (slot = 0; slot < REQUEST_DEPTH; slot = slot + 1) begin : requests
            localparam [1:0] SLOT = slot;
            reg valid;
            reg is_prefetch;
            reg [31:0] addr;
            assign request_prefetch[slot] = is_prefetch;
            assign request_addr[slot] = addr;
            assign demand_duplicate[slot+1] = demand_duplicate[slot] |
                        (valid & (addr == demand_line_addr));
            assign prefetch_duplicate[slot+1] = prefetch_duplicate[slot] |
                        (valid & (addr == prefetch_lookup_addr));
            always @(posedge i_clk) begin
                if (i_rst) begin
                    valid <= 1'b0;
                    is_prefetch <= 1'b0;
                    addr <= 32'b0;
                end else if (o_mem_rd && (request_tail == SLOT)) begin
                    valid <= 1'b1;
                    is_prefetch <= prefetch_send;
                    addr <= o_mem_rd_addr;
                end else if (request_pop && (request_head == SLOT)) begin
                    valid <= 1'b0;
                end
            end
        end
    endgenerate

    // Select a victim at fill time, including for a returning prefetch.
    // The base controller uses replacement_way for a demand fill as well.
    wire [SET_BITS-1:0] replacement_set = prefetch_fill ? prefetch_fill_set : base_replacement_set;
    wire [WAY_BITS-1:0] replacement_way;
    wire [WAY_BITS-1:0] plru_victim [0:NUM_SETS-1];
    genvar set_index;
    generate
        if (EVICT_POLICY == 1) begin : use_plru
            for (set_index = 0; set_index < NUM_SETS; set_index = set_index + 1) begin : sets
                localparam [SET_BITS-1:0] SET = set_index;
                wire fill_this_set = prefetch_fill & (prefetch_fill_set == SET);
                wire touch_this_set = base_touch & ~prefetch_cpu_stall & (base_touch_set == SET);
                plru #(.NUM_WAYS(NUM_WAYS)) replacement (
                    .i_clk(i_clk),
                    .i_rst(i_rst),
                    .i_valid(base_valid[set_index*NUM_WAYS +: NUM_WAYS]),
                    .i_touch(fill_this_set | touch_this_set),
                    .i_touch_way(fill_this_set ? replacement_way : base_touch_way),
                    .o_victim_way(plru_victim[set_index])
                );
            end
            assign replacement_way = plru_victim[replacement_set];
        end else begin : use_base_lru
            assign replacement_way = base_lru_victim_way;
        end
    endgenerate

endmodule

`default_nettype wire
