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


  localparam LINE_BITS   = BLOCK_SIZE * 8;
  localparam WORDS       = BLOCK_SIZE / 4; // words per line
  localparam NUM_LINES   = (CACHE_SIZE * 1024) / BLOCK_SIZE;
  localparam NUM_SETS    = NUM_LINES / NUM_WAYS; // power of 2
  localparam OFFSET_BITS = $clog2(BLOCK_SIZE); // byte in line
  localparam WORD_BITS   = OFFSET_BITS - 2; // word in line
  localparam INDEX_BITS  = $clog2(NUM_SETS); // 0 if fully assoc.
  localparam TAG_BITS    = 32 - OFFSET_BITS - INDEX_BITS;
  localparam WAY_BITS    = $clog2(NUM_WAYS);  // 0 if direct mapped

  //widths that cannot be zero.
  localparam IDX_W = (INDEX_BITS > 0) ? INDEX_BITS : 1;
  localparam WRD_W = (WORD_BITS  > 0) ? WORD_BITS  : 1;
  localparam WAY_W = (WAY_BITS   > 0) ? WAY_BITS   : 1;
  localparam AGE_W = WAY_W; // LRU age width

  localparam S_IDLE = 1'b0;   // ready for a CPU access
  localparam S_WAIT = 1'b1;   // load miss outstanding, waiting on i_mem_valid



  //address split: | tag | set index | word | byte | 
  wire [TAG_BITS-1:0] cpu_tag       = i_addr[31 -: TAG_BITS];
  wire [31:0]         cpu_line_addr = {i_addr[31:OFFSET_BITS], {OFFSET_BITS{1'b0}}};
  wire [IDX_W-1:0]    cpu_set;
  wire [WRD_W-1:0]    cpu_word;

  //controller state
  reg         state;
  reg  [31:0] miss_addr;      // line address of the outstanding demand read
 
  //Fill: a line arriving from main memory this cycle.
    //fill - i_mem_valid is high and a read is outstanding
    //fill_is_demand - the CPU is stalled and waiting on this line
    //fill_addr - this is the line address of the read being answered
    //without the prefetch, the only outstanding read is the demand miss,
    //so every fill is a demand fill and its address is the miss_addr.

    //prefetch hook: memory answers in order, so keep a queue of outstanding
    //reads. then fill is any arriving line, fill_addr is the address at the head of
    //the queue, and fill_is_demand is the entry's demand flag.
  wire  fill = (state == S_WAIT) & i_mem_valid;
  wire fill_is_demand = 1'b1;
  wire [31:0] fill_addr = miss_addr;
  wire [TAG_BITS-1:0] fill_tag = fill_addr[31 -: TAG_BITS];
  wire [IDX_W-1:0] fill_set;

  generate
    if (INDEX_BITS > 0) begin : g_index
      assign cpu_set  = i_addr[OFFSET_BITS +: INDEX_BITS];
      assign fill_set = fill_addr[OFFSET_BITS +: INDEX_BITS];
    end else begin : g_no_index
      assign cpu_set  = 1'b0;
      assign fill_set = 1'b0;
    end
    if (WORD_BITS > 0) begin : g_word
      assign cpu_word = i_addr[2 +: WORD_BITS];
    end else begin : g_no_word
      assign cpu_word = 1'b0;
    end
  endgenerate

  //a fill has priority over the CPU for the arrays. With no prefetch a fill
  // only happens when the CPU is waiting so this will never cost a cycle.

  wire idle   = (state == S_IDLE);
  wire cpu_go = idle & ~fill;

  // The set whose replacement state and victim are used this cycle: the fill
  // set while a line is arriving, otherwise the CPU's set.
  wire [IDX_W-1:0] acc_set = fill ? fill_set : cpu_set;



  //storage and per-way reads.
  //one generate block per way, so any NUM_WAYS works

  wire [NUM_WAYS-1:0] hit_vec; // way w holds the CPU's line
  wire [NUM_WAYS-1:0] acc_valid; // valid bits of acc_set
 
  // OR chains: element w+1 folds in way w. At most one way is selected, so
  // the last element is the selected way's value.
  wire [WAY_W-1:0]     hit_way_c   [0:NUM_WAYS];
  wire [LINE_BITS-1:0] hit_line_c  [0:NUM_WAYS];
  wire [TAG_BITS-1:0]  vic_tag_c   [0:NUM_WAYS];
  wire [LINE_BITS-1:0] vic_line_c  [0:NUM_WAYS];
  wire                 vic_dirty_c [0:NUM_WAYS];
 
  assign hit_way_c[0]   = {WAY_W{1'b0}};
  assign hit_line_c[0]  = {LINE_BITS{1'b0}};
  assign vic_tag_c[0]   = {TAG_BITS{1'b0}};
  assign vic_line_c[0]  = {LINE_BITS{1'b0}};
  assign vic_dirty_c[0] = 1'b0;
 
  wire [WAY_W-1:0]     hit_way  = hit_way_c[NUM_WAYS];
  wire [LINE_BITS-1:0] hit_line = hit_line_c[NUM_WAYS];
  wire                 hit      = |hit_vec;
 
  wire [WAY_W-1:0]     victim;// way the fill goes into
  wire [TAG_BITS-1:0]  vic_tag   = vic_tag_c[NUM_WAYS];
  wire [LINE_BITS-1:0] vic_line  = vic_line_c[NUM_WAYS];
  wire                 vic_dirty = vic_dirty_c[NUM_WAYS]; // valid & dirty
 
  // CPU access outcomes (only while idle and no fill is using the arrays).
  wire load_hit   = cpu_go & i_ren & hit;
  wire load_miss  = cpu_go & i_ren & ~hit;
  wire store_hit  = cpu_go & i_wen & hit;
  wire store_miss = cpu_go & i_wen & ~hit;
 
  wire [LINE_BITS-1:0] store_line; // hit line with the store merged in
 
  genvar w;
  generate
    for (w = 0; w < NUM_WAYS; w = w + 1) begin : way
      localparam [WAY_W-1:0] ID = w;
 
      reg  [NUM_SETS-1:0]  valid; // flat so it can be reset in one go
      reg                  dirty [0:NUM_SETS-1]; // only meaningful when valid
      reg  [TAG_BITS-1:0]  tags  [0:NUM_SETS-1];
      reg  [LINE_BITS-1:0] data  [0:NUM_SETS-1];
 
      // CPU lookup port.
      assign hit_vec[w] = valid[cpu_set] & (tags[cpu_set] == cpu_tag);
      assign hit_way_c[w+1]  = hit_way_c[w]  | (hit_vec[w] ? ID : {WAY_W{1'b0}});
      assign hit_line_c[w+1] = hit_line_c[w] | (hit_vec[w] ? data[cpu_set] : {LINE_BITS{1'b0}});
 
      // Replacement / victim port.
      wire is_victim = (victim == ID);
      assign acc_valid[w]     = valid[acc_set];
      assign vic_tag_c[w+1]   = vic_tag_c[w]   | (is_victim ? tags[acc_set] : {TAG_BITS{1'b0}});
      assign vic_line_c[w+1]  = vic_line_c[w]  | (is_victim ? data[acc_set] : {LINE_BITS{1'b0}});
      assign vic_dirty_c[w+1] = vic_dirty_c[w] | (is_victim & valid[acc_set] & dirty[acc_set]);
 
      always @(posedge i_clk) begin
        if (i_rst) begin
          valid <= {NUM_SETS{1'b0}};
        end else if (fill && is_victim) begin
          // Install the arriving line clean (a dirty victim is written back
          // on the memory port this same cycle).
          valid[acc_set] <= 1'b1;
          dirty[acc_set] <= 1'b0;
          tags[acc_set]  <= fill_tag;
          data[acc_set]  <= i_mem_rd_data;
        end else if (store_hit && hit_vec[w]) begin
          // Write-back: update the cache only and mark the line dirty.
          dirty[cpu_set] <= 1'b1;
          data[cpu_set]  <= store_line;
        end
      end
    end
  endgenerate

    //word select and store merge. word k of a line is bits [32k +: 32];
    //the variable select uses a concatenation instead of multiply.
  wire [31:0] hit_word;
  wire [31:0] fill_word;
  generate
    if (WORD_BITS > 0) begin : g_wsel
      assign hit_word  = hit_line[{cpu_word, 5'b00000} +: 32];
      assign fill_word = i_mem_rd_data[{cpu_word, 5'b00000} +: 32];
    end else begin : g_wsel1
      assign hit_word  = hit_line[31:0];
      assign fill_word = i_mem_rd_data[31:0];
    end
  endgenerate
 
  // The stored bytes over the old word.
  wire [31:0] merged_word;
  genvar b;
  generate
    for (b = 0; b < 4; b = b + 1) begin : g_merge
      assign merged_word[8*b +: 8] = i_mask[b] ? i_wdata[8*b +: 8] : hit_word[8*b +: 8];
    end
  endgenerate
 
  // Store-hit line and write-through byte enables, one word at a time.
  wire [BLOCK_SIZE-1:0] wt_be;
  genvar k;
  generate
    for (k = 0; k < WORDS; k = k + 1) begin : g_word_k
      wire sel = (cpu_word == k);
      assign store_line[32*k +: 32] = sel ? merged_word : hit_line[32*k +: 32];
      assign wt_be[4*k +: 4]        = sel ? i_mask : 4'b0000;
    end
  endgenerate

    
  //replacement. a hit or a fill touches its way. on a fill the victim
  //is the lowest-numbered invalid way, otherwise the policy's choice
  wire             touch_en  = fill | load_hit | store_hit;
  wire [WAY_W-1:0] touch_way = fill ? victim : hit_way;
  wire [WAY_W-1:0] policy_way; // LRU or PLRU choice, ignoring valid
 
  generate
    if (NUM_WAYS == 1) begin : g_dm
      // Direct mapped: there is only one place a line can go.
      assign policy_way = 1'b0;
    end else if (EVICT_POLICY == 1) begin : g_plru
        
      // PLRU: plru.v holds the tree for one set, so there is one instance
      // per set. Only the accessed set's tree is touched each cycle, and the
      // victim comes from that set's tree. i_valid is tied to all ones
      // because cache.v already picks the lowest invalid way first.
      wire [WAY_W-1:0] plru_victim [0:NUM_SETS-1];
      genvar s;
      for (s = 0; s < NUM_SETS; s = s + 1) begin : g_set
        localparam [IDX_W-1:0] SET = s;
        plru #(
            .NUM_WAYS(NUM_WAYS)
        ) u_plru (
            .i_clk       (i_clk),
            .i_rst       (i_rst),
            .i_valid     ({NUM_WAYS{1'b1}}),
            .i_touch     (touch_en & (acc_set == SET)),
            .i_touch_way (touch_way),
            .o_victim_way(plru_victim[s])
        );
      end
      assign policy_way = plru_victim[acc_set];
    end else begin : g_lru



        
      //LRU: each way in a set stores an age.
        // 0 = most recently used
        // NUM_WAYS-1 = least recently used (the victim once the set is full)
        
        //On a hit or fill to way t:
        // * way t's age turns to 0
        // * every other valid way younger than t ages by 1
        // * every other way stays the same
        //no entries move around, so each set requires
        // NUM_WAYS * log2(NUM_WAYS) bits.

        //ages aren't reset, and it isn't required to.
        // *an invalid way's age is not used
        // *a way becomes valid only through a fill, which sets the age to 0
        // *filling the invalid way ages every valid way by 1
        //the valid ways in a set always hold the ages 0 to (count - 1) each exactly once
      localparam [AGE_W-1:0] OLDEST = NUM_WAYS - 1;
 
      reg  [NUM_WAYS*AGE_W-1:0] ages_mem [0:NUM_SETS-1];
      wire [NUM_WAYS*AGE_W-1:0] ages     = ages_mem[acc_set];
      wire [NUM_WAYS*AGE_W-1:0] ages_next;
 
      wire             touch_valid = acc_valid[touch_way];
      wire [AGE_W-1:0] t_age_c [0:NUM_WAYS];     // age of the touched way
      wire [WAY_W-1:0] lru_c   [0:NUM_WAYS];     // way whose age is OLDEST
      assign t_age_c[0] = {AGE_W{1'b0}};
      assign lru_c[0]   = {WAY_W{1'b0}};
      wire [AGE_W-1:0] t_age = t_age_c[NUM_WAYS];
 
      genvar a;
      for (a = 0; a < NUM_WAYS; a = a + 1) begin : g_age
        localparam [WAY_W-1:0] ID = a;
        wire [AGE_W-1:0] age  = ages[AGE_W*a +: AGE_W];
        wire             mine = (touch_way == ID);
        assign t_age_c[a+1] = t_age_c[a] | (mine ? age : {AGE_W{1'b0}});
        assign lru_c[a+1]   = lru_c[a]   | ((age == OLDEST) ? ID : {WAY_W{1'b0}});
        assign ages_next[AGE_W*a +: AGE_W] =
            mine                                                ? {AGE_W{1'b0}} :
            (acc_valid[a] & (~touch_valid | (age < t_age)))     ? age + 1'b1 :
                                                                  age;
      end
 
      assign policy_way = lru_c[NUM_WAYS];
 
      always @(posedge i_clk) begin
        if (i_rst) begin
        end else if (touch_en) begin
          ages_mem[acc_set] <= ages_next;
        end
      end
    end
  endgenerate
 
  // Lowest-numbered invalid way first, else the policy's way. Built from the
  // top way down so the lowest invalid way wins.
  wire [WAY_W-1:0] victim_c [0:NUM_WAYS];
  assign victim_c[NUM_WAYS] = policy_way;
  generate
    for (w = 0; w < NUM_WAYS; w = w + 1) begin : g_victim
      localparam [WAY_W-1:0] ID = w;
      assign victim_c[w] = acc_valid[w] ? victim_c[w+1] : ID;
    end
  endgenerate
  assign victim = victim_c[0];
 
  //controller
  always @(posedge i_clk) 
  begin
    if (i_rst) 
    begin
      state     <= S_IDLE;
      miss_addr <= 32'd0;
    end 
    else if (load_miss) 
    begin
      state     <= S_WAIT;
      miss_addr <= cpu_line_addr;
    end 
    else if (fill && fill_is_demand)
    begin
      state     <= S_IDLE;
    end
  end

wire demand_done = fill & fill_is_demand;

assign o_ready = ~(i_ren | i_wen) | load_hit | store_hit | store_miss | demand_done;
assign o_rdata = demand_done ? fill_word : hit_word;

  //memory port
  // Read: one-cycle request for the missing line, sent in the miss cycle.
  assign o_mem_rd      = load_miss;
  assign o_mem_rd_addr = cpu_line_addr;
 
  // Write: either the dirty victim of a fill (whole line), or a store miss
  // written through without allocating (only the stored bytes). They never
  // overlap because a store only proceeds when no fill is happening.
  wire [31:0] vic_addr;
  generate
    if (INDEX_BITS > 0) begin : g_vaddr
      assign vic_addr = {vic_tag, fill_set, {OFFSET_BITS{1'b0}}};
    end else begin : g_vaddr_fa
      assign vic_addr = {vic_tag, {OFFSET_BITS{1'b0}}};
    end
  endgenerate
 
  wire write_back = fill & vic_dirty;
 
  assign o_mem_wr      = write_back | store_miss;
  assign o_mem_wr_addr = write_back ? vic_addr : cpu_line_addr;
  assign o_mem_wr_data = write_back ? vic_line : {WORDS{i_wdata}};
  assign o_mem_wr_be   = write_back ? {BLOCK_SIZE{1'b1}} : wt_be;
 
endmodule



`default_nettype wire
