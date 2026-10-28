`timescale 1ns / 1ps
`default_nettype none

// Feature-boundary tests. The base cache is not implemented: the fixture
// drives its documented hooks and checks actual cache.v feature logic.
module cache_prefetch_tb;
    parameter PREFETCH_EN = 1;
    parameter NUM_WAYS = 4;
    parameter CACHE_SIZE = 1;
    parameter BLOCK_SIZE = 16;
    localparam NUM_SETS = CACHE_SIZE*1024/(BLOCK_SIZE*NUM_WAYS);
    localparam SET_BITS = (NUM_SETS > 1) ? $clog2(NUM_SETS) : 1;
    localparam WAY_BITS = (NUM_WAYS > 1) ? $clog2(NUM_WAYS) : 1;
    localparam [31:0] A = 32'h1001_0000;
    localparam [31:0] B = 32'h2001_0000;
    localparam [31:0] C = 32'h3001_0000;
    reg clk = 1'b0;
    always #5 clk = ~clk;
    reg rst;
    reg ren;
    reg wen;
    reg [31:0] cpu_addr;
    wire ready;
    wire mem_rd;
    wire [31:0] mem_rd_addr;
    reg mem_valid;
    reg [BLOCK_SIZE*8-1:0] mem_data;
    wire mem_wr;
    wire [31:0] mem_wr_addr;
    wire [BLOCK_SIZE*8-1:0] mem_wr_data;
    wire [BLOCK_SIZE-1:0] mem_wr_be;
    reg load_miss;
    reg [31:0] miss_addr;
    reg cached;
    reg fill_cached;
    reg victim_dirty;
    reg [31:0] victim_addr;
    reg [BLOCK_SIZE*8-1:0] victim_data;
    reg store;
    reg [NUM_SETS*NUM_WAYS-1:0] valid;
    reg touch;
    reg [SET_BITS-1:0] touch_set;
    reg [WAY_BITS-1:0] touch_way;
    reg [SET_BITS-1:0] replacement_set;
    reg [WAY_BITS-1:0] sampled_victim;
    integer passed;
    integer failed;
    integer j;

    cache #(.EVICT_POLICY(1), .NUM_WAYS(NUM_WAYS), .CACHE_SIZE(CACHE_SIZE),
            .BLOCK_SIZE(BLOCK_SIZE), .PREFETCH_EN(PREFETCH_EN)) dut (
        .i_clk(clk), .i_rst(rst), .i_ren(ren), .i_wen(wen), .i_addr(cpu_addr),
        .i_mask(4'hf), .i_wdata(32'h12345678), .o_ready(ready), .o_rdata(),
        .o_mem_rd(mem_rd), .o_mem_rd_addr(mem_rd_addr),
        .i_mem_valid(mem_valid), .i_mem_rd_data(mem_data),
        .o_mem_wr(mem_wr), .o_mem_wr_addr(mem_wr_addr),
        .o_mem_wr_data(mem_wr_data), .o_mem_wr_be(mem_wr_be)
    );

    task check;
        input [511:0] label;
        input condition;
        begin
            if (condition === 1'b1)
                passed = passed + 1;
            else begin
                failed = failed + 1;
                $display("[FAIL] %0s", label);
            end
        end
    endtask

    // All inputs change at falling edges; checks precede the consuming edge.
    task step;
        begin
            @(posedge clk);
            #1;
            @(negedge clk);
        end
    endtask

    task reset_cache;
        begin
            rst = 1'b1;
            load_miss = 1'b0;
            ren = 1'b0;
            wen = 1'b0;
            cpu_addr = A;
            miss_addr = A;
            cached = 1'b0;
            fill_cached = 1'b0;
            victim_dirty = 1'b0;
            store = 1'b0;
            touch = 1'b0;
            touch_set = 0;
            touch_way = 0;
            replacement_set = 0;
            valid = 0;
            mem_valid = 1'b0;
            mem_data = {BLOCK_SIZE{8'h5a}};
            victim_addr = 32'hf001_0000;
            victim_data = {BLOCK_SIZE{8'ha5}};
            step();
            rst = 1'b0;
            #1;
            check("reset empties requests and suppresses memory writes",
                  dut.request_count == 0 && !mem_rd && !mem_wr);
        end
    endtask

    task demand;
        input [31:0] addr;
        begin
            load_miss = 1'b1;
            miss_addr = addr;
            #1;
            check("demand accepted as an aligned one-cycle read",
                  dut.demand_accept && mem_rd &&
                  mem_rd_addr == (addr & ~(BLOCK_SIZE-1)));
            step();
            load_miss = 1'b0;
        end
    endtask

    task pair;
        input [31:0] addr;
        begin
            demand(addr);
            #1;
            check("next-line request follows demand in the next cycle",
                  mem_rd && !dut.demand_accept &&
                  mem_rd_addr == ((addr & ~(BLOCK_SIZE-1)) + BLOCK_SIZE));
            step();
            #1;
            check("prefetch does not recursively prefetch", !mem_rd);
        end
    endtask

    task respond;
        input is_prefetch;
        input [31:0] addr;
        begin
            mem_valid = 1'b1;
            #1;
            check("reply matched to the oldest issued request",
                  dut.request_pop && dut.prefetch_return == is_prefetch &&
                  dut.demand_return == !is_prefetch && dut.prefetch_fill_addr == addr);
            step();
            mem_valid = 1'b0;
        end
    endtask

    initial begin
        // These forces replace only the explicitly inert base-cache hooks.
        force dut.base_load_miss = load_miss;
        force dut.base_miss_addr = miss_addr;
        force dut.base_ready = 1'b1;
        force dut.base_prefetch_cached = cached;
        force dut.base_prefetch_fill_cached = fill_cached;
        force dut.base_prefetch_victim_dirty = victim_dirty;
        force dut.base_prefetch_victim_addr = victim_addr;
        force dut.base_prefetch_victim_data = victim_data;
        force dut.base_mem_wr = store;
        force dut.base_mem_wr_addr = 32'hb001_0000;
        force dut.base_mem_wr_data = {BLOCK_SIZE{8'h3c}};
        force dut.base_mem_wr_be = {{(BLOCK_SIZE-1){1'b0}}, 1'b1};
        force dut.base_valid = valid;
        force dut.base_touch = touch;
        force dut.base_touch_set = touch_set;
        force dut.base_touch_way = touch_way;
        force dut.base_replacement_set = replacement_set;
        passed = 0;
        failed = 0;
        @(negedge clk);
        reset_cache();

        // Hits and stores are not load-miss events and never enqueue reads.
        ren = 1'b1;
        step();
        #1;
        check("load hit does not prefetch", !mem_rd);
        ren = 1'b0;
        wen = 1'b1;
        store = 1'b1;
        #1;
        check("store miss passes through without a read", mem_wr && !mem_rd);
        step();
        wen = 1'b0;
        store = 1'b0;

        if (PREFETCH_EN == 0) begin
            demand(A);
            #1;
            check("prefetch disabled emits only a demand", !mem_rd && !dut.prefetch_pending);
            step();
            respond(1'b0, A);
        end else begin
            pair(A + BLOCK_SIZE-4);
            for (j = 0; j < 9; j = j + 1) begin
                #1;
                check("arbitrary latency waits for valid", !dut.request_pop && dut.request_count == 2);
                step();
            end
            respond(1'b0, A);
            #1;
            check("CPU resumes with prefetch still outstanding", ready && dut.request_count == 1);
            ren = 1'b1;
            cpu_addr = A + BLOCK_SIZE;
            mem_valid = 1'b1;
            #1;
            check("prefetch fill returns complete line and chooses invalid way",
                  dut.prefetch_fill && dut.prefetch_fill_addr == A+BLOCK_SIZE &&
                  dut.prefetch_fill_data == mem_data && dut.replacement_way == 0);
            check("same-set read stalls during prefetch placement", !ready && dut.prefetch_cpu_stall);
            step();
            mem_valid = 1'b0;
            #1;
            check("CPU retries after exactly one fill cycle", ready && !dut.prefetch_cpu_stall);
            ren = 1'b0;

            reset_cache();
            demand(A);
            cached = 1'b1;
            #1;
            check("cached next line is skipped", !mem_rd && dut.prefetch_lookup_addr == A+BLOCK_SIZE);
            step();
            cached = 1'b0;
            respond(1'b0, A);
            #1;
            check("skip leaves no phantom outstanding request", dut.request_count == 0);

            reset_cache();
            pair(A);
            load_miss = 1'b1;
            miss_addr = A+BLOCK_SIZE;
            #1;
            check("demand for outstanding prefetch is not duplicated", !mem_rd && !dut.demand_accept);
            miss_addr = A;
            #1;
            check("held demand miss is not issued twice", !mem_rd && !dut.demand_accept);
            load_miss = 1'b0;
            demand(A-BLOCK_SIZE);
            #1;
            check("next line already in flight is skipped", !mem_rd);
            step();
            respond(1'b0, A);
            respond(1'b1, A+BLOCK_SIZE);
            respond(1'b0, A-BLOCK_SIZE);

            reset_cache();
            pair(A);
            respond(1'b0, A);
            pair(B);
            respond(1'b1, A+BLOCK_SIZE);
            respond(1'b0, B);
            respond(1'b1, B+BLOCK_SIZE);
            #1;
            check("overlapping demand/prefetch replies drain completely", dut.request_count == 0);

            // Fill the tracker, then pop and push across the pointer wrap.
            reset_cache();
            pair(A);
            pair(B);
            load_miss = 1'b1;
            miss_addr = C;
            #1;
            check("full request queue applies backpressure", !mem_rd && dut.request_count == 4);
            respond(1'b0, A);
            #1;
            check("prefetch credit remains reserved until two slots free", !mem_rd);
            mem_valid = 1'b1;
            #1;
            check("simultaneous pop and demand push reuse available credit",
                  dut.prefetch_return && dut.demand_accept && mem_rd_addr == C);
            step();
            mem_valid = 1'b0;
            load_miss = 1'b0;
            #1;
            check("reserved prefetch issues even as queue fills", mem_rd && mem_rd_addr == C+BLOCK_SIZE);
            step();
            respond(1'b0, B);
            respond(1'b1, B+BLOCK_SIZE);
            respond(1'b0, C);
            respond(1'b1, C+BLOCK_SIZE);
            #1;
            check("wrapped queue drains without loss", dut.request_count == 0);

            reset_cache();
            pair(A);
            respond(1'b0, A);
            victim_dirty = 1'b1;
            wen = 1'b1;
            store = 1'b1;
            cpu_addr = A; // Different set from prefetch unless fully associative.
            mem_valid = 1'b1;
            #1;
            check("dirty prefetch victim gets full-line write-back priority",
                  mem_wr && mem_wr_addr == victim_addr && mem_wr_data == victim_data &&
                  mem_wr_be == {BLOCK_SIZE{1'b1}} && !ready);
            step();
            mem_valid = 1'b0;
            #1;
            check("CPU store can retry after dirty eviction",
                  ready && mem_wr && mem_wr_addr == 32'hb001_0000);

            reset_cache();
            pair(A);
            respond(1'b0, A);
            fill_cached = 1'b1;
            victim_dirty = 1'b1;
            ren = 1'b1;
            cpu_addr = A+BLOCK_SIZE;
            mem_valid = 1'b1;
            #1;
            check("already-cached return preserves newer dirty data",
                  dut.prefetch_return && !dut.prefetch_fill && !mem_wr && ready);
            step();
            mem_valid = 1'b0;

            // Change recency while the prefetch is outstanding. Its victim
            // must reflect the new state when it returns, not at request time.
            reset_cache();
            valid = {NUM_SETS*NUM_WAYS{1'b1}};
            pair(A);
            respond(1'b0, A);
            touch = 1'b1;
            touch_set = (NUM_SETS > 1) ? 1 : 0;
            touch_way = 0;
            step();
            touch = 1'b0;
            ren = 1'b1;
            cpu_addr = A;
            touch = 1'b1;
            touch_set = 0;
            mem_valid = 1'b1;
            #1;
            sampled_victim = dut.replacement_way;
            check("prefetch victim reflects intervening cache hits",
                  (NUM_WAYS == 1) || sampled_victim != 0);
            check("clean fill permits CPU hits in a different set",
                  ready == (NUM_SETS > 1));
            step();
            mem_valid = 1'b0;
            touch = 1'b0;
            replacement_set = (NUM_SETS > 1) ? 1 : 0;
            #1;
            check("prefetch fill becomes most recently used",
                  (NUM_WAYS == 1) || dut.replacement_way != sampled_victim);
            if (NUM_SETS > 1) begin
                replacement_set = 0;
                #1;
                check("concurrent CPU hit updates its independent PLRU set",
                      (NUM_WAYS == 1) || dut.replacement_way != 0);
            end

            // Carry into the next tag, including the 32-bit address wrap.
            reset_cache();
            pair(32'hffff_ffff);
            respond(1'b0, 32'hffff_ffff & ~(BLOCK_SIZE-1));
            respond(1'b1, 32'b0);

            // Minimum one-cycle memory latency: response and next issue coincide.
            reset_cache();
            demand(A);
            mem_valid = 1'b1;
            #1;
            check("demand reply overlaps next-cycle prefetch issue",
                  dut.demand_return && mem_rd && mem_rd_addr == A+BLOCK_SIZE);
            step();
            respond(1'b1, A+BLOCK_SIZE);

            // Reset with a scheduled prefetch and an unanswered demand.
            reset_cache();
            demand(A);
            rst = 1'b1;
            #1;
            check("reset immediately suppresses pending traffic", !mem_rd && !mem_wr && !ready);
            step();
            rst = 1'b0;
            mem_valid = 1'b1;
            #1;
            check("empty tracker ignores an unowned response", !dut.request_pop);
            mem_valid = 1'b0;
        end

        // PLRU state is independent across sets and changes on committed hits.
        reset_cache();
        valid = {NUM_SETS*NUM_WAYS{1'b1}};
        touch = 1'b1;
        touch_way = 0;
        touch_set = 0;
        step();
        touch = 1'b0;
        #1;
        check("cache instantiates and updates PLRU", dut.replacement_way < NUM_WAYS &&
              ((NUM_WAYS == 1) || dut.replacement_way != 0));
        if (NUM_SETS > 1) begin
            replacement_set = 1;
            #1;
            check("touching one set preserves another set's PLRU", dut.replacement_way == 0);
        end

        $display("Prefetch=%0d ways=%0d sets=%0d block=%0d", PREFETCH_EN, NUM_WAYS, NUM_SETS, BLOCK_SIZE);
        $display("%0d passed, %0d failed", passed, failed);
        if (failed == 0)
            $display("ALL TESTS PASSED");
        else
            $display("TEST FAILED");
        $finish;
    end

    initial begin
        #100000;
        $display("TEST FAILED: timeout");
        $finish;
    end
endmodule

`default_nettype wire
