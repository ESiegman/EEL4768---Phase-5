`timescale 1ns / 1ps
`default_nettype none

`include "mem_model.vh"

// Unit testbench for cache.v on its own, against the main memory model in
// mem_model.vh.
//
// The cache is a black box: everything is judged at its two ports, never
// from signals inside it. The CPU side is driven the way hart.v drives it: a
// request (i_ren or i_wen, with i_addr, i_mask, i_wdata) is held unchanged
// until o_ready is high, o_rdata is sampled in that cycle, and the next
// request (if any) follows in the very next cycle. A hit is an access that
// sends no o_mem_rd. Misses, write-backs (every byte enabled) and
// write-throughs (only the stored bytes) are read off the memory side.
//
// The testbench keeps its own model of the cache, written from phase_5.pdf
// and not from cache.v: which line sits in which way of which set, which are
// dirty, and the LRU order or PLRU tree bits of every set. Most tests are
// access sequences checked one access at a time against it: after each
// access the testbench waits for memory to go quiet and then requires
// exactly the reads, write-backs and write-throughs the model predicts, in
// order, with the right addresses, byte enables and data, plus the right
// load data. A flat reference memory holds what every address should read.
// Some tests add literal checks on top (the PDF's LRU and PLRU examples,
// prefetch timing) so a failure names the rule that broke.
//
// Rules the model follows where the PDF leaves room:
//  * Victim: the lowest-numbered invalid way, else LRU or the PLRU tree
//    (table in 4.3). Figure 1's own fill sequence puts B in way 2, following
//    the tree into an invalid way; the table wins here.
//  * PLRU (4.3.2, Figure 1): node n of a set has children 2n and 2n+1, bit 0
//    points to the lower-numbered half. A hit or fill makes every node on the
//    way's path point away from it; a miss walks the bits without changing
//    them. Store hits are uses, for LRU too.
//  * Prefetch (4.8): decided when the load misses, skipped if the next line
//    is cached at that moment. The prefetched line's victim is chosen when it
//    arrives, after the demand line was stored. It is stored clean, as MRU.
//  * Prefetch collisions (4.8.1) are only checked for correct data, since
//    either option is allowed.
//
// cache_harness runs every test group on one configuration and memory
// latency. cache_tb, at the bottom, instantiates one per row of its table
// and runs them one after another.
module cache_harness #(
    // Printed in front of every message from this instance.
    parameter LABEL        = "base, L=4",
    parameter EVICT_POLICY = 0,
    parameter NUM_WAYS     = 128,
    parameter CACHE_SIZE   = 16,
    parameter BLOCK_SIZE   = 128,
    parameter PREFETCH_EN  = 0,
    parameter LATENCY      = 4,
    parameter SEED         = 1,
    // Random accesses in the stress group.
    parameter STRESS_OPS   = 2000,
    // Give up on this instance after this many cycles.
    parameter MAX_CYCLES   = 500000
) (
    input  wire i_start,
    output reg  o_done
);

    localparam [31:0] BASE        = 32'h10010000;
    localparam        BW          = BLOCK_SIZE * 8;
    localparam        LINES       = CACHE_SIZE * 1024 / BLOCK_SIZE;
    localparam        SETS        = LINES / NUM_WAYS;
    localparam        INDEX_BITS  = $clog2(SETS);
    localparam        LINE_WORDS  = BLOCK_SIZE / 4;
    localparam        CACHE_BYTES = CACHE_SIZE * 1024;
    // Bytes between consecutive lines of one set: line_addr(s, k) is the
    // k-th line that maps to set s.
    localparam        WSTRIDE     = SETS * BLOCK_SIZE;
    // Memory map. Tests use [BASE, FLUSH_BASE), eight times the cache. The
    // flush region after it, twice the cache, is only ever read, to push
    // every dirty line out of the cache. One spare block follows, so a
    // prefetch past the end is still a legal read.
    localparam        TEST_BYTES   = 8 * CACHE_BYTES;
    localparam        STRESS_BYTES = 4 * CACHE_BYTES;
    localparam [31:0] FLUSH_BASE   = BASE + TEST_BYTES;
    localparam        FLUSH_BYTES  = 2 * CACHE_BYTES;
    localparam        MEM_WORDS    = (TEST_BYTES + FLUSH_BYTES + BLOCK_SIZE) / 4;
    localparam        POW2_WAYS    = (NUM_WAYS & (NUM_WAYS - 1)) == 0;
    localparam        LOGN         = 64;
    // Cycles one access may take before the cache counts as hung.
    localparam        ACC_TIMEOUT  = 40 * LATENCY + 200;
    localparam        MAX_MSGS     = 40;

    integer passed;
    integer failed;

    reg [8*48-1:0]  group;    // test group in progress, for messages
    reg [8*160-1:0] msg;
    integer         msgs;     // failures printed so far
    integer         seed;

    // ------------------------------------------------------------------
    // Clock (runs only while this instance is active), DUT, memory
    // ------------------------------------------------------------------
    reg clk;
    reg rst;
    reg running;

    always begin
        wait (running);
        #5 clk = ~clk;
    end

    reg         c_ren;
    reg         c_wen;
    reg  [31:0] c_addr;
    reg  [ 3:0] c_mask;
    reg  [31:0] c_wdata;
    wire        ready;
    wire [31:0] rdata;

    wire                  mem_rd;
    wire [31:0]           mem_rd_addr;
    wire                  mem_valid;
    wire [BW-1:0]         mem_rd_data;
    wire                  mem_wr;
    wire [31:0]           mem_wr_addr;
    wire [BW-1:0]         mem_wr_data;
    wire [BLOCK_SIZE-1:0] mem_wr_be;

    cache #(
        .EVICT_POLICY(EVICT_POLICY),
        .NUM_WAYS    (NUM_WAYS),
        .CACHE_SIZE  (CACHE_SIZE),
        .BLOCK_SIZE  (BLOCK_SIZE),
        .PREFETCH_EN (PREFETCH_EN)
    ) dut (
        .i_clk        (clk),
        .i_rst        (rst),
        .i_ren        (c_ren),
        .i_wen        (c_wen),
        .i_addr       (c_addr),
        .i_mask       (c_mask),
        .i_wdata      (c_wdata),
        .o_ready      (ready),
        .o_rdata      (rdata),
        .o_mem_rd     (mem_rd),
        .o_mem_rd_addr(mem_rd_addr),
        .i_mem_valid  (mem_valid),
        .i_mem_rd_data(mem_rd_data),
        .o_mem_wr     (mem_wr),
        .o_mem_wr_addr(mem_wr_addr),
        .o_mem_wr_data(mem_wr_data),
        .o_mem_wr_be  (mem_wr_be)
    );

    main_mem #(
        .BLOCK_SIZE(BLOCK_SIZE),
        .LATENCY   (LATENCY),
        .BASE_ADDR (BASE),
        .SIZE_WORDS(MEM_WORDS),
        .NAME      ({LABEL, ", memory"})
    ) u_mem (
        .i_clk     (clk),
        .i_rst     (rst),
        .i_rd      (mem_rd),
        .i_rd_addr (mem_rd_addr),
        .o_rd_valid(mem_valid),
        .o_rd_data (mem_rd_data),
        .i_wr      (mem_wr),
        .i_wr_addr (mem_wr_addr),
        .i_wr_data (mem_wr_data),
        .i_wr_be   (mem_wr_be)
    );

    // ------------------------------------------------------------------
    // Memory-side monitor: logs every read request, response and write
    // ------------------------------------------------------------------
    integer cyc;          // clock edges since this instance started
    integer rd_n;         // reads sent so far
    integer val_n;        // responses seen so far
    integer wr_n;         // writes sent so far
    integer last_event;   // cycle of the last read, response or write
    reg     hung;         // the cache stopped answering: skip the rest

    reg [31:0]           rd_log      [0:LOGN-1];
    integer              rd_cyc      [0:LOGN-1];
    integer              val_cyc     [0:LOGN-1];
    reg [31:0]           wr_log_addr [0:LOGN-1];
    reg [BLOCK_SIZE-1:0] wr_log_be   [0:LOGN-1];
    reg [BW-1:0]         wr_log_data [0:LOGN-1];

    always @(posedge clk)
        cyc = cyc + 1;

    // Sampled mid-cycle, while every output is stable.
    always @(negedge clk) begin
        if (running) begin
            if (mem_rd === 1'b1) begin
                rd_log[rd_n % LOGN] = mem_rd_addr;
                rd_cyc[rd_n % LOGN] = cyc;
                rd_n       = rd_n + 1;
                last_event = cyc;
            end
            if (mem_valid === 1'b1) begin
                val_cyc[val_n % LOGN] = cyc;
                val_n      = val_n + 1;
                last_event = cyc;
            end
            if (mem_wr === 1'b1) begin
                wr_log_addr[wr_n % LOGN] = mem_wr_addr;
                wr_log_be[wr_n % LOGN]   = mem_wr_be;
                wr_log_data[wr_n % LOGN] = mem_wr_data;
                wr_n       = wr_n + 1;
                last_event = cyc;
            end
            if (cyc > MAX_CYCLES && !hung) begin
                hung   = 1'b1;
                failed = failed + 1;
                $display("[FAIL] %0s: %0s: MAX_CYCLES (%0d) reached; remaining tests skipped",
                         LABEL, group, MAX_CYCLES);
            end
        end
    end

    // ------------------------------------------------------------------
    // Reporting
    // ------------------------------------------------------------------
    task begin_group;
        input [8*48-1:0] name;
        begin
            group = name;
            if (!hung)
                $display("--- %0s: %0s ---", LABEL, name);
        end
    endtask

    // Checks after a hang are skipped, not counted.
    task check;
        input [8*160-1:0] what;
        input             ok;
        begin
            if (!hung) begin
                if (ok) begin
                    passed = passed + 1;
                end else begin
                    failed = failed + 1;
                    if (msgs < MAX_MSGS)
                        $display("[FAIL] %0s: %0s: %0s", LABEL, group, what);
                    else if (msgs == MAX_MSGS)
                        $display("[FAIL] %0s: further failures not shown", LABEL);
                    msgs = msgs + 1;
                end
            end
        end
    endtask

    // ------------------------------------------------------------------
    // Addresses and the flat reference memory
    // ------------------------------------------------------------------
    reg [31:0] ref_mem [0:MEM_WORDS-1];

    function integer set_of;
        input [31:0] a;
        begin
            set_of = ((a - BASE) / BLOCK_SIZE) % SETS;
        end
    endfunction

    function [31:0] line_of;
        input [31:0] a;
        begin
            line_of = a - (a % BLOCK_SIZE);
        end
    endfunction

    // The k-th line (k = 0, 1, ...) that maps to set s.
    function [31:0] line_addr;
        input integer s;
        input integer k;
        begin
            line_addr = BASE + k * WSTRIDE + s * BLOCK_SIZE;
        end
    endfunction

    function [BW-1:0] ref_line;
        input [31:0] line;
        integer w;
        begin
            for (w = 0; w < LINE_WORDS; w = w + 1)
                ref_line[32*w +: 32] = ref_mem[(line - BASE) / 4 + w];
        end
    endfunction

    // Masks by number: bytes 0-3, halves 0 and 2, word.
    function [3:0] mask_of;
        input integer k;
        begin
            case (k)
                0:       mask_of = 4'b0001;
                1:       mask_of = 4'b0010;
                2:       mask_of = 4'b0100;
                3:       mask_of = 4'b1000;
                4:       mask_of = 4'b0011;
                5:       mask_of = 4'b1100;
                default: mask_of = 4'b1111;
            endcase
        end
    endfunction

    // Initial memory contents: different in every word, so a hit on the
    // wrong line returns visibly wrong data.
    function [31:0] init_word;
        input integer i;
        begin
            init_word = (i * 32'h9E3779B1) ^ 32'hA5A5_0000 ^ SEED;
        end
    endfunction

    // ------------------------------------------------------------------
    // Reference model of the cache
    // ------------------------------------------------------------------
    // Way w of set s is entry s * NUM_WAYS + w. PLRU node n (1..NUM_WAYS-1)
    // of set s is m_tree[s * NUM_WAYS + n].
    reg        m_valid [0:LINES-1];
    reg        m_dirty [0:LINES-1];
    reg [31:0] m_line  [0:LINES-1];
    integer    m_age   [0:LINES-1];
    reg        m_tree  [0:LINES-1];
    integer    m_tick;

    // Traffic the model expects from the access in progress.
    integer              x_nrd;
    integer              x_nwr;
    reg [31:0]           x_rd      [0:3];
    reg [31:0]           x_wr_addr [0:3];
    reg [BLOCK_SIZE-1:0] x_wr_be   [0:3];
    reg [BW-1:0]         x_wr_data [0:3];
    integer              m_last_victim;   // way chosen by the last m_fill

    task model_reset;
        integer i;
        begin
            for (i = 0; i < LINES; i = i + 1) begin
                m_valid[i] = 1'b0;
                m_dirty[i] = 1'b0;
                m_line[i]  = 32'd0;
                m_age[i]   = 0;
                m_tree[i]  = 1'b0;
            end
            m_tick = 0;
        end
    endtask

    function integer m_lookup;
        input [31:0] line;
        integer s;
        integer w;
        begin
            m_lookup = -1;
            s = set_of(line);
            for (w = 0; w < NUM_WAYS; w = w + 1)
                if (m_valid[s * NUM_WAYS + w] && m_line[s * NUM_WAYS + w] == line)
                    m_lookup = w;
        end
    endfunction

    function integer m_victim;
        input integer s;
        integer w;
        integer n;
        begin
            m_victim = -1;
            for (w = NUM_WAYS - 1; w >= 0; w = w - 1)
                if (!m_valid[s * NUM_WAYS + w])
                    m_victim = w;
            if (m_victim < 0) begin
                if (EVICT_POLICY == 0) begin
                    m_victim = 0;
                    for (w = 1; w < NUM_WAYS; w = w + 1)
                        if (m_age[s * NUM_WAYS + w] < m_age[s * NUM_WAYS + m_victim])
                            m_victim = w;
                end else begin
                    n = 1;
                    while (n < NUM_WAYS)
                        n = 2 * n + m_tree[s * NUM_WAYS + n];
                    m_victim = n - NUM_WAYS;
                end
            end
        end
    endfunction

    // A hit or fill of way w in set s.
    task m_touch;
        input integer s;
        input integer w;
        integer n;
        integer lo;
        integer hi;
        begin
            m_tick = m_tick + 1;
            m_age[s * NUM_WAYS + w] = m_tick;
            n  = 1;
            lo = 0;
            hi = NUM_WAYS;
            while (n < NUM_WAYS) begin
                if (w < (lo + hi) / 2) begin
                    m_tree[s * NUM_WAYS + n] = 1'b1;
                    hi = (lo + hi) / 2;
                    n  = 2 * n;
                end else begin
                    m_tree[s * NUM_WAYS + n] = 1'b0;
                    lo = (lo + hi) / 2;
                    n  = 2 * n + 1;
                end
            end
        end
    endtask

    // Stores a line arriving from memory, expecting a write-back if the
    // victim is dirty.
    task m_fill;
        input [31:0] line;
        integer s;
        integer v;
        begin
            s = set_of(line);
            v = m_victim(s);
            m_last_victim = v;
            if (m_valid[s * NUM_WAYS + v] && m_dirty[s * NUM_WAYS + v]) begin
                x_wr_addr[x_nwr] = m_line[s * NUM_WAYS + v];
                x_wr_be[x_nwr]   = {BLOCK_SIZE{1'b1}};
                x_wr_data[x_nwr] = ref_line(m_line[s * NUM_WAYS + v]);
                x_nwr = x_nwr + 1;
            end
            m_valid[s * NUM_WAYS + v] = 1'b1;
            m_dirty[s * NUM_WAYS + v] = 1'b0;
            m_line[s * NUM_WAYS + v]  = line;
            m_touch(s, v);
        end
    endtask

    // Runs one access through the model: fills x_* with the traffic it
    // should cause and updates the model and, for a store, ref_mem.
    task model_access;
        input        is_wr;
        input [31:0] addr;
        input [ 3:0] mask;
        input [31:0] wdata;
        reg   [31:0] line;
        reg   [31:0] nxt;
        reg          pf;
        integer      s;
        integer      w;
        integer      wo;
        integer      b;
        begin
            x_nrd = 0;
            x_nwr = 0;
            line  = line_of(addr);
            s     = set_of(addr);
            wo    = (addr % BLOCK_SIZE) / 4;
            w     = m_lookup(line);
            if (w >= 0) begin
                m_touch(s, w);
                if (is_wr)
                    m_dirty[s * NUM_WAYS + w] = 1'b1;
            end else if (is_wr) begin
                // Write-through, no allocate, never a prefetch.
                x_wr_addr[0] = line;
                x_wr_be[0]   = {BLOCK_SIZE{1'b0}};
                x_wr_be[0][4*wo +: 4] = mask;
                x_wr_data[0] = {BW{1'b0}};
                x_wr_data[0][32*wo +: 32] = wdata;
                x_nwr = 1;
            end else begin
                nxt = line + BLOCK_SIZE;
                pf  = PREFETCH_EN && m_lookup(nxt) < 0;
                x_rd[0] = line;
                x_nrd   = 1;
                if (pf) begin
                    x_rd[1] = nxt;
                    x_nrd   = 2;
                end
                m_fill(line);
                if (pf)
                    m_fill(nxt);
            end
            if (is_wr)
                for (b = 0; b < 4; b = b + 1)
                    if (mask[b])
                        ref_mem[(addr - BASE) / 4][8*b +: 8] = wdata[8*b +: 8];
        end
    endtask

    // ------------------------------------------------------------------
    // CPU-side driver
    // ------------------------------------------------------------------
    // Every task below starts and ends just after a rising edge.
    integer    acc_start;      // cycle the last access was presented
    integer    acc_ready;      // cycle its o_ready was high
    reg [31:0] acc_rdata;
    integer    rd_mark;
    integer    val_mark;
    integer    wr_mark;
    integer    n_acc;          // accesses made
    integer    n_acc_hold;     // extra cycles a request is held after o_ready

    function [8*48-1:0] op_name;
        input        is_wr;
        input [31:0] addr;
        input [ 3:0] mask;
        reg   [8*48-1:0] t;
        begin
            $sformat(t, "%0s %h mask %b", is_wr ? "store" : "load", addr, mask);
            op_name = t;
        end
    endfunction

    // Presents one request and holds it until o_ready (then n_acc_hold more
    // cycles, as hart.v does when the other cache stalls the pipeline).
    task access;
        input        is_wr;
        input [31:0] addr;
        input [ 3:0] mask;
        input [31:0] wdata;
        integer      waited;
        integer      h;
        begin
            if (!hung) begin
                n_acc     = n_acc + 1;
                c_ren     = !is_wr;
                c_wen     = is_wr;
                c_addr    = addr;
                c_mask    = mask;
                c_wdata   = wdata;
                acc_start = cyc;
                for (h = 0; h <= n_acc_hold && !hung; h = h + 1) begin
                    waited = 0;
                    @(negedge clk);
                    while (ready !== 1'b1 && waited < ACC_TIMEOUT && !hung) begin
                        @(negedge clk);
                        waited = waited + 1;
                    end
                    if (waited >= ACC_TIMEOUT) begin
                        hung   = 1'b1;
                        failed = failed + 1;
                        $display("[FAIL] %0s: %0s: %0s got no o_ready within %0d cycles (cycle %0d); remaining tests skipped",
                                 LABEL, group, op_name(is_wr, addr, mask), ACC_TIMEOUT, cyc);
                    end
                    if (h == 0) begin
                        acc_ready = cyc;
                        acc_rdata = rdata;
                    end else if (!is_wr && !hung) begin
                        check("load held after o_ready returns the same word",
                              rdata === acc_rdata);
                    end
                    @(posedge clk);
                    #1;
                end
                c_ren  = 1'b0;
                c_wen  = 1'b0;
                c_mask = 4'b0000;
            end
        end
    endtask

    // Waits until no read is outstanding and memory has been idle for a
    // few cycles.
    task wait_quiet;
        integer waited;
        begin
            waited = 0;
            while (!hung && (rd_n != val_n || cyc - last_event < 3)) begin
                @(posedge clk);
                #1;
                waited = waited + 1;
                if (waited > ACC_TIMEOUT) begin
                    hung   = 1'b1;
                    failed = failed + 1;
                    $display("[FAIL] %0s: %0s: memory traffic never stopped (%0d reads sent, %0d answered, cycle %0d); remaining tests skipped",
                             LABEL, group, rd_n, val_n, cyc);
                end
            end
        end
    endtask

    task mark;
        begin
            rd_mark  = rd_n;
            val_mark = val_n;
            wr_mark  = wr_n;
        end
    endtask

    task show_traffic;
        integer j;
        begin
            $display("         expected %0d read(s), %0d write(s):", x_nrd, x_nwr);
            for (j = 0; j < x_nrd; j = j + 1)
                $display("           read  %h", x_rd[j]);
            for (j = 0; j < x_nwr; j = j + 1)
                $display("           write %h %0s", x_wr_addr[j],
                         &x_wr_be[j] ? "(all bytes)" : "(partial)");
            $display("         saw %0d read(s), %0d write(s):", rd_n - rd_mark, wr_n - wr_mark);
            for (j = rd_mark; j < rd_n && j < rd_mark + 4; j = j + 1)
                $display("           read  %h", rd_log[j % LOGN]);
            for (j = wr_mark; j < wr_n && j < wr_mark + 4; j = j + 1)
                $display("           write %h %0s", wr_log_addr[j % LOGN],
                         &wr_log_be[j % LOGN] ? "(all bytes)" : "(partial)");
        end
    endtask

    // Does the traffic since mark match x_*? Write data is compared only in
    // enabled bytes.
    function traffic_ok;
        input dummy;
        integer j;
        integer b;
        begin
            traffic_ok = (rd_n - rd_mark == x_nrd) && (wr_n - wr_mark == x_nwr);
            for (j = 0; traffic_ok && j < x_nrd; j = j + 1)
                if (rd_log[(rd_mark + j) % LOGN] !== x_rd[j])
                    traffic_ok = 1'b0;
            for (j = 0; traffic_ok && j < x_nwr; j = j + 1) begin
                if (wr_log_addr[(wr_mark + j) % LOGN] !== x_wr_addr[j] ||
                    wr_log_be[(wr_mark + j) % LOGN] !== x_wr_be[j])
                    traffic_ok = 1'b0;
                for (b = 0; b < BLOCK_SIZE; b = b + 1)
                    if (x_wr_be[j][b] &&
                        wr_log_data[(wr_mark + j) % LOGN][8*b +: 8] !== x_wr_data[j][8*b +: 8])
                        traffic_ok = 1'b0;
            end
        end
    endfunction

    // One access checked against the model: load data, and exactly the
    // memory traffic the model expects, once memory has gone quiet.
    task macc;
        input        is_wr;
        input [31:0] addr;
        input [ 3:0] mask;
        input [31:0] wdata;
        reg   [31:0] want;
        reg          ok;
        begin
            if (!hung) begin
                model_access(is_wr, addr, mask, wdata);
                want = ref_mem[(addr - BASE) / 4];
                mark;
                access(is_wr, addr, mask, wdata);
                wait_quiet;
                if (!hung) begin
                    ok = traffic_ok(1'b0);
                    $sformat(msg, "%0s: memory traffic differs from the model", op_name(is_wr, addr, mask));
                    check(msg, ok);
                    if (!ok && msgs <= MAX_MSGS)
                        show_traffic;
                    if (!is_wr) begin
                        $sformat(msg, "%0s: o_rdata %h, expected %h", op_name(is_wr, addr, mask),
                                 acc_rdata, want);
                        check(msg, acc_rdata === want);
                    end
                end
            end
        end
    endtask

    task mload;
        input [31:0] addr;
        begin
            macc(1'b0, addr, 4'b1111, 32'd0);
        end
    endtask

    task mstore;
        input [31:0] addr;
        input [ 3:0] mask;
        input [31:0] wdata;
        begin
            macc(1'b1, addr, mask, wdata);
        end
    endtask

    // Loads a line and stores to its first word, leaving it cached and dirty.
    task mfill_dirty;
        input [31:0] line;
        begin
            mload(line);
            mstore(line, 4'b1111, $random(seed));
        end
    endtask

    // One access checked only for load data (no model, no waiting), so
    // accesses run back to back as in the pipeline. The model goes stale;
    // call reset_all before the next model-checked group.
    task uacc;
        input        is_wr;
        input [31:0] addr;
        input [ 3:0] mask;
        input [31:0] wdata;
        reg   [31:0] want;
        integer      b;
        begin
            if (!hung) begin
                want = ref_mem[(addr - BASE) / 4];
                if (is_wr)
                    for (b = 0; b < 4; b = b + 1)
                        if (mask[b])
                            ref_mem[(addr - BASE) / 4][8*b +: 8] = wdata[8*b +: 8];
                access(is_wr, addr, mask, wdata);
                if (!is_wr && !hung) begin
                    $sformat(msg, "%0s: o_rdata %h, expected %h", op_name(is_wr, addr, mask),
                             acc_rdata, want);
                    check(msg, acc_rdata === want);
                end
            end
        end
    endtask

    // Resets the cache (memory is quiet first, so nothing arrives after
    // it). Dirty lines are dropped, so main memory becomes the reference.
    task reset_all;
        integer i;
        begin
            wait_quiet;
            rst = 1'b1;
            @(posedge clk);
            @(posedge clk);
            #1 rst = 1'b0;
            model_reset;
            for (i = 0; i < MEM_WORDS; i = i + 1)
                ref_mem[i] = u_mem.mem[i];
        end
    endtask

    // Reads one word from every line of the flush region (twice the cache,
    // so every set sees at least 2 * NUM_WAYS fresh lines), which evicts
    // every line the tests left in the cache. Then main memory must equal
    // the reference everywhere.
    task flush_and_compare;
        input [8*64-1:0] what;
        integer i;
        integer n_bad;
        begin
            if (!hung) begin
                for (i = 0; i < FLUSH_BYTES; i = i + BLOCK_SIZE)
                    uacc(1'b0, FLUSH_BASE + i, 4'b1111, 32'd0);
                wait_quiet;
                n_bad = 0;
                for (i = 0; i < MEM_WORDS; i = i + 1) begin
                    if (u_mem.mem[i] !== ref_mem[i]) begin
                        if (n_bad < 5 && !hung)
                            $display("[FAIL] %0s: %0s: after flushing, memory at %h is %h, expected %h",
                                     LABEL, group, BASE + 4 * i, u_mem.mem[i], ref_mem[i]);
                        n_bad = n_bad + 1;
                    end
                end
                $sformat(msg, "%0s: main memory after flushing the cache (%0d words differ)", what, n_bad);
                check(msg, n_bad == 0);
            end
        end
    endtask

    // ------------------------------------------------------------------
    // Test groups
    // ------------------------------------------------------------------
    integer    i;
    integer    j;
    integer    k;
    integer    s;
    integer    q;
    integer    n;
    integer    wr0;
    integer    rd0;
    reg [31:0] a;
    reg [31:0] l;
    reg [31:0] x;
    reg [31:0] p;
    reg        ok;

    task t_cold_miss;
        begin
            begin_group("1. cold load miss, then hits");
            reset_all;
            s = SETS / 2;
            a = line_addr(s, 1) + BLOCK_SIZE - 4;
            mload(a);
            check("the miss reads its own line first",
                  rd_n > rd_mark && rd_log[rd_mark % LOGN] === line_of(a));
            check("o_ready is high in the cycle the line arrives (PDF 4.7)",
                  val_n > val_mark && acc_ready == val_cyc[val_mark % LOGN]);
            mload(a);
            check("repeat load is a hit (no block read)", rd_n == rd_mark);
            mload(line_of(a));
            if (LINE_WORDS > 2)
                macc(1'b0, line_of(a) + 4 * (LINE_WORDS / 2), 4'b0011, 32'd0);
        end
    endtask

    task t_load_masks;
        begin
            begin_group("2. loads with every byte/half/word mask");
            reset_all;
            l = line_addr(0, 2);
            for (i = 0; i < LINE_WORDS; i = i + (LINE_WORDS > 1 ? LINE_WORDS - 1 : 1))
                for (k = 0; k < 7; k = k + 1)
                    macc(1'b0, l + 4 * i, mask_of(k), 32'd0);
        end
    endtask

    task t_store_hits;
        begin
            begin_group("3. store hits, every mask; write-back");
            reset_all;
            s = SETS > 1 ? 1 : 0;
            l = line_addr(s, 0);
            mload(l);
            wr0 = wr_n;
            for (i = 0; i < LINE_WORDS; i = i + (LINE_WORDS > 1 ? LINE_WORDS - 1 : 1)) begin
                for (k = 0; k < 7; k = k + 1) begin
                    mstore(l + 4 * i, mask_of(k), $random(seed));
                    mload(l + 4 * i);
                end
            end
            check("store hits write nothing to memory (write-back)", wr_n == wr0);
            // NUM_WAYS more lines in the set evict it.
            wr0 = wr_n;
            for (k = 1; k <= NUM_WAYS; k = k + 1)
                mload(line_addr(s, k));
            ok = (wr_n - wr0 == 1) && wr_log_addr[wr0 % LOGN] === l &&
                 &wr_log_be[wr0 % LOGN] && wr_log_data[wr0 % LOGN] === ref_line(l);
            check("evicting the dirty line writes it back once, every byte enabled, with all its data",
                  ok);
        end
    endtask

    task t_clean_evict;
        begin
            begin_group("4. clean eviction writes nothing");
            reset_all;
            s = SETS > 1 ? 2 % SETS : 0;
            wr0 = wr_n;
            for (k = 0; k <= NUM_WAYS + 1; k = k + 1)
                mload(line_addr(s, k));
            check("no write while evicting clean lines", wr_n == wr0);
        end
    endtask

    task t_store_miss;
        begin
            begin_group("5. store miss: write-through, no allocate");
            reset_all;
            s = SETS > 1 ? 3 % SETS : 0;
            n = SETS > 1 ? 1 : 2;    // fully associative: skip prefetched neighbours
            rd0 = rd_n;
            for (k = 0; k < 7; k = k + 1) begin
                a = line_addr(s, n * k) + 4 * ((3 * k) % LINE_WORDS);
                mstore(a, mask_of(k), $random(seed));
                ok = (wr_n - wr_mark == 1) && wr_log_addr[wr_mark % LOGN] === line_of(a) &&
                     wr_log_be[wr_mark % LOGN] === ({{BLOCK_SIZE{1'b0}}, mask_of(k)} << (4 * ((3 * k) % LINE_WORDS)));
                $sformat(msg, "store miss %h mask %b: one write, enabling exactly the stored bytes",
                         a, mask_of(k));
                check(msg, ok);
            end
            check("store misses send no block read (no allocate, no prefetch)", rd_n == rd0);
            for (k = 0; k < 7; k = k + 1) begin
                a = line_addr(s, n * k) + 4 * ((3 * k) % LINE_WORDS);
                mload(a);
                $sformat(msg, "load after store miss to %h misses (line was not allocated)", a);
                check(msg, rd_n > rd_mark && rd_log[rd_mark % LOGN] === line_of(a));
            end
        end
    endtask

    task t_address_split;
        begin
            begin_group("6. address split: offset, index, tag");
            reset_all;
            l = line_addr(SETS - 1, 3);
            rd0 = rd_n;
            for (i = 0; i < LINE_WORDS; i = i + 1)
                mload(l + 4 * i);
            check("one block read covers every word of the line",
                  rd_n - rd0 == (PREFETCH_EN ? 2 : 1));
            // Each index bit: two sets that differ only in that bit don't
            // conflict.
            for (j = 0; j < INDEX_BITS; j = j + 1) begin
                reset_all;
                for (k = 0; k < NUM_WAYS; k = k + 1) begin
                    mload(line_addr(0, k));
                    mload(line_addr(1 << j, k));
                end
                for (k = 0; k < NUM_WAYS; k = k + 1) begin
                    mload(line_addr(0, k));
                    mload(line_addr(1 << j, k));
                end
            end
            // Each tag bit the memory can reach: same set, tags that differ
            // in that bit are different lines.
            reset_all;
            s = SETS > 1 ? 1 : 0;
            a = line_addr(s, 0);
            mload(a);
            // (Fully associative with prefetch: a + BLOCK_SIZE was prefetched.)
            for (j = PREFETCH_EN && SETS == 1 ? 1 : 0; (WSTRIDE << j) < TEST_BYTES; j = j + 1) begin
                mload(a + (WSTRIDE << j));
                $sformat(msg, "line %h is not a hit on %h (tag bit %0d)",
                         a + (WSTRIDE << j), a, j);
                check(msg, rd_n > rd_mark && rd_log[rd_mark % LOGN] === a + (WSTRIDE << j));
            end
            mload(a);
            if (NUM_WAYS == 1) begin
                // Direct mapped: two lines one cache size apart thrash.
                reset_all;
                a = line_addr(s, 0);
                mfill_dirty(a);
                for (k = 0; k < 3; k = k + 1) begin
                    mload(a + CACHE_BYTES);
                    check("direct mapped: line one cache size away misses", rd_n > rd_mark);
                    mload(a);
                    check("direct mapped: and evicts the first one", rd_n > rd_mark);
                end
            end
        end
    endtask

    task t_invalid_first;
        begin
            begin_group("7. invalid ways are used before any eviction");
            reset_all;
            s = SETS > 1 ? 1 : 0;
            wr0 = wr_n;
            mfill_dirty(line_addr(s, 0));
            if (NUM_WAYS >= 3) begin
                // Make line 1 the least recently used valid line.
                mfill_dirty(line_addr(s, 1));
                mload(line_addr(s, 0));
            end
            for (k = NUM_WAYS >= 3 ? 2 : 1; k < NUM_WAYS; k = k + 1)
                mfill_dirty(line_addr(s, k));
            if (!(PREFETCH_EN && SETS == 1)) begin
                check("filling the set evicts nothing (no write-back)", wr_n == wr0);
                rd0 = rd_n;
                for (k = 0; k < NUM_WAYS; k = k + 1)
                    mload(line_addr(s, k));
                check("every line put in the set is still cached", rd_n == rd0);
            end
            mload(line_addr(s, NUM_WAYS));
        end
    endtask

    task t_lru;
        begin
            begin_group("8. LRU");
            reset_all;
            if (NUM_WAYS == 4) begin
                // PDF 4.3.1: lines A-D in ways 0-3, all dirty, so each eviction
                // shows up as a write-back naming its victim.
                s = SETS > 1 ? 1 : 0;
                for (k = 0; k < 4; k = k + 1)
                    mload(line_addr(s, k));
                for (k = 3; k >= 0; k = k - 1)
                    mstore(line_addr(s, k), 4'b1111, $random(seed));
                // Order (MRU..LRU) is now 0, 1, 2, 3.
                mload(line_addr(s, 2));           // 2, 0, 1, 3
                mload(line_addr(s, 3));           // 3, 2, 0, 1
                mload(line_addr(s, 4));           // miss E: evicts way 1
                check("PDF 4.3.1: after hits on ways 2 and 3, the miss evicts way 1 (B)",
                      wr_n - wr_mark == 1 && wr_log_addr[wr_mark % LOGN] === line_addr(s, 1));
                mload(line_addr(s, 5));           // miss F: evicts way 0
                check("PDF 4.3.1: the next victim is way 0 (A)",
                      wr_n - wr_mark == 1 && wr_log_addr[wr_mark % LOGN] === line_addr(s, 0));
                rd0 = rd_n;
                mload(line_addr(s, 2));
                mload(line_addr(s, 3));
                mload(line_addr(s, 4));
                mload(line_addr(s, 5));
                check("C, D, E and F are all still cached", rd_n == rd0);
                mload(line_addr(s, 0));
                check("A was evicted", rd_n > rd_mark);
                reset_all;
            end
            if (NUM_WAYS >= 2) begin
                // LRU, not FIFO: touching the oldest line saves it.
                s = SETS > 1 ? 2 % SETS : 0;
                for (k = 0; k < NUM_WAYS; k = k + 1)
                    mfill_dirty(line_addr(s, k));
                mload(line_addr(s, 0));
                mload(line_addr(s, NUM_WAYS));
                check("the miss evicts the least recently used line, not the oldest one",
                      wr_n - wr_mark >= 1 && wr_log_addr[wr_mark % LOGN] === line_addr(s, 1));
            end
        end
    endtask

    task t_plru;
        begin
            begin_group("9. PLRU (Figure 1 tree)");
            reset_all;
            // Fill ways 0..W-1 in order (each dirty, so a victim shows up as
            // a write-back), then hit way 0. The tree now leads to way W/2;
            // true LRU would pick way 1.
            s = SETS > 1 ? 1 : 0;
            for (k = 0; k < NUM_WAYS; k = k + 1)
                mfill_dirty(line_addr(s, k));
            mload(line_addr(s, 0));
            mload(line_addr(s, NUM_WAYS));
            $sformat(msg, "after filling ways 0-%0d and hitting way 0, the miss evicts way %0d (true LRU would evict way 1)",
                     NUM_WAYS - 1, NUM_WAYS / 2);
            check(msg, wr_n - wr_mark >= 1 &&
                       wr_log_addr[wr_mark % LOGN] === line_addr(s, NUM_WAYS / 2));
            // The new line went to way W/2. The tree now leads to the other
            // half, to way 1 (the sibling of way 0, which the hit pointed
            // away from) when NUM_WAYS = 4.
            mload(line_addr(s, NUM_WAYS + 1));
            if (NUM_WAYS == 4)
                check("4-way: the next miss evicts way 1",
                      wr_n - wr_mark >= 1 && wr_log_addr[wr_mark % LOGN] === line_addr(s, 1));
        end
    endtask

    // Random loads and stores over a few sets and more lines than fit, each
    // checked against the model: hit or miss, victim, write-back data.
    task t_random_model;
        integer nops;
        integer nsets;
        integer ntags;
        begin
            begin_group("10. random accesses vs the reference model");
            reset_all;
            nsets = SETS > 1 ? 2 : 1;
            ntags = NUM_WAYS + 3;
            nops  = 12 * NUM_WAYS + 200;
            if (nops > 1200)
                nops = 1200;
            for (i = 0; i < nops && !hung; i = i + 1) begin
                s = SETS / 2 + ($unsigned($random(seed)) % nsets);
                k = $unsigned($random(seed)) % ntags;
                j = $unsigned($random(seed)) % 7;
                a = line_addr(s % SETS, k) + 4 * ($unsigned($random(seed)) % LINE_WORDS);
                macc($unsigned($random(seed)) % 5 >= 3, a, mask_of(j), $random(seed));
            end
        end
    endtask

    task t_prefetch;
        begin
            begin_group("11. next-line prefetch");
            // a. A load miss on X reads X, then X + BLOCK_SIZE one cycle
            //    later; o_ready comes with X, not with the prefetch.
            reset_all;
            s = SETS / 2;
            x = line_addr(s, 2);
            mload(x + 4);
            ok = rd_n - rd_mark == 2 && rd_log[rd_mark % LOGN] === x &&
                 rd_log[(rd_mark + 1) % LOGN] === x + BLOCK_SIZE;
            check("a load miss on X reads X, then X + BLOCK_SIZE", ok);
            check("the prefetch read is sent the cycle after the demand read",
                  ok && rd_cyc[(rd_mark + 1) % LOGN] == rd_cyc[rd_mark % LOGN] + 1);
            check("o_ready comes when X arrives, before the prefetched line",
                  val_n - val_mark == 2 && acc_ready == val_cyc[val_mark % LOGN] &&
                  acc_ready < val_cyc[(val_mark + 1) % LOGN]);
            mload(x + BLOCK_SIZE);
            check("the prefetched line is a hit", rd_n == rd_mark);
            // b. No prefetch when the next line is already cached.
            mload(x + 4 * BLOCK_SIZE);           // caches x+4 and x+5 lines
            mload(x + 3 * BLOCK_SIZE);
            check("no prefetch when the next line is already cached",
                  rd_n - rd_mark == 1 && rd_log[rd_mark % LOGN] === x + 3 * BLOCK_SIZE);
            // c. No prefetch on a hit.
            mload(x + 3 * BLOCK_SIZE);
            check("no prefetch on a hit", rd_n == rd_mark);
            // d. No prefetch on a store miss.
            a = line_addr(s, 0);
            mstore(a + 4 * (LINE_WORDS - 1), 4'b1100, $random(seed));
            check("a store miss sends no read at all", rd_n == rd_mark);
            mload(a + BLOCK_SIZE);
            check("the line after a store miss was not prefetched",
                  rd_n > rd_mark && rd_log[rd_mark % LOGN] === a + BLOCK_SIZE);
            // e. The last set wraps to set 0 and carries into the tag.
            reset_all;
            x = line_addr(SETS - 1, 1);
            mload(x);
            check("prefetch from the last set: address + BLOCK_SIZE carries into the tag",
                  rd_n - rd_mark == 2 && rd_log[(rd_mark + 1) % LOGN] === x + BLOCK_SIZE);
            mload(x + BLOCK_SIZE);
            check("the wrapped prefetched line is a hit", rd_n == rd_mark);
            // f. The prefetched line's victim is chosen on arrival and written
            //    back if dirty; the line itself is clean and most recently used.
            if (SETS > 1) begin
                reset_all;
                q = SETS / 2;
                for (k = 0; k < NUM_WAYS; k = k + 1)
                    mfill_dirty(line_addr(q, k));
                x = line_addr(q - 1, NUM_WAYS);
                p = x + BLOCK_SIZE;              // = line_addr(q, NUM_WAYS)
                mload(x);
                if (EVICT_POLICY == 0)
                    check("the prefetched line evicts the set's LRU line and writes it back (dirty)",
                          wr_n - wr_mark == 1 && wr_log_addr[wr_mark % LOGN] === line_addr(q, 0) &&
                          &wr_log_be[wr_mark % LOGN]);
                wr0 = wr_n;
                for (k = 1; k <= NUM_WAYS + 1; k = k + 1) begin
                    mload(line_addr(q, NUM_WAYS + k));
                    if (EVICT_POLICY == 0 && k < NUM_WAYS)
                        check("the prefetched line is MRU: older dirty lines go first",
                              wr_n - wr_mark >= 1 && wr_log_addr[wr_mark % LOGN] === line_addr(q, k));
                end
                ok = 1'b1;
                for (j = wr0; j < wr_n; j = j + 1)
                    if (wr_log_addr[j % LOGN] === p)
                        ok = 1'b0;
                check("the prefetched line is clean: evicting it writes nothing", ok);
            end
        end
    endtask

    // A CPU access in the cycle the prefetched line arrives (PDF 4.8.1).
    // Only correctness is checked: either option is allowed. Each case runs
    // with the set clean and with it dirty, since a dirty victim also needs
    // the write port in that cycle.
    task t_collisions;
        integer    v;
        integer    d;
        reg [31:0] r;
        begin
            begin_group("12. prefetch collisions");
            for (d = 0; d < 2; d = d + 1) begin
                for (v = 0; v < 6 && !hung; v = v + 1) begin
                    if (v < 4 || NUM_WAYS > 1) begin
                        reset_all;
                        // Set q is full and its LRU line is Q. X's prefetch P
                        // lands in q and evicts Q in the cycle after X's
                        // o_ready, the very cycle the next access below is
                        // presented in.
                        q = SETS > 1 ? SETS / 2 : 0;
                        for (k = 0; k < NUM_WAYS; k = k + 1) begin
                            if (d)
                                mfill_dirty(line_addr(q, k));
                            else
                                mload(line_addr(q, k));
                        end
                        x = line_addr(SETS > 1 ? q - 1 : 0, NUM_WAYS);
                        p = x + BLOCK_SIZE;
                        a = line_addr(q, 0);               // Q, the LRU line
                        r = line_addr(q, NUM_WAYS - 1);    // R, stays
                        wait_quiet;
                        uacc(1'b0, x, 4'b1111, 32'd0);
                        case (v)
                            0: uacc(1'b0, p + 4, 4'b1111, 32'd0);
                            1: uacc(1'b1, p + 4, 4'b0011, $random(seed));
                            2: uacc(1'b0, a, 4'b1111, 32'd0);
                            3: uacc(1'b1, a, 4'b1111, $random(seed));
                            4: uacc(1'b0, r, 4'b1111, 32'd0);
                            default: uacc(1'b1, r + 4, 4'b1100, $random(seed));
                        endcase
                        uacc(1'b0, p + 4, 4'b1111, 32'd0);
                        uacc(1'b0, a, 4'b1111, 32'd0);
                        uacc(1'b0, r + 4, 4'b1111, 32'd0);
                        uacc(1'b0, x, 4'b1111, 32'd0);
                        wait_quiet;
                        case (v)
                            0: msg = "load of the arriving line";
                            1: msg = "store to the arriving line";
                            2: msg = "load of the line it evicts";
                            3: msg = "store to the line it evicts";
                            4: msg = "load of another line in the set";
                            default: msg = "store to another line in the set";
                        endcase
                        flush_and_compare({msg[8*40-1:0], d ? " (dirty set)" : " (clean set)"});
                    end
                end
            end
        end
    endtask

    task t_reset;
        begin
            begin_group("13. reset clears the valid bits");
            reset_all;
            a = line_addr(SETS > 1 ? 1 : 0, 1);
            mload(a);
            mstore(a, 4'b1111, 32'hDEADBEEF);
            wr0 = wr_n;
            reset_all;
            mload(a);
            check("a load after reset misses", rd_n > rd_mark && rd_log[rd_mark % LOGN] === a);
            check("reset writes nothing back (the dirty line is dropped)", wr_n == wr0);
        end
    endtask

    task t_held;
        begin
            begin_group("14. requests held after o_ready (as hart does)");
            reset_all;
            n_acc_hold = 2;
            s = SETS > 1 ? 1 : 0;
            a = line_addr(s, 0) + 4 * (LINE_WORDS - 1);
            rd0 = rd_n;
            uacc(1'b0, a, 4'b1111, 32'd0);
            wait_quiet;
            check("a held load miss reads its line once",
                  rd_n - rd0 == (PREFETCH_EN ? 2 : 1));
            wr0 = wr_n;
            uacc(1'b1, a, 4'b0010, 32'h0000A500);
            check("a held store hit writes nothing", wr_n == wr0);
            uacc(1'b1, line_addr(s, 3), 4'b1100, 32'h5A5A0000);
            uacc(1'b0, a, 4'b1111, 32'd0);
            n_acc_hold = 0;
            wait_quiet;
            flush_and_compare("held requests");
        end
    endtask

    // Back-to-back random accesses of every size, mostly near a moving hot
    // spot so there are hits as well as misses, over four times the cache.
    task t_stress;
        reg [31:0] hot;
        reg [31:0] prev;
        integer    r;
        begin
            begin_group("15. random stress, back to back");
            reset_all;
            hot  = BASE;
            prev = BASE;
            for (i = 0; i < STRESS_OPS && !hung; i = i + 1) begin
                r = $unsigned($random(seed)) % 100;
                if (r < 2)
                    hot = BASE + ($unsigned($random(seed)) % (STRESS_BYTES / BLOCK_SIZE)) * BLOCK_SIZE;
                if (r < 45)
                    a = hot + ($unsigned($random(seed)) % (CACHE_BYTES / 2));
                else if (r < 70)
                    a = prev + 4 + ($unsigned($random(seed)) % (2 * BLOCK_SIZE));
                else
                    a = BASE + ($unsigned($random(seed)) % STRESS_BYTES);
                if (a >= BASE + STRESS_BYTES)
                    a = a - STRESS_BYTES;
                a    = a - (a % 4);
                prev = a;
                uacc($unsigned($random(seed)) % 2, a, mask_of($unsigned($random(seed)) % 7),
                     $random(seed));
            end
            flush_and_compare("random stress");
        end
    endtask

    // ------------------------------------------------------------------
    // Run
    // ------------------------------------------------------------------
    initial begin
        clk        = 1'b0;
        rst        = 1'b1;
        running    = 1'b0;
        o_done     = 1'b0;
        c_ren      = 1'b0;
        c_wen      = 1'b0;
        c_addr     = 32'd0;
        c_mask     = 4'b0000;
        c_wdata    = 32'd0;
        passed     = 0;
        failed     = 0;
        msgs       = 0;
        cyc        = 0;
        rd_n       = 0;
        val_n      = 0;
        wr_n       = 0;
        last_event = 0;
        hung       = 1'b0;
        n_acc      = 0;
        n_acc_hold = 0;
        group      = "setup";
        seed       = SEED;
        model_reset;

        wait (i_start === 1'b1);
        for (i = 0; i < MEM_WORDS; i = i + 1)
            u_mem.mem[i] = init_word(i);
        $display("--- %0s: EVICT_POLICY=%0d NUM_WAYS=%0d CACHE_SIZE=%0d BLOCK_SIZE=%0d PREFETCH_EN=%0d (%0d sets), memory latency %0d ---",
                 LABEL, EVICT_POLICY, NUM_WAYS, CACHE_SIZE, BLOCK_SIZE, PREFETCH_EN, SETS, LATENCY);
        running = 1'b1;
        repeat (3) @(posedge clk);
        #1 rst = 1'b0;

        t_cold_miss;
        t_load_masks;
        t_store_hits;
        t_clean_evict;
        t_store_miss;
        t_address_split;
        if (NUM_WAYS >= 2)
            t_invalid_first;
        if (EVICT_POLICY == 0)
            t_lru;
        if (EVICT_POLICY == 1 && NUM_WAYS >= 4 && POW2_WAYS)
            t_plru;
        t_random_model;
        if (PREFETCH_EN) begin
            t_prefetch;
            t_collisions;
        end
        t_reset;
        t_held;
        t_stress;

        group = "end";
        wait_quiet;
        check("memory protocol (see mem_model.vh)", u_mem.errors == 0);
        if (u_mem.errors != 0 && hung)
            failed = failed + 1;
        $display("  %0s: %0d accesses in %0d cycles | memory reads %0d, write-backs %0d, write-throughs %0d",
                 LABEL, n_acc, cyc, u_mem.n_rd, u_mem.n_wb, u_mem.n_wt);
        running = 1'b0;
        o_done  = 1'b1;
    end

endmodule

// Every configuration below runs every test group that applies to it; the
// LRU, PLRU and prefetch groups only run where they apply. Latencies vary
// across rows since the cache must work for any delay.
module cache_tb;

    localparam N = 20;

    reg  [N-1:0] start;
    wire [N-1:0] done;

    // Base: LRU, fully associative, 16 KB, 128 B blocks.
    cache_harness #(.LABEL("base FA128 16K/128B, L=4"), .LATENCY(4), .SEED(101),
        .EVICT_POLICY(0), .NUM_WAYS(128), .CACHE_SIZE(16), .BLOCK_SIZE(128), .PREFETCH_EN(0))
        c0 (.i_start(start[0]), .o_done(done[0]));
    cache_harness #(.LABEL("base FA128 16K/128B, L=1"), .LATENCY(1), .SEED(102),
        .EVICT_POLICY(0), .NUM_WAYS(128), .CACHE_SIZE(16), .BLOCK_SIZE(128), .PREFETCH_EN(0))
        c1 (.i_start(start[1]), .o_done(done[1]));
    cache_harness #(.LABEL("base FA128 16K/128B, L=7"), .LATENCY(7), .SEED(103),
        .EVICT_POLICY(0), .NUM_WAYS(128), .CACHE_SIZE(16), .BLOCK_SIZE(128), .PREFETCH_EN(0))
        c2 (.i_start(start[2]), .o_done(done[2]));
    // Direct mapped.
    cache_harness #(.LABEL("DM 16K/128B, L=4"), .LATENCY(4), .SEED(104),
        .EVICT_POLICY(0), .NUM_WAYS(1), .CACHE_SIZE(16), .BLOCK_SIZE(128), .PREFETCH_EN(0))
        c3 (.i_start(start[3]), .o_done(done[3]));
    cache_harness #(.LABEL("DM 1K/16B, L=1"), .LATENCY(1), .SEED(105),
        .EVICT_POLICY(0), .NUM_WAYS(1), .CACHE_SIZE(1), .BLOCK_SIZE(16), .PREFETCH_EN(0))
        c4 (.i_start(start[4]), .o_done(done[4]));
    cache_harness #(.LABEL("DM 1K/16B, L=7"), .LATENCY(7), .SEED(106),
        .EVICT_POLICY(0), .NUM_WAYS(1), .CACHE_SIZE(1), .BLOCK_SIZE(16), .PREFETCH_EN(0))
        c5 (.i_start(start[5]), .o_done(done[5]));
    // Set associative, power-of-2 ways and the PDF's 3-way 12 KB example.
    cache_harness #(.LABEL("4-way 16K/128B, L=4"), .LATENCY(4), .SEED(107),
        .EVICT_POLICY(0), .NUM_WAYS(4), .CACHE_SIZE(16), .BLOCK_SIZE(128), .PREFETCH_EN(0))
        c6 (.i_start(start[6]), .o_done(done[6]));
    cache_harness #(.LABEL("3-way 12K/64B, L=4"), .LATENCY(4), .SEED(108),
        .EVICT_POLICY(0), .NUM_WAYS(3), .CACHE_SIZE(12), .BLOCK_SIZE(64), .PREFETCH_EN(0))
        c7 (.i_start(start[7]), .o_done(done[7]));
    cache_harness #(.LABEL("3-way 12K/64B, L=7"), .LATENCY(7), .SEED(109),
        .EVICT_POLICY(0), .NUM_WAYS(3), .CACHE_SIZE(12), .BLOCK_SIZE(64), .PREFETCH_EN(0))
        c8 (.i_start(start[8]), .o_done(done[8]));
    // Minimum block size.
    cache_harness #(.LABEL("2-way 1K/4B, L=4"), .LATENCY(4), .SEED(110),
        .EVICT_POLICY(0), .NUM_WAYS(2), .CACHE_SIZE(1), .BLOCK_SIZE(4), .PREFETCH_EN(0))
        c9 (.i_start(start[9]), .o_done(done[9]));
    cache_harness #(.LABEL("2-way 1K/4B, L=1"), .LATENCY(1), .SEED(111),
        .EVICT_POLICY(0), .NUM_WAYS(2), .CACHE_SIZE(1), .BLOCK_SIZE(4), .PREFETCH_EN(0))
        c10 (.i_start(start[10]), .o_done(done[10]));
    // PLRU.
    cache_harness #(.LABEL("PLRU 4-way 2K/16B, L=4"), .LATENCY(4), .SEED(112),
        .EVICT_POLICY(1), .NUM_WAYS(4), .CACHE_SIZE(2), .BLOCK_SIZE(16), .PREFETCH_EN(0))
        c11 (.i_start(start[11]), .o_done(done[11]));
    cache_harness #(.LABEL("PLRU 4-way 2K/16B, L=1"), .LATENCY(1), .SEED(113),
        .EVICT_POLICY(1), .NUM_WAYS(4), .CACHE_SIZE(2), .BLOCK_SIZE(16), .PREFETCH_EN(0))
        c12 (.i_start(start[12]), .o_done(done[12]));
    cache_harness #(.LABEL("PLRU 8-way 4K/32B, L=4"), .LATENCY(4), .SEED(114),
        .EVICT_POLICY(1), .NUM_WAYS(8), .CACHE_SIZE(4), .BLOCK_SIZE(32), .PREFETCH_EN(0))
        c13 (.i_start(start[13]), .o_done(done[13]));
    // Prefetch.
    cache_harness #(.LABEL("PF 2-way 1K/32B, L=4"), .LATENCY(4), .SEED(115),
        .EVICT_POLICY(0), .NUM_WAYS(2), .CACHE_SIZE(1), .BLOCK_SIZE(32), .PREFETCH_EN(1))
        c14 (.i_start(start[14]), .o_done(done[14]));
    cache_harness #(.LABEL("PF 2-way 1K/32B, L=1"), .LATENCY(1), .SEED(116),
        .EVICT_POLICY(0), .NUM_WAYS(2), .CACHE_SIZE(1), .BLOCK_SIZE(32), .PREFETCH_EN(1))
        c15 (.i_start(start[15]), .o_done(done[15]));
    cache_harness #(.LABEL("PF 2-way 1K/32B, L=7"), .LATENCY(7), .SEED(117),
        .EVICT_POLICY(0), .NUM_WAYS(2), .CACHE_SIZE(1), .BLOCK_SIZE(32), .PREFETCH_EN(1))
        c16 (.i_start(start[16]), .o_done(done[16]));
    cache_harness #(.LABEL("PF DM 1K/16B, L=4"), .LATENCY(4), .SEED(118),
        .EVICT_POLICY(0), .NUM_WAYS(1), .CACHE_SIZE(1), .BLOCK_SIZE(16), .PREFETCH_EN(1))
        c17 (.i_start(start[17]), .o_done(done[17]));
    cache_harness #(.LABEL("PF FA16 1K/64B, L=4"), .LATENCY(4), .SEED(119),
        .EVICT_POLICY(0), .NUM_WAYS(16), .CACHE_SIZE(1), .BLOCK_SIZE(64), .PREFETCH_EN(1))
        c18 (.i_start(start[18]), .o_done(done[18]));
    cache_harness #(.LABEL("PF PLRU 4-way 2K/32B, L=4"), .LATENCY(4), .SEED(120),
        .EVICT_POLICY(1), .NUM_WAYS(4), .CACHE_SIZE(2), .BLOCK_SIZE(32), .PREFETCH_EN(1))
        c19 (.i_start(start[19]), .o_done(done[19]));

    integer passed;
    integer failed;
    integer backstop;

    task summary;
        begin
            passed = c0.passed + c1.passed + c2.passed + c3.passed + c4.passed +
                     c5.passed + c6.passed + c7.passed + c8.passed + c9.passed +
                     c10.passed + c11.passed + c12.passed + c13.passed + c14.passed +
                     c15.passed + c16.passed + c17.passed + c18.passed + c19.passed;
            failed = c0.failed + c1.failed + c2.failed + c3.failed + c4.failed +
                     c5.failed + c6.failed + c7.failed + c8.failed + c9.failed +
                     c10.failed + c11.failed + c12.failed + c13.failed + c14.failed +
                     c15.failed + c16.failed + c17.failed + c18.failed + c19.failed +
                     backstop;
            $display("============================================");
            $display("%0d passed, %0d failed", passed, failed);
            if (failed == 0)
                $display("ALL TESTS PASSED");
            else
                $display("TEST FAILED");
        end
    endtask

    integer c;

    initial begin
        if ($test$plusargs("vcd")) begin
            $dumpfile("cache.vcd");
            $dumpvars(0, cache_tb);
        end
        start    = {N{1'b0}};
        backstop = 0;
        $display("========== cache testbench ==========");
        for (c = 0; c < N; c = c + 1) begin
            start[c] = 1'b1;
            wait (done[c]);
        end
        summary;
        $finish;
    end

    // Backstop in case a harness's own watchdogs never fire.
    initial begin
        #(64'd10 * N * 600000);
        $display("[FAIL] cache: simulation time limit reached (start %b, done %b)",
                 start, done);
        backstop = 1;
        summary;
        $finish;
    end

endmodule

`default_nettype wire
