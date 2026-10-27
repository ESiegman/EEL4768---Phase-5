// Harness shared by the five hart testbenches (hart_*_tb.v): one hart with a
// given cache configuration and memory latency, its instruction and data
// main memories, and every check that runs on it. Each hart_*_tb.v
// instantiates one harness per configuration/latency pair and starts them
// one after another, so the log reads in order.
//
// A .vh, not a .v, for the same reason as mem_model.vh: run_test.sh would
// otherwise compile it into every check and lint it as a design file.
//
// A harness resets the hart, runs traces/random_program.hex until the final
// ebreak retires, then checks:
//  * The run itself: it halts within MAX_CYCLES, with no gap of IDLE_LIMIT
//    cycles between two retirements. On a timeout it prints the last
//    retired PC.
//  * The retire stream, against a shadow data memory that starts at all
//    zeros and takes every retiring store. This is the main checker: the
//    dcache is write-back, so main memory is stale for dirty lines at halt.
//      - every retiring load's o_retire_dmem_rdata (the full word the dcache
//        returned) equals the shadow word before that load
//      - o_retire_inst is the program word at o_retire_pc (the icache)
//      - o_retire_pc starts at RESET_ADDR and each one equals the previous
//        o_retire_next_pc; next_pc is pc + 4 except for branches and jumps
//      - no traps (the trace has none)
//      - the dmem mask fits the instruction and the address is in data
//        memory; a load's rd value is the right lanes of rdata, sign or zero
//        extended, and a store's wdata holds rs2 in the masked lanes
//  * After halt, the shadow memory equals traces/random_dmem.hex.
//  * Main memory after halt. Words that differ from random_dmem.hex are
//    only legal inside lines the dcache still holds dirty. Without access to
//    the dcache's storage this cannot be checked word by word, so the count
//    is reported, and it is a failure only if more lines differ than the
//    dcache has. With HART_TB_PEEK_DCACHE defined (see below) the dirty
//    lines are read out of dut.dcache and overlaid, and the result must
//    match exactly.
//  * The memory models saw no protocol errors (mem_model.vh).
// It then prints cycles, instructions retired, CPI and memory traffic, which
// also catches a cache that "passes" by never hitting.

`ifndef HART_TB_COMMON_VH
`define HART_TB_COMMON_VH

`include "mem_model.vh"

module hart_harness #(
    // Printed in front of every message from this instance.
    parameter LABEL        = "base, L=4",
    parameter EVICT_POLICY = 0,
    parameter NUM_WAYS     = 128,
    parameter CACHE_SIZE   = 16,
    parameter BLOCK_SIZE   = 128,
    parameter PREFETCH_EN  = 0,
    // Main memory read latency in cycles, for both memories.
    parameter LATENCY      = 4,
    // Watchdogs: give up after this many cycles in total, or after this many
    // cycles in a row without a retirement.
    parameter MAX_CYCLES   = 600000,
    parameter IDLE_LIMIT   = 2000
) (
    input  wire i_start,
    output reg  o_done
);

    localparam [31:0] IMEM_BASE   = 32'h00400000;
    localparam [31:0] DMEM_BASE   = 32'h10010000;
    localparam        MEM_WORDS   = 16384;  // 64 KB each, as in phase_5.pdf 4.6
    // One spare block past the end of each memory, so a next-line prefetch
    // after a miss on the last line is still a legal read. Nothing should
    // ever write it.
    localparam        GUARD_WORDS = BLOCK_SIZE / 4;
    localparam        NUM_LINES   = CACHE_SIZE * 1024 / BLOCK_SIZE;
    localparam        LINE_WORDS  = BLOCK_SIZE / 4;
    localparam        MAX_MSGS    = 20;

    // Results, read by the top-level testbench.
    integer passed;
    integer failed;

    // ------------------------------------------------------------------
    // Clock (runs only while this instance is active), reset, DUT
    // ------------------------------------------------------------------
    reg clk;
    reg rst;
    reg running;

    always begin
        wait (running);
        #5 clk = ~clk;
    end

    wire                    imem_rd;
    wire [31:0]             imem_rd_addr;
    wire                    imem_valid;
    wire [BLOCK_SIZE*8-1:0] imem_rd_data;
    wire                    dmem_rd;
    wire [31:0]             dmem_rd_addr;
    wire                    dmem_valid;
    wire [BLOCK_SIZE*8-1:0] dmem_rd_data;
    wire                    dmem_wr;
    wire [31:0]             dmem_wr_addr;
    wire [BLOCK_SIZE*8-1:0] dmem_wr_data;
    wire [BLOCK_SIZE-1:0]   dmem_wr_be;

    wire        retire_valid;
    wire [31:0] retire_inst;
    wire        retire_trap;
    wire        retire_halt;
    wire [ 4:0] retire_rs1_raddr;
    wire [ 4:0] retire_rs2_raddr;
    wire [31:0] retire_rs1_rdata;
    wire [31:0] retire_rs2_rdata;
    wire [ 4:0] retire_rd_waddr;
    wire [31:0] retire_rd_wdata;
    wire [31:0] retire_dmem_addr;
    wire [ 3:0] retire_dmem_mask;
    wire        retire_dmem_ren;
    wire        retire_dmem_wen;
    wire [31:0] retire_dmem_rdata;
    wire [31:0] retire_dmem_wdata;
    wire [31:0] retire_pc;
    wire [31:0] retire_next_pc;

    hart #(
        .RESET_ADDR  (IMEM_BASE),
        .EVICT_POLICY(EVICT_POLICY),
        .NUM_WAYS    (NUM_WAYS),
        .CACHE_SIZE  (CACHE_SIZE),
        .BLOCK_SIZE  (BLOCK_SIZE),
        .PREFETCH_EN (PREFETCH_EN)
    ) dut (
        .i_clk              (clk),
        .i_rst              (rst),
        .o_imem_rd          (imem_rd),
        .o_imem_rd_addr     (imem_rd_addr),
        .i_imem_valid       (imem_valid),
        .i_imem_rd_data     (imem_rd_data),
        .o_dmem_rd          (dmem_rd),
        .o_dmem_rd_addr     (dmem_rd_addr),
        .i_dmem_valid       (dmem_valid),
        .i_dmem_rd_data     (dmem_rd_data),
        .o_dmem_wr          (dmem_wr),
        .o_dmem_wr_addr     (dmem_wr_addr),
        .o_dmem_wr_data     (dmem_wr_data),
        .o_dmem_wr_be       (dmem_wr_be),
        .o_retire_valid     (retire_valid),
        .o_retire_inst      (retire_inst),
        .o_retire_trap      (retire_trap),
        .o_retire_halt      (retire_halt),
        .o_retire_rs1_raddr (retire_rs1_raddr),
        .o_retire_rs2_raddr (retire_rs2_raddr),
        .o_retire_rs1_rdata (retire_rs1_rdata),
        .o_retire_rs2_rdata (retire_rs2_rdata),
        .o_retire_rd_waddr  (retire_rd_waddr),
        .o_retire_rd_wdata  (retire_rd_wdata),
        .o_retire_dmem_addr (retire_dmem_addr),
        .o_retire_dmem_mask (retire_dmem_mask),
        .o_retire_dmem_ren  (retire_dmem_ren),
        .o_retire_dmem_wen  (retire_dmem_wen),
        .o_retire_dmem_rdata(retire_dmem_rdata),
        .o_retire_dmem_wdata(retire_dmem_wdata),
        .o_retire_pc        (retire_pc),
        .o_retire_next_pc   (retire_next_pc)
    );

    // Instruction memory is read only: its write port is tied off.
    main_mem #(
        .BLOCK_SIZE(BLOCK_SIZE),
        .LATENCY   (LATENCY),
        .BASE_ADDR (IMEM_BASE),
        .SIZE_WORDS(MEM_WORDS + GUARD_WORDS),
        .NAME      ({LABEL, ", imem"})
    ) u_imem (
        .i_clk     (clk),
        .i_rst     (rst),
        .i_rd      (imem_rd),
        .i_rd_addr (imem_rd_addr),
        .o_rd_valid(imem_valid),
        .o_rd_data (imem_rd_data),
        .i_wr      (1'b0),
        .i_wr_addr (32'd0),
        .i_wr_data ({BLOCK_SIZE*8{1'b0}}),
        .i_wr_be   ({BLOCK_SIZE{1'b0}})
    );

    main_mem #(
        .BLOCK_SIZE(BLOCK_SIZE),
        .LATENCY   (LATENCY),
        .BASE_ADDR (DMEM_BASE),
        .SIZE_WORDS(MEM_WORDS + GUARD_WORDS),
        .NAME      ({LABEL, ", dmem"})
    ) u_dmem (
        .i_clk     (clk),
        .i_rst     (rst),
        .i_rd      (dmem_rd),
        .i_rd_addr (dmem_rd_addr),
        .o_rd_valid(dmem_valid),
        .o_rd_data (dmem_rd_data),
        .i_wr      (dmem_wr),
        .i_wr_addr (dmem_wr_addr),
        .i_wr_data (dmem_wr_data),
        .i_wr_be   (dmem_wr_be)
    );

    // ------------------------------------------------------------------
    // Checks
    // ------------------------------------------------------------------
    task check;
        input [8*80-1:0] what;
        input            ok;
        begin
            if (ok) begin
                passed = passed + 1;
            end else begin
                failed = failed + 1;
                $display("[FAIL] %0s: %0s", LABEL, what);
            end
        end
    endtask

    // Mismatches found while running, by kind. Only the first MAX_MSGS of
    // them, all kinds together, are printed.
    integer bad_load;     // load data from the dcache
    integer bad_inst;     // instruction word from the icache
    integer bad_pc;       // retire PC sequence
    integer bad_lane;     // dmem mask/address/lane formatting
    integer n_traps;
    integer msgs;

    integer cycles;       // cycles since reset was released
    integer idle;         // cycles since the last retirement
    integer retired;
    integer n_loads;
    integer n_stores;
    integer prog_words;
    reg     halted;
    reg     have_prev;
    reg [31:0] prev_next_pc;
    reg [31:0] last_pc;

    reg [31:0] shadow [0:MEM_WORDS-1];
    reg [31:0] golden [0:MEM_WORDS-1];

    // Prints one retire-stream mismatch (the first MAX_MSGS of them).
    task report;
        input [8*40-1:0] what;
        input [31:0]     got;
        input [31:0]     want;
        begin
            if (msgs < MAX_MSGS && (retire_dmem_ren === 1'b1 || retire_dmem_wen === 1'b1))
                $display("[FAIL] %0s: %0s at pc %h (inst %h, dmem addr %h): got %h, expected %h (retire #%0d, cycle %0d)",
                         LABEL, what, retire_pc, retire_inst, retire_dmem_addr,
                         got, want, retired, cycles);
            else if (msgs < MAX_MSGS)
                $display("[FAIL] %0s: %0s at pc %h (inst %h): got %h, expected %h (retire #%0d, cycle %0d)",
                         LABEL, what, retire_pc, retire_inst, got, want, retired, cycles);
            else if (msgs == MAX_MSGS)
                $display("[FAIL] %0s: further retire mismatches not shown", LABEL);
            msgs = msgs + 1;
        end
    endtask

    // Byte offset a load or store mask implies (0 for an illegal mask).
    function [1:0] mask_offset;
        input [3:0] mask;
        begin
            case (mask)
                4'b0010: mask_offset = 2'd1;
                4'b0100: mask_offset = 2'd2;
                4'b1000: mask_offset = 2'd3;
                4'b1100: mask_offset = 2'd2;
                default: mask_offset = 2'd0;
            endcase
        end
    endfunction

    // Whether a mask is legal for a load/store funct3 (size in [1:0]).
    function mask_ok;
        input [1:0] size;
        input [3:0] mask;
        begin
            case (size)
                2'd0:    mask_ok = (mask == 4'b0001) || (mask == 4'b0010) ||
                                   (mask == 4'b0100) || (mask == 4'b1000);
                2'd1:    mask_ok = (mask == 4'b0011) || (mask == 4'b1100);
                2'd2:    mask_ok = (mask == 4'b1111);
                default: mask_ok = 1'b0;
            endcase
        end
    endfunction

    reg  [31:0] widx;
    reg  [31:0] want;
    reg  [31:0] lanes;
    reg  [ 2:0] funct3;
    reg  [ 6:0] opcode;
    integer     b;

    // Retire monitor, sampled mid-cycle while the outputs are stable.
    always @(negedge clk) begin
        if (running && !rst && !halted) begin
            cycles = cycles + 1;
            idle   = idle + 1;
            if (retire_valid === 1'b1) begin
                retired = retired + 1;
                idle    = 0;
                last_pc = retire_pc;
                opcode  = retire_inst[6:0];
                funct3  = retire_inst[14:12];

                // Instruction word, from the icache.
                if (retire_pc < IMEM_BASE || retire_pc >= IMEM_BASE + 4 * MEM_WORDS ||
                    retire_pc[1:0] != 2'b00) begin
                    bad_pc = bad_pc + 1;
                    report("retire PC outside instruction memory", retire_pc, IMEM_BASE);
                end else if (retire_inst !== u_imem.mem[(retire_pc - IMEM_BASE) >> 2]) begin
                    bad_inst = bad_inst + 1;
                    report("instruction word", retire_inst,
                           u_imem.mem[(retire_pc - IMEM_BASE) >> 2]);
                end

                // PC sequence.
                if (!have_prev && retire_pc !== IMEM_BASE) begin
                    bad_pc = bad_pc + 1;
                    report("first retired PC", retire_pc, IMEM_BASE);
                end else if (have_prev && retire_pc !== prev_next_pc) begin
                    bad_pc = bad_pc + 1;
                    report("PC (previous next_pc)", retire_pc, prev_next_pc);
                end
                if (opcode != 7'b1100011 && opcode != 7'b1101111 &&
                    opcode != 7'b1100111 && !retire_halt &&
                    retire_next_pc !== retire_pc + 32'd4) begin
                    bad_pc = bad_pc + 1;
                    report("next_pc of a non-jump", retire_next_pc, retire_pc + 32'd4);
                end
                have_prev    = 1'b1;
                prev_next_pc = retire_next_pc;

                if (retire_trap === 1'b1) begin
                    n_traps = n_traps + 1;
                    report("unexpected trap", {31'd0, retire_trap}, 32'd0);
                end

                if (retire_dmem_ren === 1'b1 && retire_dmem_wen === 1'b1) begin
                    bad_lane = bad_lane + 1;
                    report("dmem ren and wen both high", 32'd1, 32'd0);
                end else if (retire_dmem_ren === 1'b1 || retire_dmem_wen === 1'b1) begin
                    widx = (retire_dmem_addr - DMEM_BASE) >> 2;
                    if (retire_dmem_addr < DMEM_BASE ||
                        retire_dmem_addr >= DMEM_BASE + 4 * MEM_WORDS ||
                        retire_dmem_addr[1:0] != 2'b00) begin
                        bad_lane = bad_lane + 1;
                        report("dmem address outside data memory", retire_dmem_addr, DMEM_BASE);
                    end else if (!mask_ok(funct3[1:0], retire_dmem_mask)) begin
                        bad_lane = bad_lane + 1;
                        report("dmem mask", {28'd0, retire_dmem_mask}, 32'd0);
                    end else if (retire_dmem_ren === 1'b1) begin
                        // The dcache's word, then the hart's extraction.
                        n_loads = n_loads + 1;
                        if (retire_dmem_rdata !== shadow[widx]) begin
                            bad_load = bad_load + 1;
                            report("load data from the dcache", retire_dmem_rdata, shadow[widx]);
                        end
                        lanes = retire_dmem_rdata >> (8 * mask_offset(retire_dmem_mask));
                        case (funct3)
                            3'b000:  want = {{24{lanes[7]}}, lanes[7:0]};
                            3'b001:  want = {{16{lanes[15]}}, lanes[15:0]};
                            3'b100:  want = {24'd0, lanes[7:0]};
                            3'b101:  want = {16'd0, lanes[15:0]};
                            default: want = lanes;
                        endcase
                        if (retire_rd_waddr != 5'd0 && retire_rd_wdata !== want) begin
                            bad_lane = bad_lane + 1;
                            report("load rd value from rdata", retire_rd_wdata, want);
                        end
                    end else begin
                        n_stores = n_stores + 1;
                        want = retire_rs2_rdata << (8 * mask_offset(retire_dmem_mask));
                        for (b = 0; b < 4; b = b + 1) begin
                            if (retire_dmem_mask[b] &&
                                retire_dmem_wdata[8*b +: 8] !== want[8*b +: 8]) begin
                                bad_lane = bad_lane + 1;
                                report("store wdata lane vs rs2", retire_dmem_wdata, want);
                            end
                            if (retire_dmem_mask[b])
                                shadow[widx][8*b +: 8] = retire_dmem_wdata[8*b +: 8];
                        end
                    end
                end

                if (retire_halt === 1'b1) begin
                    if (retire_inst !== 32'h00100073)
                        report("halt on a non-ebreak", retire_inst, 32'h00100073);
                    halted = 1'b1;
                end
            end else if (retire_valid !== 1'b0) begin
                bad_pc = bad_pc + 1;
                report("o_retire_valid is X/Z", 32'd0, 32'd0);
            end
        end
    end

    // ------------------------------------------------------------------
    // Optional: overlay the dcache's dirty lines onto main memory
    // ------------------------------------------------------------------
`ifdef HART_TB_PEEK_DCACHE
    // Off by default because it reads cache.v's storage directly. Once the
    // names there are final, point these four macros at them and define
    // HART_TB_PEEK_DCACHE (a `define at the top of each hart_*_tb.v, or
    // iverilog -DHART_TB_PEEK_DCACHE). s is a runtime set index and w a
    // constant way index (a genvar), so they work for flat arrays and for
    // per-way generate blocks alike. DC_TAG is the stored tag, i.e. the
    // address bits above the set index.
    `ifndef DC_VALID
    `define DC_VALID(s, w) dut.dcache.valid_q[(s) * NUM_WAYS + (w)]
    `define DC_DIRTY(s, w) dut.dcache.dirty_q[(s) * NUM_WAYS + (w)]
    `define DC_TAG(s, w)   dut.dcache.tag_q[(s) * NUM_WAYS + (w)]
    `define DC_DATA(s, w)  dut.dcache.data_q[(s) * NUM_WAYS + (w)]
    `endif

    localparam NUM_SETS    = NUM_LINES / NUM_WAYS;
    localparam OFFSET_BITS = $clog2(BLOCK_SIZE);
    localparam INDEX_BITS  = $clog2(NUM_SETS);

    reg [31:0] merged [0:MEM_WORDS-1];
    integer    n_dirty;      // dirty lines overlaid
    integer    n_dirty_bad;  // dirty lines outside data memory
    event      do_overlay;

    genvar gw;
    generate
        for (gw = 0; gw < NUM_WAYS; gw = gw + 1) begin : g_overlay
            integer    s;
            integer    j;
            reg [31:0] line;
            always @(do_overlay) begin
                for (s = 0; s < NUM_SETS; s = s + 1) begin
                    if (`DC_VALID(s, gw) === 1'b1 && `DC_DIRTY(s, gw) === 1'b1) begin
                        line = (`DC_TAG(s, gw) << (OFFSET_BITS + INDEX_BITS)) |
                               (s << OFFSET_BITS);
                        if (line < DMEM_BASE || line >= DMEM_BASE + 4 * MEM_WORDS) begin
                            n_dirty_bad = n_dirty_bad + 1;
                            $display("[FAIL] %0s: dcache set %0d way %0d holds dirty line %h, outside data memory",
                                     LABEL, s, gw, line);
                        end else begin
                            n_dirty = n_dirty + 1;
                            for (j = 0; j < LINE_WORDS; j = j + 1)
                                merged[((line - DMEM_BASE) >> 2) + j] =
                                    `DC_DATA(s, gw) >> (32 * j);
                        end
                    end
                end
            end
        end
    endgenerate
`endif

    // ------------------------------------------------------------------
    // Run
    // ------------------------------------------------------------------
    reg [8*48-1:0]  why;
    reg [8*256-1:0] prog_path;
    reg [8*256-1:0] dmem_path;
    integer i;
    integer fd;
    integer n_words;
    integer stale_words;
    integer stale_lines;
    integer line_bad;
    integer guard_bad;
    integer cpi_x100;

    initial begin
        clk         = 1'b0;
        rst         = 1'b1;
        running     = 1'b0;
        o_done      = 1'b0;
        passed      = 0;
        failed      = 0;
        bad_load    = 0;
        bad_inst    = 0;
        bad_pc      = 0;
        bad_lane    = 0;
        n_traps     = 0;
        msgs        = 0;
        cycles      = 0;
        idle        = 0;
        retired     = 0;
        n_loads     = 0;
        n_stores    = 0;
        halted      = 1'b0;
        have_prev   = 1'b0;
        stale_words = 0;
        stale_lines = 0;
        for (i = 0; i < MEM_WORDS; i = i + 1)
            shadow[i] = 32'd0;

        wait (i_start === 1'b1);

        // vvp runs from build/ under make test; fall back to the repo root
        // for a manual run from there.
        prog_path = "../traces/random_program.hex";
        dmem_path = "../traces/random_dmem.hex";
        fd = $fopen(prog_path, "r");
        if (fd == 0) begin
            prog_path = "traces/random_program.hex";
            dmem_path = "traces/random_dmem.hex";
        end else begin
            $fclose(fd);
        end
        u_imem.load_hex(prog_path, prog_words);
        $readmemh(dmem_path, golden);

        $display("--- %0s: EVICT_POLICY=%0d NUM_WAYS=%0d CACHE_SIZE=%0d BLOCK_SIZE=%0d PREFETCH_EN=%0d, memory latency %0d ---",
                 LABEL, EVICT_POLICY, NUM_WAYS, CACHE_SIZE, BLOCK_SIZE, PREFETCH_EN, LATENCY);

        running = 1'b1;
        repeat (5) @(posedge clk);
        #1 rst = 1'b0;

        while (!halted && cycles < MAX_CYCLES && idle < IDLE_LIMIT &&
               retired < 2 * prog_words)
            @(posedge clk);

        if (!halted) begin
            $sformat(why, "%0s", idle >= IDLE_LIMIT ? "nothing retired for IDLE_LIMIT cycles" :
                                 cycles >= MAX_CYCLES ? "MAX_CYCLES reached" :
                                                        "retired twice the program length");
            if (retired == 0)
                $display("[FAIL] %0s: timed out at cycle %0d: %0s; nothing retired at all",
                         LABEL, cycles, why);
            else
                $display("[FAIL] %0s: timed out at cycle %0d: %0s; %0d instructions retired, last retired PC %h",
                         LABEL, cycles, why, retired, last_pc);
        end

        // Let memory traffic still in flight (e.g. a prefetch) land.
        repeat (4 * LATENCY + 20) @(posedge clk);
        running = 1'b0;

        check("program ran to its final ebreak", halted);
        // With nothing retired these would pass without checking anything.
        if (retired > 0) begin
            check("retired instruction words match the program (icache)", bad_inst == 0);
            check("retire PC sequence", bad_pc == 0);
            check("no traps", n_traps == 0);
            check("load data returned by the dcache", bad_load == 0);
            check("dmem address/mask/lane formatting", bad_lane == 0);
        end
        if (msgs > 0)
            $display("       %0d retire mismatches: dcache load data %0d, icache word %0d, pc %0d, lanes %0d, traps %0d",
                     msgs, bad_load, bad_inst, bad_pc, bad_lane, n_traps);

        if (halted) begin
            // Architectural result, from the retire stream.
            n_words = 0;
            for (i = 0; i < MEM_WORDS; i = i + 1) begin
                if (shadow[i] !== golden[i]) begin
                    if (n_words < 10)
                        $display("[FAIL] %0s: final memory (retire stream) at %h: got %h, expected %h",
                                 LABEL, DMEM_BASE + 4 * i, shadow[i], golden[i]);
                    n_words = n_words + 1;
                end
            end
            check("final data memory from the retire stream matches random_dmem.hex",
                  n_words == 0);

            // Main memory: stale words are fine only inside dirty dcache lines.
            stale_words = 0;
            stale_lines = 0;
            for (i = 0; i < MEM_WORDS; i = i + 1) begin
                if (i % LINE_WORDS == 0)
                    line_bad = 0;
                if (u_dmem.mem[i] !== golden[i]) begin
                    stale_words = stale_words + 1;
                    if (!line_bad)
                        stale_lines = stale_lines + 1;
                    line_bad = 1;
                end
            end
            if (stale_lines > NUM_LINES)
                $display("[FAIL] %0s: %0d lines of main memory differ from random_dmem.hex, but the dcache only holds %0d",
                         LABEL, stale_lines, NUM_LINES);
            check("main memory: no more stale lines than the dcache can hold dirty",
                  stale_lines <= NUM_LINES);
            guard_bad = 0;
            for (i = MEM_WORDS; i < MEM_WORDS + GUARD_WORDS; i = i + 1)
                if (u_dmem.mem[i] !== 32'd0)
                    guard_bad = guard_bad + 1;
            check("nothing written past the end of data memory", guard_bad == 0);

`ifdef HART_TB_PEEK_DCACHE
            for (i = 0; i < MEM_WORDS; i = i + 1)
                merged[i] = u_dmem.mem[i];
            n_dirty     = 0;
            n_dirty_bad = 0;
            -> do_overlay;
            #1;
            n_words = 0;
            for (i = 0; i < MEM_WORDS; i = i + 1) begin
                if (merged[i] !== golden[i]) begin
                    if (n_words < 10)
                        $display("[FAIL] %0s: main memory + dirty dcache lines at %h: got %h (memory %h), expected %h",
                                 LABEL, DMEM_BASE + 4 * i, merged[i], u_dmem.mem[i], golden[i]);
                    n_words = n_words + 1;
                end
            end
            check("main memory with dirty dcache lines overlaid matches random_dmem.hex",
                  n_words == 0 && n_dirty_bad == 0);
`endif
        end

        check("memory protocol (see mem_model.vh)",
              u_imem.errors == 0 && u_dmem.errors == 0);

        cpi_x100 = retired > 0 ? (cycles * 100) / retired : 0;
        $display("  %0s: %0d cycles, %0d retired, CPI %0d.%02d | imem reads %0d | dmem reads %0d, write-backs %0d, write-throughs %0d | %0d loads, %0d stores",
                 LABEL, cycles, retired, cpi_x100 / 100, cpi_x100 % 100,
                 u_imem.n_rd, u_dmem.n_rd, u_dmem.n_wb, u_dmem.n_wt,
                 n_loads, n_stores);
`ifdef HART_TB_PEEK_DCACHE
        if (halted)
            $display("  %0s: main memory at halt: %0d stale words in %0d lines; %0d dirty lines overlaid",
                     LABEL, stale_words, stale_lines, n_dirty);
`else
        if (halted)
            $display("  %0s: main memory at halt: %0d stale words in %0d lines (dirty in the dcache; not checked word by word without HART_TB_PEEK_DCACHE)",
                     LABEL, stale_words, stale_lines);
`endif
        if (u_dmem.n_wr_empty + u_dmem.n_rd_wr_same + u_dmem.n_wr_inflight > 0)
            $display("  %0s: note: dmem writes with no byte enabled %0d, read+write of one line in a cycle %0d, writes to a line with a read in flight %0d",
                     LABEL, u_dmem.n_wr_empty, u_dmem.n_rd_wr_same, u_dmem.n_wr_inflight);

        o_done = 1'b1;
    end

endmodule

`endif
