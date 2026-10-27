// Main memory model shared by the phase 5 testbenches (phase_5.pdf 4.6).
//
// This is a .vh, not a .v, on purpose: run_test.sh compiles every
// non-testbench .v at the repo root into every check and `ece552 validate`
// lints them all, and this model uses loops, file I/O and other
// simulation-only constructs. Testbenches pull it in with
//     `include "mem_model.vh"
// which iverilog resolves against the directory it is run from (the repo
// root under `make test` and CI), not against the including file.
//
// Behavior
//  * SIZE_WORDS 32-bit words starting at BASE_ADDR, byte addressable and
//    little endian: byte b of a block travels on bus bits [8*b+7:8*b].
//  * Reads are fully pipelined. A request (i_rd high for a cycle with a
//    block-aligned i_rd_addr) is accepted every cycle it is high, and each
//    one is answered exactly LATENCY cycles later by o_rd_valid high for one
//    cycle with the whole block on o_rd_data. With LATENCY = 4 a request in
//    cycle 1 is answered in cycle 5, as in the PDF 4.7 timeline. Answers
//    come back in order and any number can be in flight. LATENCY >= 1.
//    o_rd_data is X whenever o_rd_valid is low.
//  * When the read data is taken from the array (SAMPLE_AT_RESPONSE):
//      0  when the request is accepted (default, stricter). A write that
//         lands while the read is in flight is not in the answer, so a cache
//         that lets a write-through store miss race an in-flight prefetch of
//         the same line ends up holding stale data.
//      1  in the cycle the answer is sent.
//    Either way a read sees every write sent in a cycle before its sampling
//    cycle and none sent in the same cycle.
//  * Writes (i_wr high for a cycle, block-aligned i_wr_addr, data in place
//    within the block, one enable bit per byte) are applied at the clock
//    edge that ends the cycle they are sent in. Only enabled bytes change.
//    Writes never stall.
//  * Errors, printed as [FAIL] and counted in `errors` (the testbench adds
//    them to its own failure count): an access outside
//    [BASE_ADDR, BASE_ADDR + 4*SIZE_WORDS), an address that is not block
//    aligned, and X/Z on i_rd, i_wr or the byte enables outside reset.
//  * Counted for the testbench summary: reads, writes, write-backs (every
//    byte enabled), write-throughs (some bytes enabled), and some
//    informational protocol counters (see the declarations below).
//
// The testbench reaches into an instance for setup and final checks:
// `mem[i]` is the word at BASE_ADDR + 4*i, and load_hex/peek/poke below
// cover the common cases.

`ifndef MEM_MODEL_VH
`define MEM_MODEL_VH

module main_mem #(
    parameter BLOCK_SIZE         = 128,
    parameter LATENCY            = 4,
    parameter BASE_ADDR          = 32'h10010000,
    parameter SIZE_WORDS         = 16384,
    parameter SAMPLE_AT_RESPONSE = 0,
    // Prefix for this instance's messages.
    parameter NAME               = "mem"
) (
    input  wire                    i_clk,
    // Only masks the X/Z checks while the cache is in reset. The array and
    // the reads in flight are not reset.
    input  wire                    i_rst,
    // Ports are named from the memory's side: i_rd is the cache's o_mem_rd,
    // o_rd_valid is the cache's i_mem_valid, and so on.
    input  wire                    i_rd,
    input  wire [31:0]             i_rd_addr,
    output reg                     o_rd_valid,
    output reg  [BLOCK_SIZE*8-1:0] o_rd_data,
    input  wire                    i_wr,
    input  wire [31:0]             i_wr_addr,
    input  wire [BLOCK_SIZE*8-1:0] i_wr_data,
    input  wire [BLOCK_SIZE-1:0]   i_wr_be
);

    localparam BW          = BLOCK_SIZE * 8;
    localparam BLOCK_WORDS = BLOCK_SIZE / 4;
    localparam MAX_MSGS    = 10;

    reg [31:0] mem [0:SIZE_WORDS-1];

    // Read pipeline. After the shift at a clock edge, stage k holds the
    // request accepted k edges earlier, so stage LATENCY-1 is answered now.
    reg          p_valid [0:LATENCY-1];
    reg [31:0]   p_addr  [0:LATENCY-1];
    reg [BW-1:0] p_data  [0:LATENCY-1];

    integer cycle;          // clock edges seen
    integer errors;
    integer n_rd;           // read requests accepted
    integer n_wr;           // write pulses
    integer n_wb;           // writes with every byte enabled (write-backs)
    integer n_wt;           // writes with some bytes enabled (write-throughs)
    integer n_wr_empty;     // writes with no byte enabled
    integer n_rd_repeat;    // read of the block the previous cycle also read
    integer n_rd_wr_same;   // read and write of one block in the same cycle
    integer n_wr_inflight;  // write to a block that has a read in flight
    integer msgs;
    integer i;
    integer k;
    reg        prev_rd;
    reg [31:0] prev_rd_addr;
    reg        rd_ok;
    reg        wr_ok;
    reg        resp_ok [0:LATENCY-1];

    initial begin
        for (i = 0; i < SIZE_WORDS; i = i + 1)
            mem[i] = 32'd0;
        for (k = 0; k < LATENCY; k = k + 1) begin
            p_valid[k] = 1'b0;
            p_addr[k]  = 32'd0;
            p_data[k]  = {BW{1'bx}};
            resp_ok[k] = 1'b0;
        end
        o_rd_valid    = 1'b0;
        o_rd_data     = {BW{1'bx}};
        cycle         = 0;
        errors        = 0;
        n_rd          = 0;
        n_wr          = 0;
        n_wb          = 0;
        n_wt          = 0;
        n_wr_empty    = 0;
        n_rd_repeat   = 0;
        n_rd_wr_same  = 0;
        n_wr_inflight = 0;
        msgs          = 0;
        prev_rd       = 1'b0;
        prev_rd_addr  = 32'd0;
    end

    task error;
        input [8*96-1:0] what;
        input [31:0]     addr;
        begin
            errors = errors + 1;
            if (msgs < MAX_MSGS)
                $display("[FAIL] %0s: %0s (address %h, memory cycle %0d)",
                         NAME, what, addr, cycle);
            else if (msgs == MAX_MSGS)
                $display("[FAIL] %0s: further memory errors not shown", NAME);
            msgs = msgs + 1;
        end
    endtask

    function in_range;
        input [31:0] a;
        begin
            in_range = ({32'd0, a} >= {32'd0, BASE_ADDR}) &&
                       ({32'd0, a} + BLOCK_SIZE <=
                        {32'd0, BASE_ADDR} + 4 * SIZE_WORDS);
        end
    endfunction

    // Checks a request address, reporting any problem; ok = 1 if it is usable.
    task check_addr;
        input        is_wr;
        input [31:0] a;
        output       ok;
        begin
            ok = 1'b0;
            if (^a === 1'bx)
                error(is_wr ? "write address has X/Z bits"
                            : "read address has X/Z bits", a);
            else if (!in_range(a))
                error(is_wr ? "write outside the modeled range"
                            : "read outside the modeled range", a);
            else if (a % BLOCK_SIZE != 0)
                error(is_wr ? "write address not block aligned"
                            : "read address not block aligned", a);
            else
                ok = 1'b1;
        end
    endtask

    function [BW-1:0] read_block;
        input [31:0] a;
        integer w;
        begin
            for (w = 0; w < BLOCK_WORDS; w = w + 1)
                read_block[32*w +: 32] = mem[(a - BASE_ADDR) / 4 + w];
        end
    endfunction

    always @(posedge i_clk) begin
        cycle = cycle + 1;

        // 1. Shift the read pipeline and accept this cycle's request. With
        //    SAMPLE_AT_RESPONSE = 0 the block is copied here, before this
        //    edge's write is applied.
        for (k = LATENCY - 1; k > 0; k = k - 1) begin
            p_valid[k] = p_valid[k-1];
            p_addr[k]  = p_addr[k-1];
            p_data[k]  = p_data[k-1];
            resp_ok[k] = resp_ok[k-1];
        end
        p_valid[0] = 1'b0;
        p_data[0]  = {BW{1'bx}};
        resp_ok[0] = 1'b0;
        if (i_rd === 1'b1) begin
            n_rd = n_rd + 1;
            if (prev_rd && i_rd_addr === prev_rd_addr)
                n_rd_repeat = n_rd_repeat + 1;
            check_addr(1'b0, i_rd_addr, rd_ok);
            // An unusable request is still answered (with X) so the cache
            // does not hang on top of the reported error.
            p_valid[0] = 1'b1;
            p_addr[0]  = i_rd_addr;
            resp_ok[0] = rd_ok;
            if (rd_ok && SAMPLE_AT_RESPONSE == 0)
                p_data[0] = read_block(i_rd_addr);
        end else if (i_rd !== 1'b0 && i_rst !== 1'b1) begin
            error("o_mem_rd is X/Z", i_rd_addr);
        end
        prev_rd      = (i_rd === 1'b1);
        prev_rd_addr = i_rd_addr;

        // 2. Apply this cycle's write.
        if (i_wr === 1'b1) begin
            n_wr = n_wr + 1;
            check_addr(1'b1, i_wr_addr, wr_ok);
            if (wr_ok && ^i_wr_be === 1'bx) begin
                error("write byte enables have X/Z bits", i_wr_addr);
                wr_ok = 1'b0;
            end
            if (wr_ok) begin
                if (&i_wr_be)
                    n_wb = n_wb + 1;
                else if (|i_wr_be)
                    n_wt = n_wt + 1;
                else
                    n_wr_empty = n_wr_empty + 1;
                for (k = 0; k < BLOCK_SIZE; k = k + 1)
                    if (i_wr_be[k])
                        mem[(i_wr_addr - BASE_ADDR) / 4 + k / 4][8*(k%4) +: 8]
                            = i_wr_data[8*k +: 8];
                if (p_valid[0] && p_addr[0] == i_wr_addr)
                    n_rd_wr_same = n_rd_wr_same + 1;
                for (k = 1; k < LATENCY; k = k + 1)
                    if (p_valid[k] && p_addr[k] == i_wr_addr)
                        n_wr_inflight = n_wr_inflight + 1;
            end
        end else if (i_wr !== 1'b0 && i_rst !== 1'b1) begin
            error("o_mem_wr is X/Z", i_wr_addr);
        end

        // 3. Answer the request that is due.
        o_rd_valid <= p_valid[LATENCY-1];
        if (!p_valid[LATENCY-1] || !resp_ok[LATENCY-1])
            o_rd_data <= {BW{1'bx}};
        else if (SAMPLE_AT_RESPONSE != 0)
            o_rd_data <= read_block(p_addr[LATENCY-1]);
        else
            o_rd_data <= p_data[LATENCY-1];
    end

    // Reads hex words, one per line, into the array from BASE_ADDR up.
    // Words past the end of the file keep their value. Unlike $readmemh this
    // does not warn when the file is shorter than the array.
    task load_hex;
        input  [8*256-1:0] path;
        output integer     nwords;
        integer     fd;
        integer     r;
        reg  [31:0] w;
        begin
            nwords = 0;
            fd = $fopen(path, "r");
            if (fd == 0) begin
                errors = errors + 1;
                $display("[FAIL] %0s: cannot open %0s", NAME, path);
            end else begin
                r = $fscanf(fd, "%h", w);
                while (r == 1 && nwords < SIZE_WORDS) begin
                    mem[nwords] = w;
                    nwords = nwords + 1;
                    r = $fscanf(fd, "%h", w);
                end
                $fclose(fd);
            end
        end
    endtask

    // Word at a byte address (the low two bits are ignored).
    function [31:0] peek;
        input [31:0] addr;
        begin
            peek = mem[(addr - BASE_ADDR) >> 2];
        end
    endfunction

    task poke;
        input [31:0] addr;
        input [31:0] data;
        begin
            mem[(addr - BASE_ADDR) >> 2] = data;
        end
    endtask

endmodule

`endif
