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

// Write your code here.

endmodule

`default_nettype wire
