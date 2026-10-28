`default_nettype none

// One set's tree PLRU state. The cache supplies its valid bits separately;
// invalid ways always take priority over the tree, lowest way first.
module plru #(
    parameter NUM_WAYS = 4,
    parameter WAY_BITS = $clog2((NUM_WAYS > 1) ? NUM_WAYS : 2)
) (
    input  wire                i_clk,
    input  wire                i_rst,
    input  wire [NUM_WAYS-1:0]  i_valid,
    input  wire                i_touch,
    input  wire [WAY_BITS-1:0]  i_touch_way,
    output wire [WAY_BITS-1:0]  o_victim_way
);
    // Pad to a binary tree, but prune subtrees containing no real ways.
    // Thus 3/6/12-way configurations never select a padding leaf.
    localparam LEAVES = 2 ** $clog2(NUM_WAYS);
    localparam [WAY_BITS:0] WAY_COUNT = NUM_WAYS;
    wire [WAY_BITS:0] touch_index = {1'b0, i_touch_way};
    wire [WAY_BITS-1:0] victim [0:2*LEAVES-2];
    wire [NUM_WAYS:0] invalid_found;
    wire [WAY_BITS-1:0] invalid_way [0:NUM_WAYS];

    assign invalid_found[0] = 1'b0;
    assign invalid_way[0] = {WAY_BITS{1'b0}};
    assign o_victim_way = invalid_found[NUM_WAYS] ?
                         invalid_way[NUM_WAYS] : victim[0];

    genvar way;
    generate
        for (way = 0; way < NUM_WAYS; way = way + 1) begin : invalid_scan
            localparam [WAY_BITS-1:0] WAY = way;
            assign invalid_found[way+1] = invalid_found[way] | ~i_valid[way];
            assign invalid_way[way+1] = invalid_found[way] ? invalid_way[way] : WAY;
        end
        for (way = 0; way < LEAVES; way = way + 1) begin : leaves
            localparam [WAY_BITS-1:0] WAY = way;
            assign victim[LEAVES-1+way] = WAY;
        end
    endgenerate

    genvar node;
    generate
        for (node = 0; node < LEAVES-1; node = node + 1) begin : tree
            localparam DEPTH = $clog2(node+2)-1;
            localparam SPAN = LEAVES / (2 ** DEPTH);
            localparam FIRST = (node+1 - (2 ** DEPTH)) * SPAN;
            localparam MIDDLE = FIRST + SPAN/2;
            localparam LIMIT = FIRST + SPAN;
            localparam [WAY_BITS:0] FIRST_INDEX = FIRST;
            localparam [WAY_BITS:0] MIDDLE_INDEX = MIDDLE;
            localparam [WAY_BITS:0] LIMIT_INDEX = LIMIT;
            reg direction;

            // A direction bit points toward the next victim. Touches point
            // away from the used way; merely reading the victim changes none.
            always @(posedge i_clk) begin
                if (i_rst)
                    direction <= 1'b0;
                else if (i_touch && (touch_index < WAY_COUNT) &&
                         (touch_index >= FIRST_INDEX) && (touch_index < LIMIT_INDEX))
                    direction <= (touch_index < MIDDLE_INDEX);
            end

            assign victim[node] = (MIDDLE >= NUM_WAYS) ? victim[2*node+1] :
                                  direction ? victim[2*node+2] : victim[2*node+1];
        end
    endgenerate
endmodule

`default_nettype wire
