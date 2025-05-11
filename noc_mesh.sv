module noc_mesh #(
    parameter ROWS = 2,
    parameter COLS = 2,
    parameter FLIT_WIDTH = 128,
    parameter VC_COUNT = 3
)(
    input  logic clk,
    input  logic reset_n,
    
    // Interface to cores/cache controllers
    input  logic [ROWS*COLS-1:0][FLIT_WIDTH-1:0] local_flit_in,
    input  logic [ROWS*COLS-1:0][VC_COUNT-1:0] local_valid_in,
    output logic [ROWS*COLS-1:0][VC_COUNT-1:0] local_ready_out,
    
    output logic [ROWS*COLS-1:0][FLIT_WIDTH-1:0] local_flit_out,
    output logic [ROWS*COLS-1:0][VC_COUNT-1:0] local_valid_out,
    input  logic [ROWS*COLS-1:0][VC_COUNT-1:0] local_ready_in
);
    // Router port definitions
    localparam NORTH = 0;
    localparam EAST  = 1;
    localparam SOUTH = 2;
    localparam WEST  = 3;
    localparam LOCAL = 4;
    localparam PORTS = 5;
    
    // Inter-router connections (horizontal)
    logic [ROWS-1:0][COLS-1:0][VC_COUNT-1:0][FLIT_WIDTH-1:0] east_west_flit;
    logic [ROWS-1:0][COLS-1:0][VC_COUNT-1:0] east_west_valid;
    logic [ROWS-1:0][COLS-1:0][VC_COUNT-1:0] east_west_ready;
    
    logic [ROWS-1:0][COLS-1:0][VC_COUNT-1:0][FLIT_WIDTH-1:0] west_east_flit;
    logic [ROWS-1:0][COLS-1:0][VC_COUNT-1:0] west_east_valid;
    logic [ROWS-1:0][COLS-1:0][VC_COUNT-1:0] west_east_ready;
    
    // Inter-router connections (vertical)
    logic [ROWS-1:0][COLS-1:0][VC_COUNT-1:0][FLIT_WIDTH-1:0] north_south_flit;
    logic [ROWS-1:0][COLS-1:0][VC_COUNT-1:0] north_south_valid;
    logic [ROWS-1:0][COLS-1:0][VC_COUNT-1:0] north_south_ready;
    
    logic [ROWS-1:0][COLS-1:0][VC_COUNT-1:0][FLIT_WIDTH-1:0] south_north_flit;
    logic [ROWS-1:0][COLS-1:0][VC_COUNT-1:0] south_north_valid;
    logic [ROWS-1:0][COLS-1:0][VC_COUNT-1:0] south_north_ready;
    
    // Instantiate the mesh of routers
    genvar r, c;
    generate
        for (r = 0; r < ROWS; r++) begin : row_gen
            for (c = 0; c < COLS; c++) begin : col_gen
                // Calculate node ID (row-major ordering)
                localparam NODE_ID = r * COLS + c;
                
                // Router input ports
                logic [PORTS-1:0][VC_COUNT-1:0][FLIT_WIDTH-1:0] router_flit_in;
                logic [PORTS-1:0][VC_COUNT-1:0] router_valid_in;
                logic [PORTS-1:0][VC_COUNT-1:0] router_ready_out;
                
                // Router output ports
                logic [PORTS-1:0][VC_COUNT-1:0][FLIT_WIDTH-1:0] router_flit_out;
                logic [PORTS-1:0][VC_COUNT-1:0] router_valid_out;
                logic [PORTS-1:0][VC_COUNT-1:0] router_ready_in;
                
                // Connect local port to network interface
                assign router_flit_in[LOCAL] = '{default: local_flit_in[NODE_ID]};
                assign router_valid_in[LOCAL] = local_valid_in[NODE_ID];
                assign local_ready_out[NODE_ID] = router_ready_out[LOCAL];
                
                assign local_flit_out[NODE_ID] = router_flit_out[LOCAL][0]; // Take first VC
                assign local_valid_out[NODE_ID] = router_valid_out[LOCAL];
                assign router_ready_in[LOCAL] = local_ready_in[NODE_ID];
                
                // Connect North port
                if (r > 0) begin
                    // Connect to router above
                    assign router_flit_in[NORTH] = south_north_flit[r-1][c];
                    assign router_valid_in[NORTH] = south_north_valid[r-1][c];
                    assign north_south_ready[r-1][c] = router_ready_out[NORTH];
                    
                    assign north_south_flit[r-1][c] = router_flit_out[NORTH];
                    assign north_south_valid[r-1][c] = router_valid_out[NORTH];
                    assign router_ready_in[NORTH] = south_north_ready[r-1][c];
                end
                else begin
                    // Boundary condition - no connection
                    assign router_flit_in[NORTH] = '{default: '0};
                    assign router_valid_in[NORTH] = '{default: 1'b0};
                    assign router_ready_in[NORTH] = '{default: 1'b0};
                end
                
                // Connect East port
                if (c < COLS-1) begin
                    // Connect to router to the right
                    assign router_flit_in[EAST] = west_east_flit[r][c+1];
                    assign router_valid_in[EAST] = west_east_valid[r][c+1];
                    assign east_west_ready[r][c+1] = router_ready_out[EAST];
                    
                    assign east_west_flit[r][c+1] = router_flit_out[EAST];
                    assign east_west_valid[r][c+1] = router_valid_out[EAST];
                    assign router_ready_in[EAST] = west_east_ready[r][c+1];
                end
                else begin
                    // Boundary condition - no connection
                    assign router_flit_in[EAST] = '{default: '0};
                    assign router_valid_in[EAST] = '{default: 1'b0};
                    assign router_ready_in[EAST] = '{default: 1'b0};
                end
                
                // Connect South port
                if (r < ROWS-1) begin
                    // Connect to router below
                    assign router_flit_in[SOUTH] = north_south_flit[r+1][c];
                    assign router_valid_in[SOUTH] = north_south_valid[r+1][c];
                    assign south_north_ready[r+1][c] = router_ready_out[SOUTH];
                    
                    assign south_north_flit[r+1][c] = router_flit_out[SOUTH];
                    assign south_north_valid[r+1][c] = router_valid_out[SOUTH];
                    assign router_ready_in[SOUTH] = north_south_ready[r+1][c];
                end
                else begin
                    // Boundary condition - no connection
                    assign router_flit_in[SOUTH] = '{default: '0};
                    assign router_valid_in[SOUTH] = '{default: 1'b0};
                    assign router_ready_in[SOUTH] = '{default: 1'b0};
                end
                
                // Connect West port
                if (c > 0) begin
                    // Connect to router to the left
                    assign router_flit_in[WEST] = east_west_flit[r][c-1];
                    assign router_valid_in[WEST] = east_west_valid[r][c-1];
                    assign west_east_ready[r][c-1] = router_ready_out[WEST];
                    
                    assign west_east_flit[r][c-1] = router_flit_out[WEST];
                    assign west_east_valid[r][c-1] = router_valid_out[WEST];
                    assign router_ready_in[WEST] = east_west_ready[r][c-1];
                end
                else begin
                    // Boundary condition - no connection
                    assign router_flit_in[WEST] = '{default: '0};
                    assign router_valid_in[WEST] = '{default: 1'b0};
                    assign router_ready_in[WEST] = '{default: 1'b0};
                end
                
                // Instantiate the router
                noc_router #(
                    .X_COORD(c),
                    .Y_COORD(r),
                    .NODE_ID(NODE_ID),
                    .FLIT_WIDTH(FLIT_WIDTH),
                    .VC_COUNT(VC_COUNT),
                    .BUFFER_DEPTH(4),
                    .PORTS(PORTS)
                ) router_inst (
                    .clk(clk),
                    .reset_n(reset_n),
                    .flit_in(router_flit_in),
                    .valid_in(router_valid_in),
                    .ready_out(router_ready_out),
                    .flit_out(router_flit_out),
                    .valid_out(router_valid_out),
                    .ready_in(router_ready_in)
                );
            end
        end
    endgenerate
endmodule