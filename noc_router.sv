module noc_router #(
    parameter X_COORD = 0,
    parameter Y_COORD = 0,
    parameter NODE_ID = 0,
    parameter FLIT_WIDTH = 128,
    parameter VC_COUNT = 3,
    parameter BUFFER_DEPTH = 4,
    parameter PORTS = 5   // N, E, S, W, Local
)(
    input  logic clk,
    input  logic reset_n,
    
    // Input ports from neighbors
    input  logic [PORTS-1:0][VC_COUNT-1:0][FLIT_WIDTH-1:0] flit_in,
    input  logic [PORTS-1:0][VC_COUNT-1:0] valid_in,
    output logic [PORTS-1:0][VC_COUNT-1:0] ready_out,
    
    // Output ports to neighbors
    output logic [PORTS-1:0][VC_COUNT-1:0][FLIT_WIDTH-1:0] flit_out,
    output logic [PORTS-1:0][VC_COUNT-1:0] valid_out,
    input  logic [PORTS-1:0][VC_COUNT-1:0] ready_in
);
    // Port definitions
    localparam NORTH = 0;
    localparam EAST  = 1;
    localparam SOUTH = 2;
    localparam WEST  = 3;
    localparam LOCAL = 4;
    
    // Packet structure for routing decisions
    typedef struct packed {
        logic [7:0] dest_id;
        logic [7:0] source_id;
        logic [7:0] msg_type;
        logic [47:0] addr;        // Address field starts at bit position 16
        logic [63:0] data;
    } packet_t;
    
    // Internal buffers (one per input port per VC)
    logic [PORTS-1:0][VC_COUNT-1:0][BUFFER_DEPTH-1:0][FLIT_WIDTH-1:0] buffers;
    logic [PORTS-1:0][VC_COUNT-1:0][$clog2(BUFFER_DEPTH):0] buffer_count;
    
    // Buffer read/write pointers
    logic [PORTS-1:0][VC_COUNT-1:0][$clog2(BUFFER_DEPTH)-1:0] write_ptr;
    logic [PORTS-1:0][VC_COUNT-1:0][$clog2(BUFFER_DEPTH)-1:0] read_ptr;
    
    // Route computation results
    logic [PORTS-1:0][VC_COUNT-1:0][2:0] output_port;
    logic [PORTS-1:0][VC_COUNT-1:0] route_valid;
    
    // Virtual channel allocation
    logic [PORTS-1:0][VC_COUNT-1:0] vc_allocated;
    logic [PORTS-1:0][VC_COUNT-1:0][VC_COUNT-1:0] vc_alloc;
    
    // Switch allocation
    logic [PORTS-1:0][VC_COUNT-1:0][PORTS-1:0] switch_alloc;
    logic [PORTS-1:0][VC_COUNT-1:0] switch_allocated;
    
    // Destination ID to X,Y coordinate mapping
    // Simple row-major mapping
    function automatic logic [3:0] get_x_coord(input logic [7:0] dest_id);
        return dest_id[3:0]; // Assuming 4 bits for X dimension
    endfunction
    
    function automatic logic [3:0] get_y_coord(input logic [7:0] dest_id);
        return dest_id[7:4]; // Assuming 4 bits for Y dimension
    endfunction
    
    // XY routing algorithm
    function automatic logic [2:0] compute_output_port(input logic [7:0] dest_id);
        logic [3:0] dest_x, dest_y;
        dest_x = get_x_coord(dest_id);
        dest_y = get_y_coord(dest_id);
        
        // Check if packet has arrived at destination
        if (dest_x == X_COORD && dest_y == Y_COORD)
            return LOCAL;
            
        // X-first routing
        if (dest_x > X_COORD)
            return EAST;
        else if (dest_x < X_COORD)
            return WEST;
        else if (dest_y > Y_COORD)
            return NORTH;
        else
            return SOUTH;
    endfunction
    
    // Input buffer management
    always_ff @(posedge clk or negedge reset_n) begin
        if (~reset_n) begin
            buffers <= '0;
            buffer_count <= '0;
            write_ptr <= '0;
            read_ptr <= '0;
            ready_out <= '1; // Ready to receive initially
        end
        else begin
            // Process all input ports and VCs
            for (int port = 0; port < PORTS; port++) begin
                for (int vc = 0; vc < VC_COUNT; vc++) begin
                    // Write to buffer if input is valid and we have space
                    if (valid_in[port][vc] && buffer_count[port][vc] < BUFFER_DEPTH) begin
                        buffers[port][vc][write_ptr[port][vc]] <= flit_in[port][vc];
                        write_ptr[port][vc] <= (write_ptr[port][vc] + 1) % BUFFER_DEPTH;
                        buffer_count[port][vc] <= buffer_count[port][vc] + 1;
                    end
                    
                    // Read from buffer if switch allocation succeeded and downstream is ready
                    if (switch_allocated[port][vc] && buffer_count[port][vc] > 0) begin
                        // Determine which output port was allocated
                        for (int out_port = 0; out_port < PORTS; out_port++) begin
                            if (switch_alloc[port][vc][out_port]) begin
                                // Check if downstream router/port is ready
                                if (ready_in[out_port][vc]) begin
                                    read_ptr[port][vc] <= (read_ptr[port][vc] + 1) % BUFFER_DEPTH;
                                    buffer_count[port][vc] <= buffer_count[port][vc] - 1;
                                end
                            end
                        end
                    end
                    
                    // Update ready signal based on buffer space
                    ready_out[port][vc] <= (buffer_count[port][vc] < BUFFER_DEPTH);
                end
            end
        end
    end
    
    // Route computation stage
    always_ff @(posedge clk or negedge reset_n) begin
        if (~reset_n) begin
            output_port <= '0;
            route_valid <= '0;
        end
        else begin
            for (int port = 0; port < PORTS; port++) begin
                for (int vc = 0; vc < VC_COUNT; vc++) begin
                    if (buffer_count[port][vc] > 0 && !route_valid[port][vc]) begin
                        // Extract packet from buffer for routing
                        packet_t packet;
                        packet = buffers[port][vc][read_ptr[port][vc]];
                        
                        // Compute output port using XY routing
                        output_port[port][vc] <= compute_output_port(packet.dest_id);
                        route_valid[port][vc] <= 1'b1;
                    end
                    else if (buffer_count[port][vc] == 0) begin
                        // No packet in buffer, invalidate route
                        route_valid[port][vc] <= 1'b0;
                    end
                end
            end
        end
    end
    
    // Virtual channel allocation (simplified)
    // In a full implementation, this would handle dependencies and deadlock avoidance
    always_ff @(posedge clk or negedge reset_n) begin
        if (~reset_n) begin
            vc_alloc <= '0;
            vc_allocated <= '0;
        end
        else begin
            // Simple allocation: each VC maps to same VC on output port
            // In reality, this would be more complex with arbitration
            for (int port = 0; port < PORTS; port++) begin
                for (int vc = 0; vc < VC_COUNT; vc++) begin
                    if (route_valid[port][vc] && !vc_allocated[port][vc]) begin
                        vc_alloc[port][vc] <= (1 << vc); // Same VC number
                        vc_allocated[port][vc] <= 1'b1;
                    end
                    else if (!route_valid[port][vc]) begin
                        vc_allocated[port][vc] <= 1'b0;
                    end
                end
            end
        end
    end
    
    // Switch allocation (simplified round-robin)
    logic [PORTS-1:0][2:0] round_robin_counter;
    
    always_ff @(posedge clk or negedge reset_n) begin
        if (~reset_n) begin
            switch_alloc <= '0;
            switch_allocated <= '0;
            round_robin_counter <= '0;
        end
        else begin
            // First, clear allocations from previous cycle
            switch_alloc <= '0;
            switch_allocated <= '0;
            
            // For each output port
            for (int out_port = 0; out_port < PORTS; out_port++) begin
                logic allocated;
                allocated = 1'b0;
                
                // Start from the priority position and check all ports/VCs
                for (int i = 0; i < PORTS && !allocated; i++) begin
                    int port;
                    port = (round_robin_counter[out_port] + i) % PORTS;
                    
                    for (int vc = 0; vc < VC_COUNT && !allocated; vc++) begin
                        if (route_valid[port][vc] && vc_allocated[port][vc] && 
                            output_port[port][vc] == out_port) begin
                            // Allocate switch
                            switch_alloc[port][vc][out_port] <= 1'b1;
                            switch_allocated[port][vc] <= 1'b1;
                            allocated = 1'b1;
                            
                            // Update round-robin counter
                            round_robin_counter[out_port] <= (port + 1) % PORTS;
                        end
                    end
                end
            end
        end
    end
    
    // Output stage
    always_ff @(posedge clk or negedge reset_n) begin
        if (~reset_n) begin
            flit_out <= '0;
            valid_out <= '0;
        end
        else begin
            // Default all outputs to invalid
            valid_out <= '0;
            
            // For each input port and VC
            for (int port = 0; port < PORTS; port++) begin
                for (int vc = 0; vc < VC_COUNT; vc++) begin
                    if (switch_allocated[port][vc] && buffer_count[port][vc] > 0) begin
                        // Determine which output port was allocated
                        for (int out_port = 0; out_port < PORTS; out_port++) begin
                            if (switch_alloc[port][vc][out_port]) begin
                                // Forward flit to output
                                flit_out[out_port][vc] <= buffers[port][vc][read_ptr[port][vc]];
                                valid_out[out_port][vc] <= 1'b1;
                            end
                        end
                    end
                end
            end
        end
    end
endmodule