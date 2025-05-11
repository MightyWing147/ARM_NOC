module coherence_directory #(
    parameter CORES = 4,
    parameter ADDR_WIDTH = 48,
    parameter LINE_SIZE = 64,
    parameter DIRECTORY_ENTRIES = 8192  // Number of directory entries to track
)(
    input  logic        clk,
    input  logic        reset_n,
    
    // NoC Interface
    output logic [47:0] noc_req_addr,
    output logic [2:0]  noc_req_type,   // Request type
    output logic [7:0]  noc_req_target, // Target node ID
    output logic [63:0] noc_req_data,   // Data if needed
    output logic        noc_req_valid,
    input  logic        noc_req_ready,
    
    input  logic [47:0] noc_resp_addr,
    input  logic [2:0]  noc_resp_type,  // Response type
    input  logic [7:0]  noc_resp_source,// Source node ID
    input  logic [63:0] noc_resp_data,  // Data if needed
    input  logic        noc_resp_valid,
    output logic        noc_resp_ready,
    
    // Stats and debug
    output logic [31:0] dir_hit_count,
    output logic [31:0] dir_miss_count,
    output logic [31:0] dir_eviction_count
);
    // Constants for coherence protocol operations
    localparam REQ_READ          = 3'b000;  // Read request
    localparam REQ_READ_EXCL     = 3'b001;  // Read exclusive request
    localparam REQ_WRITE         = 3'b010;  // Write request
    localparam REQ_INVALIDATE    = 3'b011;  // Invalidate request
    localparam RESP_DATA         = 3'b100;  // Data response
    localparam RESP_DATA_EXCL    = 3'b101;  // Data response with exclusivity
    localparam RESP_ACK          = 3'b110;  // Acknowledgment
    
    // Line address calculation (remove offset bits)
    localparam OFFSET_BITS = $clog2(LINE_SIZE);
    localparam TAG_BITS = ADDR_WIDTH - OFFSET_BITS;
    localparam INDEX_BITS = $clog2(DIRECTORY_ENTRIES);
    localparam DIR_TAG_BITS = TAG_BITS - INDEX_BITS;
    
    // Directory entry definition
    typedef struct packed {
        logic                valid;          // Entry is valid
        logic [DIR_TAG_BITS-1:0] tag;       // Tag bits
        logic [CORES-1:0]   sharers;        // Bit vector of cores sharing this line
        logic [2:0]         owner;          // Owner core ID (for M/O states)
        logic               exclusive;      // Line is in exclusive/modified state
    } dir_entry_t;
    
    // Directory storage
    dir_entry_t directory[DIRECTORY_ENTRIES-1:0];
    
    // Request and response FIFOs for handling NoC traffic
    typedef struct packed {
        logic [47:0] addr;
        logic [2:0]  type_field;
        logic [7:0]  source;
        logic [63:0] data;
    } request_t;
    
    // Request FIFO
    request_t req_fifo[4];      // Small FIFO for pending requests
    logic [1:0] req_fifo_head;  // Head pointer
    logic [1:0] req_fifo_tail;  // Tail pointer
    logic req_fifo_empty;       // FIFO empty flag
    logic req_fifo_full;        // FIFO full flag
    
    // Current request being processed
    request_t current_req;
    logic processing_req;
    
    // Address extraction for current request
    logic [DIR_TAG_BITS-1:0] req_tag;
    logic [INDEX_BITS-1:0] req_index;
    
    // Directory lookup result
    logic dir_hit;
    logic [CORES-1:0] current_sharers;
    logic [2:0] current_owner;
    logic current_exclusive;
    
    // State machine for directory controller
    typedef enum logic [2:0] {
        IDLE,
        LOOKUP,
        SEND_INVALIDATIONS,
        WAIT_ACKS,
        UPDATE_DIRECTORY,
        RESPOND
    } dir_state_t;
    
    dir_state_t state, next_state;
    
    // Counters and tracking
    logic [CORES-1:0] invalidation_targets;
    logic [CORES-1:0] acks_received;
    logic invalidations_complete;
    
    // Extract address components
    function automatic logic [INDEX_BITS-1:0] get_index(input logic [ADDR_WIDTH-1:0] addr);
        return addr[OFFSET_BITS+INDEX_BITS-1:OFFSET_BITS];
    endfunction
    
    function automatic logic [DIR_TAG_BITS-1:0] get_tag(input logic [ADDR_WIDTH-1:0] addr);
        return addr[ADDR_WIDTH-1:OFFSET_BITS+INDEX_BITS];
    endfunction
    
    // FIFO management
    assign req_fifo_empty = (req_fifo_head == req_fifo_tail);
    assign req_fifo_full = ((req_fifo_tail + 1) & 2'b11) == req_fifo_head;
    
    // Directory entry lookup
    assign req_tag = get_tag(current_req.addr);
    assign req_index = get_index(current_req.addr);
    
    // Hit detection
    assign dir_hit = directory[req_index].valid && (directory[req_index].tag == req_tag);
    
    // Current directory entry information
    assign current_sharers = dir_hit ? directory[req_index].sharers : '0;
    assign current_owner = dir_hit ? directory[req_index].owner : '0;
    assign current_exclusive = dir_hit ? directory[req_index].exclusive : 1'b0;
    
    // Check if all invalidation acknowledgments have been received
    assign invalidations_complete = (acks_received == invalidation_targets);
    
    // State machine registers
    always_ff @(posedge clk or negedge reset_n) begin
        if (~reset_n) begin
            state <= IDLE;
        end else begin
            state <= next_state;
        end
    end
    
    // Next state logic
    always_comb begin
        next_state = state;
        
        case (state)
            IDLE: begin
                if (!req_fifo_empty && !processing_req) begin
                    next_state = LOOKUP;
                end
            end
            
            LOOKUP: begin
                if (current_req.type_field == REQ_READ) begin
                    // Simple read request
                    if (current_exclusive && current_owner != current_req.source[2:0]) begin
                        // Need to get data from owner first
                        next_state = SEND_INVALIDATIONS;
                    end else begin
                        // Can directly update directory and respond
                        next_state = UPDATE_DIRECTORY;
                    end
                end
                else if (current_req.type_field == REQ_READ_EXCL) begin
                    // Read exclusive request
                    if (|current_sharers) begin
                        // Need to invalidate all sharers
                        next_state = SEND_INVALIDATIONS;
                    end else begin
                        // No sharers, can grant exclusive access directly
                        next_state = UPDATE_DIRECTORY;
                    end
                end
                else if (current_req.type_field == REQ_WRITE) begin
                    // Write request - need exclusive access
                    if (|current_sharers && current_sharers != (1 << current_req.source[2:0])) begin
                        // Other sharers exist, need to invalidate them
                        next_state = SEND_INVALIDATIONS;
                    end else begin
                        // Only requester has the line or no sharers
                        next_state = UPDATE_DIRECTORY;
                    end
                end
                else if (current_req.type_field == REQ_INVALIDATE) begin
                    // Direct invalidation request
                    if (|current_sharers) begin
                        next_state = SEND_INVALIDATIONS;
                    end else begin
                        next_state = UPDATE_DIRECTORY;
                    end
                end
                else begin
                    // Unknown request type
                    next_state = RESPOND;
                end
            end
            
            SEND_INVALIDATIONS: begin
                // After sending invalidations, wait for acks
                next_state = WAIT_ACKS;
            end
            
            WAIT_ACKS: begin
                if (invalidations_complete) begin
                    next_state = UPDATE_DIRECTORY;
                end
            end
            
            UPDATE_DIRECTORY: begin
                next_state = RESPOND;
            end
            
            RESPOND: begin
                // Go back to IDLE after sending response
                next_state = IDLE;
            end
        endcase
    end
    
    // Main logic for directory operations
    always_ff @(posedge clk or negedge reset_n) begin
        if (~reset_n) begin
            // Reset all registers
            req_fifo_head <= 2'b00;
            req_fifo_tail <= 2'b00;
            processing_req <= 1'b0;
            
            noc_req_valid <= 1'b0;
            noc_resp_ready <= 1'b0;
            
            dir_hit_count <= 32'h0;
            dir_miss_count <= 32'h0;
            dir_eviction_count <= 32'h0;
            
            invalidation_targets <= '0;
            acks_received <= '0;
            
            // Initialize directory entries
            for (int i = 0; i < DIRECTORY_ENTRIES; i++) begin
                directory[i].valid <= 1'b0;
                directory[i].tag <= '0;
                directory[i].sharers <= '0;
                directory[i].owner <= 3'b000;
                directory[i].exclusive <= 1'b0;
            end
        end else begin
            // Default values
            noc_req_valid <= 1'b0;
            noc_resp_ready <= 1'b1; // Always ready to receive responses
            
            // Process incoming requests
            if (noc_resp_valid && noc_resp_ready) begin
                // Enqueue new request if FIFO not full
                if (!req_fifo_full) begin
                    req_fifo[req_fifo_tail].addr <= noc_resp_addr;
                    req_fifo[req_fifo_tail].type_field <= noc_resp_type;
                    req_fifo[req_fifo_tail].source <= noc_resp_source;
                    req_fifo[req_fifo_tail].data <= noc_resp_data;
                    req_fifo_tail <= (req_fifo_tail + 1) & 2'b11;
                end
            end
            
            // Process acknowledgments for invalidations
            if (noc_resp_valid && noc_resp_type == RESP_ACK && state == WAIT_ACKS) begin
                // Record ack from the source
                acks_received[noc_resp_source[2:0]] <= 1'b1;
            end
            
            // State machine logic
            case (state)
                IDLE: begin
                    if (!req_fifo_empty && !processing_req) begin
                        // Start processing a new request
                        current_req <= req_fifo[req_fifo_head];
                        req_fifo_head <= (req_fifo_head + 1) & 2'b11;
                        processing_req <= 1'b1;
                    end
                end
                
                LOOKUP: begin
                    // Update statistics
                    if (dir_hit) begin
                        dir_hit_count <= dir_hit_count + 1;
                    end else begin
                        dir_miss_count <= dir_miss_count + 1;
                    end
                    
                    // Prepare invalidation targets if needed
                    if (current_req.type_field == REQ_READ_EXCL || 
                        current_req.type_field == REQ_WRITE ||
                        current_req.type_field == REQ_INVALIDATE) begin
                        
                        // Invalidate all sharers except requestor
                        invalidation_targets <= current_sharers & ~(1 << current_req.source[2:0]);
                        acks_received <= '0;
                    end
                    else if (current_req.type_field == REQ_READ && current_exclusive) begin
                        // For reads to exclusive lines, only target the owner
                        invalidation_targets <= (1 << current_owner);
                        acks_received <= '0;
                    end
                end
                
                SEND_INVALIDATIONS: begin
                    // Send invalidation requests to all targets
                    if (|invalidation_targets && noc_req_ready) begin
                        noc_req_valid <= 1'b1;
                        noc_req_type <= REQ_INVALIDATE;
                        noc_req_addr <= current_req.addr;
                        
                        // Find first target to invalidate
                        for (int i = 0; i < CORES; i++) begin
                            if (invalidation_targets[i]) begin
                                noc_req_target <= i;
                                // Clear this bit after sending
                                invalidation_targets[i] <= 1'b0;
                                break;
                            end
                        end
                    end
                end
                
                WAIT_ACKS: begin
                    // Just wait for all acks to be received
                    // Logic for tracking acks is handled above
                end
                
                UPDATE_DIRECTORY: begin
                    // Update directory entry based on request type
                    if (!dir_hit) begin
                        // Create new entry for miss
                        directory[req_index].valid <= 1'b1;
                        directory[req_index].tag <= req_tag;
                        directory[req_index].sharers <= '0;
                        directory[req_index].owner <= 3'b000;
                        directory[req_index].exclusive <= 1'b0;
                    end
                    
                    case (current_req.type_field)
                        REQ_READ: begin
                            // Add requestor to sharers list
                            directory[req_index].sharers[current_req.source[2:0]] <= 1'b1;
                            directory[req_index].exclusive <= 1'b0; // No longer exclusive
                        end
                        
                        REQ_READ_EXCL: begin
                            // Clear all sharers and set requestor as exclusive owner
                            directory[req_index].sharers <= (1 << current_req.source[2:0]);
                            directory[req_index].owner <= current_req.source[2:0];
                            directory[req_index].exclusive <= 1'b1;
                        end
                        
                        REQ_WRITE: begin
                            // Set requestor as exclusive owner
                            directory[req_index].sharers <= (1 << current_req.source[2:0]);
                            directory[req_index].owner <= current_req.source[2:0];
                            directory[req_index].exclusive <= 1'b1;
                        end
                        
                        REQ_INVALIDATE: begin
                            // Invalidate the entry or clear specific sharer
                            if (current_req.source[7]) begin
                                // Request came from memory controller - full invalidate
                                directory[req_index].valid <= 1'b0;
                                directory[req_index].sharers <= '0;
                                directory[req_index].exclusive <= 1'b0;
                            end else begin
                                // Remove requester from sharers
                                directory[req_index].sharers[current_req.source[2:0]] <= 1'b0;
                                // If no more sharers, invalidate entry
                                if (directory[req_index].sharers == '0) begin
                                    directory[req_index].valid <= 1'b0;
                                    directory[req_index].exclusive <= 1'b0;
                                end
                            end
                        end
                        
                        default: begin
                            // Unknown request - no directory update
                        end
                    endcase
                end
                
                RESPOND: begin
                    // Send response to the requestor
                    noc_req_valid <= 1'b1;
                    noc_req_addr <= current_req.addr;
                    noc_req_target <= current_req.source;
                    
                    case (current_req.type_field)
                        REQ_READ: begin
                            noc_req_type <= RESP_DATA;
                        end
                        
                        REQ_READ_EXCL: begin
                            noc_req_type <= RESP_DATA_EXCL;
                        end
                        
                        REQ_WRITE, REQ_INVALIDATE: begin
                            noc_req_type <= RESP_ACK;
                        end
                        
                        default: begin
                            noc_req_type <= RESP_ACK;
                        end
                    endcase
                    
                    // When response is sent, clear processing flag
                    if (noc_req_ready) begin
                        processing_req <= 1'b0;
                    end
                end
            endcase
        end
    end
endmodule