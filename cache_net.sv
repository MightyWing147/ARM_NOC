module l1dcache_coherent(
    input  logic clk,
    input  logic reset_n,
    
    // CPU interface (unchanged)
    input  logic [47:0] cpu_addr,
    input  logic cpu_read,
    input  logic cpu_write,
    input  logic [63:0] cpu_wdata,
    input  logic [7:0] cpu_byte_en,
    output logic [63:0] cpu_rdata,
    output logic cpu_ready,
    
    // Network Interface connection
    output logic [47:0] ni_req_addr,
    output logic [63:0] ni_req_data,
    output logic [7:0] ni_req_type,
    output logic ni_req_valid,
    input  logic ni_req_ready,
    
    input  logic [63:0] ni_resp_data,
    input  logic [7:0] ni_resp_type,
    input  logic ni_resp_valid,
    output logic ni_resp_ready,
    
    // Coherence interface
    output logic [47:0] coh_req_addr,
    output logic [2:0] coh_req_type,
    output logic [7:0] coh_req_target,
    output logic coh_req_valid,
    input  logic coh_req_ready,
    
    input  logic [47:0] coh_resp_addr,
    input  logic [2:0] coh_resp_type,
    input  logic [7:0] coh_resp_source,
    input  logic [63:0] coh_resp_data,
    input  logic coh_resp_valid,
    output logic coh_resp_ready
);
    // Cache configuration (from your existing design)
    localparam int CACHE_SIZE = 256 * 1024;
    localparam int LINE_SIZE = 64;
    localparam int ASSOCIATIVITY = 8;
    localparam int SETS = CACHE_SIZE/(LINE_SIZE * ASSOCIATIVITY);
    
    localparam int OFFSET_BITS = $clog2(LINE_SIZE);
    localparam int INDEX_BITS = $clog2(SETS);
    localparam int TAG_BITS = 48 - INDEX_BITS - OFFSET_BITS;
    
    // MOESI state encoding for cache coherence
    localparam logic [2:0] INVALID   = 3'b000;
    localparam logic [2:0] SHARED    = 3'b001;
    localparam logic [2:0] EXCLUSIVE = 3'b010;
    localparam logic [2:0] OWNED     = 3'b011;
    localparam logic [2:0] MODIFIED  = 3'b100;
    
    // Extract address components
    logic [TAG_BITS-1:0] tag;
    logic [INDEX_BITS-1:0] index;
    logic [OFFSET_BITS-1:0] offset;
    
    assign tag = cpu_addr[47:OFFSET_BITS+INDEX_BITS];
    assign index = cpu_addr[OFFSET_BITS+INDEX_BITS-1:OFFSET_BITS];
    assign offset = cpu_addr[OFFSET_BITS-1:0];
    
    // Modified cache line structure with coherence state
    typedef struct packed {
        logic valid;
        logic [2:0] state;  // MOESI state
        logic [TAG_BITS-1:0] tag;
        logic [LINE_SIZE*8-1:0] data;
    } cache_line_t;
    
    // Cache storage
    cache_line_t cache_mem [SETS-1:0][ASSOCIATIVITY-1:0];
    logic [2:0] lru_counter [SETS-1:0][ASSOCIATIVITY-1:0];
    
    // Message type definitions for NoC
    localparam REQ_READ         = 8'h01;
    localparam REQ_WRITE        = 8'h02;
    localparam REQ_FETCH        = 8'h03;
    localparam RESP_READ_DATA   = 8'h81;
    localparam RESP_WRITE_ACK   = 8'h82;
    localparam RESP_FETCH_DATA  = 8'h83;
    
    // Coherence message types
    localparam COH_INVALIDATE   = 3'b001;
    localparam COH_SHARED_REQ   = 3'b010;
    localparam COH_EXCL_REQ     = 3'b011;
    localparam COH_WB_REQ       = 3'b100;
    localparam COH_INV_ACK      = 3'b101;
    localparam COH_DATA_RESP    = 3'b110;
    
    // Internal signals
    logic [ASSOCIATIVITY-1:0] hit_way;
    logic cache_hit;
    logic [2:0] replace_way;
    
    // Cache hit detection
    always_comb begin
        hit_way = '0;
        cache_hit = 1'b0;
        
        for (int i = 0; i < ASSOCIATIVITY; i++) begin
            if (cache_mem[index][i].valid && cache_mem[index][i].tag == tag && 
                cache_mem[index][i].state != INVALID) begin
                hit_way[i] = 1'b1;
                cache_hit = 1'b1;
            end
        end
    end
    
    // Find LRU way for replacement
    always_comb begin
        replace_way = '0;
        for (int i = 0; i < ASSOCIATIVITY; i++) begin
            if (lru_counter[index][i] == '0) begin
                replace_way = i[2:0];
                break;
            end
        end
    end
    
    // Cache controller state machine with coherence
    typedef enum logic [3:0] {
        IDLE,
        TAG_CHECK,
        WRITE_BACK,
        FETCH_LINE,
        UPGRADE_REQ,
        UPDATE_CACHE,
        HANDLE_INVALIDATE,
        HANDLE_SHARED_REQ,
        HANDLE_EXCLUSIVE_REQ,
        SEND_RESPONSE,
        WAIT_RESPONSE,
        COMPLETE
    } cache_state_t;
    
    cache_state_t current_state, next_state;
    
    // Track hit way or replacement way
    logic [2:0] current_way;
    
    // State machine
    always_ff @(posedge clk or negedge reset_n) begin
        if (~reset_n) begin
            current_state <= IDLE;
            current_way <= '0;
            
            // Initialize cache and LRU
            for (int set = 0; set < SETS; set++) begin
                for (int w = 0; w < ASSOCIATIVITY; w++) begin
                    cache_mem[set][w].valid <= 1'b0;
                    cache_mem[set][w].state <= INVALID;
                    cache_mem[set][w].tag <= '0;
                    cache_mem[set][w].data <= '0;
                    lru_counter[set][w] <= w[2:0];  // Initialize LRU counters
                end
            end
        end
        else begin
            current_state <= next_state;
            
            // Save the current way when we have a hit or decide on replacement
            if (current_state == TAG_CHECK) begin
                if (cache_hit) begin
                    // Find the way that hit
                    for (int i = 0; i < ASSOCIATIVITY; i++) begin
                        if (hit_way[i]) begin
                            current_way <= i[2:0];
                            break;
                        end
                    end
                end
                else begin
                    // Use replacement way
                    current_way <= replace_way;
                end
            end
            
            // Update cache line on completion
            if (current_state == UPDATE_CACHE) begin
                // Update cache line with fetched data or write data
                if (ni_resp_valid && (ni_resp_type == RESP_READ_DATA || 
                                      ni_resp_type == RESP_FETCH_DATA)) begin
                    // Update cache line with fetched data
                    cache_mem[index][current_way].valid <= 1'b1;
                    cache_mem[index][current_way].tag <= tag;
                    
                    // Determine state based on coherence response
                    if (ni_resp_type == RESP_READ_DATA) begin
                        // Shared or exclusive based on response
                        cache_mem[index][current_way].state <= SHARED;
                    end
                    else begin
                        // Exclusive for instruction fetch
                        cache_mem[index][current_way].state <= EXCLUSIVE;
                    end
                    
                    // Update data (this would actually be more complex to handle partial line)
                    cache_mem[index][current_way].data <= ni_resp_data;
                end
                else if (cpu_write && cache_hit) begin
                    // Apply write to cache line
                    for (int i = 0; i < 8; i++) begin
                        if (cpu_byte_en[i]) begin
                            cache_mem[index][current_way].data[offset*8 + i*8 +: 8] <= cpu_wdata[i*8 +: 8];
                        end
                    end
                    
                    // Update state to modified
                    cache_mem[index][current_way].state <= MODIFIED;
                end
            end
            
            // Handle coherence operations
            if (current_state == HANDLE_INVALIDATE) begin
                // Find the matching cache line
                for (int i = 0; i < ASSOCIATIVITY; i++) begin
                    if (cache_mem[index][i].valid && cache_mem[index][i].tag == tag) begin
                        // Invalidate the line
                        cache_mem[index][i].state <= INVALID;
                        break;
                    end
                end
            end
            else if (current_state == HANDLE_SHARED_REQ) begin
                // Find the matching cache line
                for (int i = 0; i < ASSOCIATIVITY; i++) begin
                    if (cache_mem[index][i].valid && cache_mem[index][i].tag == tag) begin
                        // Downgrade from exclusive/modified to shared if necessary
                        if (cache_mem[index][i].state == EXCLUSIVE || 
                            cache_mem[index][i].state == MODIFIED) begin
                            cache_mem[index][i].state <= SHARED;
                        end
                        break;
                    end
                end
            end
            
            // Update LRU counters on hit or update
            if ((current_state == TAG_CHECK && cache_hit) || 
                current_state == UPDATE_CACHE) begin
                for (int w = 0; w < ASSOCIATIVITY; w++) begin
                    if (w == current_way) begin
                        // Move accessed way to MRU
                        lru_counter[index][w] <= '1;
                    end
                    else if (lru_counter[index][w] > '0) begin
                        // Decrement other counters
                        lru_counter[index][w] <= lru_counter[index][w] - 1'b1;
                    end
                end
            end
        end
    end
    
    // Next state logic
    always_comb begin
        next_state = current_state;
        
        // Default values for outputs
        cpu_rdata = '0;
        cpu_ready = 1'b0;
        
        ni_req_addr = '0;
        ni_req_data = '0;
        ni_req_type = '0;
        ni_req_valid = 1'b0;
        ni_resp_ready = 1'b0;
        
        coh_req_addr = '0;
        coh_req_type = '0;
        coh_req_target = '0;
        coh_req_valid = 1'b0;
        coh_resp_ready = 1'b0;
        
        case (current_state)
            IDLE: begin
                cpu_ready = 1'b1;
                
                if (cpu_read || cpu_write) begin
                    next_state = TAG_CHECK;
                end
                else if (coh_resp_valid) begin
                    // Handle incoming coherence message
                    if (coh_resp_type == COH_INVALIDATE) begin
                        next_state = HANDLE_INVALIDATE;
                    end
                    else if (coh_resp_type == COH_SHARED_REQ) begin
                        next_state = HANDLE_SHARED_REQ;
                    end
                    else if (coh_resp_type == COH_EXCL_REQ) begin
                        next_state = HANDLE_EXCLUSIVE_REQ;
                    end
                    else begin
                        next_state = IDLE;
                    end
                end
            end
            
            TAG_CHECK: begin
                if (cache_hit) begin
                    if (cpu_read) begin
                        // Read hit - deliver data
                        cpu_rdata = cache_mem[index][current_way].data[offset*8 +: 64];
                        cpu_ready = 1'b1;
                        next_state = COMPLETE;
                    end
                    else if (cpu_write) begin
                        // Write hit - check if we have write permissions
                        if (cache_mem[index][current_way].state == MODIFIED || 
                            cache_mem[index][current_way].state == EXCLUSIVE) begin
                            // Have write permission, update cache
                            next_state = UPDATE_CACHE;
                        end
                        else begin
                            // Need to request upgrade to exclusive
                            next_state = UPGRADE_REQ;
                        end
                    end
                end
                else begin
                    // Cache miss - check if need write-back first
                    if (cache_mem[index][replace_way].valid && 
                        (cache_mem[index][replace_way].state == MODIFIED || 
                         cache_mem[index][replace_way].state == OWNED)) begin
                        next_state = WRITE_BACK;
                    end
                    else begin
                        next_state = FETCH_LINE;
                    end
                end
            end
            
            WRITE_BACK: begin
                // Send write-back request through NoC
                ni_req_addr = {cache_mem[index][replace_way].tag, index, {OFFSET_BITS{1'b0}}};
                ni_req_data = cache_mem[index][replace_way].data[0 +: 64]; // First word (simplification)
                ni_req_type = REQ_WRITE;
                ni_req_valid = 1'b1;
                
                if (ni_req_ready) begin
                    next_state = FETCH_LINE;
                end
            end
            
            FETCH_LINE: begin
                // Send read request through NoC
                ni_req_addr = cpu_addr;
                ni_req_type = REQ_READ;
                ni_req_valid = 1'b1;
                
                if (ni_req_ready) begin
                    next_state = WAIT_RESPONSE;
                end
            end
            
            UPGRADE_REQ: begin
                // Send coherence request for exclusive access
                coh_req_addr = cpu_addr;
                coh_req_type = COH_EXCL_REQ;
                coh_req_target = 8'hFF; // Special value for directory
                coh_req_valid = 1'b1;
                
                if (coh_req_ready) begin
                    next_state = WAIT_RESPONSE;
                end
            end
            
            WAIT_RESPONSE: begin
                ni_resp_ready = 1'b1;
                
                if (ni_resp_valid) begin
                    next_state = UPDATE_CACHE;
                end
            end
            
            UPDATE_CACHE: begin
                // Cache line updated in sequential logic
                cpu_rdata = ni_resp_data; // For reads
                cpu_ready = 1'b1;
                next_state = COMPLETE;
            end
            
            HANDLE_INVALIDATE: begin
                // Cache line invalidated in sequential logic
                coh_resp_ready = 1'b1;
                
                // Send acknowledgment
                coh_req_addr = coh_resp_addr;
                coh_req_type = COH_INV_ACK;
                coh_req_target = coh_resp_source;
                coh_req_valid = 1'b1;
                
                if (coh_req_ready) begin
                    next_state = IDLE;
                end
            end
            
            HANDLE_SHARED_REQ: begin
                // Update state in sequential logic
                coh_resp_ready = 1'b1;
                
                // Send response with data if modified
                coh_req_addr = coh_resp_addr;
                coh_req_type = COH_DATA_RESP;
                coh_req_target = coh_resp_source;
                coh_req_valid = 1'b1;
                
                if (coh_req_ready) begin
                    next_state = IDLE;
                end
            end
            
            HANDLE_EXCLUSIVE_REQ: begin
                // Must invalidate and possibly send data
                coh_resp_ready = 1'b1;
                
                // Send response with data if modified
                coh_req_addr = coh_resp_addr;
                coh_req_type = COH_DATA_RESP;
                coh_req_target = coh_resp_source;
                coh_req_valid = 1'b1;
                
                if (coh_req_ready) begin
                    next_state = IDLE;
                end
            end
            
            COMPLETE: begin
                next_state = IDLE;
            end
            
            default: next_state = IDLE;
        endcase
    end
endmodule