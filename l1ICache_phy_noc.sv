module l1icache(
    input  logic        clk,
    input  logic        reset_n,                  // Active low reset
    input  logic        cpu_req_valid,            // CPU instruction request
    input  logic [47:0] cpu_req_addr,             // Physical Address from CPU
    input  logic        flush,                    // Trigger to flush the cache
    
    // Memory/NoC data interface
    input  logic        mem_resp_valid,           // Data from Memory/NoC available
    input  logic [63:0] mem_resp_data[0:7],       // Instruction data from memory/NoC
    input  logic        mem_resp_last,            // Last data transfer flag
    
    // CPU interface outputs
    output logic        cpu_resp_valid,           // Response to CPU valid
    output logic [31:0] cpu_resp_data,            // 32-bit instruction to CPU
    output logic        cpu_resp_ready,           // Ready to accept new CPU requests
    
    // Memory/NoC request interface
    output logic        mem_req_valid,            // Request to memory/NoC
    output logic [47:0] mem_req_addr,             // Physical address to memory/NoC
    output logic        mem_req_ready,            // Ready to receive mem responses
    
    // NoC coherence interface inputs
    input  logic        noc_req_valid,            // Incoming coherence request valid
    input  logic [2:0]  noc_req_type,             // Request type (read, write, invalidate)
    input  logic [47:0] noc_req_addr,             // Request address
    input  logic [2:0]  noc_req_source,           // Source core/cache ID
    input  logic [2:0]  noc_req_dest,             // Destination (only needed for multicast)
    input  logic [63:0] noc_req_data,             // Data if needed
    
    // NoC coherence interface outputs
    output logic        noc_resp_valid,           // Outgoing coherence response valid
    output logic [2:0]  noc_resp_type,            // Response type (data, ack, etc.)
    output logic [47:0] noc_resp_addr,            // Response address
    output logic [2:0]  noc_resp_dest,            // Destination core/cache ID
    output logic [63:0] noc_resp_data,            // Response data if needed
    
    // Directory interface (optional, depends on implementation)
    input  logic        dir_resp_valid,           // Directory response valid
    input  logic [7:0]  dir_resp_sharers,         // Bit vector of sharers
    input  logic [2:0]  dir_resp_owner,           // Owner core ID
    output logic        dir_req_valid,            // Directory request valid
    output logic [47:0] dir_req_addr,             // Directory request address
    output logic [2:0]  dir_req_type,             // Directory request type
    
    // Configuration
    input  logic [2:0]  core_id,                  // Core ID for this cache
    input  logic        exclusive_inst_fetch      // When 1, get exclusive access for fetched lines
);
    // Cache config
    localparam int CACHE_SIZE     = 128 * 1024;   // 128KB L1 I-Cache
    localparam int LINE_SIZE      = 64;           // 64B cache line size
    localparam int ASSOCIATIVITY  = 8;            // 8-way set associative
    localparam int SETS           = CACHE_SIZE/(LINE_SIZE * ASSOCIATIVITY);
    
    localparam int OFFSET_BITS    = $clog2(LINE_SIZE);
    localparam int INDEX_BITS     = $clog2(SETS);
    localparam int TAG_BITS       = 48 - INDEX_BITS - OFFSET_BITS;
    localparam int WORD_OFFSET_BITS = $clog2(LINE_SIZE/4);  // Word (4B) offset bits
    
    // MOESI state encoding
    localparam logic [2:0] INVALID   = 3'b000;
    localparam logic [2:0] SHARED    = 3'b001;
    localparam logic [2:0] EXCLUSIVE = 3'b010;
    localparam logic [2:0] OWNED     = 3'b011;
    localparam logic [2:0] MODIFIED  = 3'b100;
    
    // NoC Message Types
    localparam logic [2:0] NOC_READ_REQ       = 3'b000;  // Read request
    localparam logic [2:0] NOC_READ_EXCL_REQ  = 3'b001;  // Read exclusive request
    localparam logic [2:0] NOC_WRITE_REQ      = 3'b010;  // Write request (rare for I-cache)
    localparam logic [2:0] NOC_INVALIDATE_REQ = 3'b011;  // Invalidate request
    localparam logic [2:0] NOC_DATA_RESP      = 3'b100;  // Data response
    localparam logic [2:0] NOC_DATA_EXCL_RESP = 3'b101;  // Data response with exclusivity
    localparam logic [2:0] NOC_ACK_RESP       = 3'b110;  // Acknowledgment
    
    // Address components extraction
    logic [INDEX_BITS-1:0] cpu_index;
    logic [OFFSET_BITS-3:0] cpu_word_offset; // Word offset (4B words)
    logic [TAG_BITS-1:0] cpu_tag;
    
    // Extract address components from CPU address
    assign cpu_index = cpu_req_addr[OFFSET_BITS+INDEX_BITS-1:OFFSET_BITS];
    assign cpu_word_offset = cpu_req_addr[OFFSET_BITS-1:2]; // 4B words
    assign cpu_tag = cpu_req_addr[47:OFFSET_BITS+INDEX_BITS];
    
    // Same for NoC request address
    logic [INDEX_BITS-1:0] noc_index;
    logic [OFFSET_BITS-3:0] noc_word_offset;
    logic [TAG_BITS-1:0] noc_tag;
    
    assign noc_index = noc_req_addr[OFFSET_BITS+INDEX_BITS-1:OFFSET_BITS];
    assign noc_word_offset = noc_req_addr[OFFSET_BITS-1:2];
    assign noc_tag = noc_req_addr[47:OFFSET_BITS+INDEX_BITS];
    
    // Cache line definition with coherence state
    typedef struct packed {
        logic valid;
        logic [2:0] coherence_state;   // MOESI state
        logic [TAG_BITS-1:0] tag;
        logic [LINE_SIZE*8-1:0] data;  // 64 bytes = 512 bits of data
    } cache_line_t;
    
    // Cache storage
    cache_line_t cache_mem[SETS-1:0][ASSOCIATIVITY-1:0];
    
    // LRU tracking - use 4 bits for up to 16-way associativity
    // Each entry counts from 0 (LRU) to ASSOCIATIVITY-1 (MRU)
    logic [3:0] lru_counter[SETS-1:0][ASSOCIATIVITY-1:0];
    
    // Hit detection logic
    logic [ASSOCIATIVITY-1:0] cpu_hit_way;  // One-hot encoding of hit way
    logic cpu_cache_hit;                    // Overall hit signal
    logic [3:0] cpu_hit_way_idx;            // Binary index of hit way
    
    // Replace way selection
    logic [3:0] replace_way_idx;            // Way to replace on miss
    
    // Coherence transaction tracking
    logic [2:0] pending_coherence_state;    // State to apply after transaction
    logic [7:0] expected_acks;              // Number of acks expected
    logic [7:0] received_acks;              // Number of acks received
    logic all_acks_received;                // Flag when all acks received
    
    // Line refill tracking
    logic [2:0] refill_word_count;          // Track words during refill
    logic [LINE_SIZE*8-1:0] refill_buffer;  // Buffer for line being filled
    
    // Selected data from cache
    logic [31:0] cache_word[ASSOCIATIVITY-1:0]; // Output word from each way
    logic [31:0] selected_word;                 // Selected output word
    
    // Priority control
    logic noc_has_priority;  // NoC request has priority over CPU
    
    // State machines for CPU and NoC operations
    typedef enum logic [2:0] {
        CPU_IDLE,
        CPU_TAG_CHECK,
        CPU_REFILL_REQUEST,
        CPU_REFILL_WAIT,
        CPU_COHERENCE_WAIT,
        CPU_COMPLETE
    } cpu_state_t;
    
    typedef enum logic [2:0] {
        NOC_IDLE,
        NOC_PROCESS_REQUEST,
        NOC_WAIT_ACKS,
        NOC_SEND_RESPONSE,
        NOC_UPDATE_STATE
    } noc_state_t;
    
    // State registers
    cpu_state_t cpu_state, cpu_next_state;
    noc_state_t noc_state, noc_next_state;
    
    // Snoop hit detection
    logic [ASSOCIATIVITY-1:0] noc_hit_way;  // Hit way for NoC request
    logic noc_cache_hit;                    // Hit on NoC request
    logic [3:0] noc_hit_way_idx;            // Binary index of hit way
    
    // Hit detection logic for CPU requests
    genvar way;
    generate
        for (way = 0; way < ASSOCIATIVITY; way++) begin : hit_logic
            assign cpu_hit_way[way] = cache_mem[cpu_index][way].valid && 
                                     (cache_mem[cpu_index][way].tag == cpu_tag) && 
                                     (cache_mem[cpu_index][way].coherence_state != INVALID);
                                     
            // Extract the appropriate 32-bit word from each cache line
            assign cache_word[way] = cache_mem[cpu_index][way].data[cpu_word_offset*32 +: 32];
            
            // Same for NoC requests
            assign noc_hit_way[way] = cache_mem[noc_index][way].valid && 
                                     (cache_mem[noc_index][way].tag == noc_tag) && 
                                     (cache_mem[noc_index][way].coherence_state != INVALID);
        end
    endgenerate
    
    // Overall hit detection
    assign cpu_cache_hit = |cpu_hit_way && cpu_req_valid;
    assign noc_cache_hit = |noc_hit_way && noc_req_valid;
    
    // Priority control - NoC requests have priority over CPU by default
    // This prevents coherence protocol deadlocks
    assign noc_has_priority = noc_req_valid;
    
    // Convert one-hot hit way to binary index
    always_comb begin
        cpu_hit_way_idx = '0;
        noc_hit_way_idx = '0;
        
        for (int i = 0; i < ASSOCIATIVITY; i++) begin
            if (cpu_hit_way[i]) cpu_hit_way_idx = i[3:0];
            if (noc_hit_way[i]) noc_hit_way_idx = i[3:0];
        end
    end
    
    // Find LRU way for replacement
    always_comb begin
        replace_way_idx = '0;
        for (int i = 0; i < ASSOCIATIVITY; i++) begin
            if (lru_counter[cpu_index][i] == '0) begin
                replace_way_idx = i[3:0];
                break;
            end
        end
    end
    
    // Select output word from the hit way
    always_comb begin
        selected_word = '0;
        for (int i = 0; i < ASSOCIATIVITY; i++) begin
            if (cpu_hit_way[i]) begin
                selected_word = cache_word[i];
                break;
            end
        end
    end
    
    // All acknowledgments received check
    assign all_acks_received = (received_acks >= expected_acks);
    
    // CPU State Machine
    always_ff @(posedge clk or negedge reset_n) begin
        if (~reset_n) begin
            cpu_state <= CPU_IDLE;
        end else if (flush) begin
            cpu_state <= CPU_IDLE;
        end else begin
            cpu_state <= cpu_next_state;
        end
    end
    
    // CPU Next State Logic
    always_comb begin
        cpu_next_state = cpu_state;
        
        case (cpu_state)
            CPU_IDLE: begin
                if (cpu_req_valid && !noc_has_priority) begin
                    cpu_next_state = CPU_TAG_CHECK;
                end
            end
            
            CPU_TAG_CHECK: begin
                if (cpu_cache_hit) begin
                    // For instruction fetch, most coherence protocols allow 
                    // reading in any valid state without state change
                    cpu_next_state = CPU_COMPLETE;
                end else begin
                    // Miss - need to fetch from memory or another cache
                    cpu_next_state = CPU_REFILL_REQUEST;
                end
            end
            
            CPU_REFILL_REQUEST: begin
                // Send memory request and wait for response
                if (mem_req_ready) begin
                    cpu_next_state = CPU_REFILL_WAIT;
                end
            end
            
            CPU_REFILL_WAIT: begin
                // Wait for memory response
                if (mem_resp_valid && mem_resp_last) begin
                    cpu_next_state = CPU_COMPLETE;
                end
            end
            
            CPU_COHERENCE_WAIT: begin
                // Waiting for coherence operation to complete
                if (all_acks_received) begin
                    cpu_next_state = CPU_COMPLETE;
                end
            end
            
            CPU_COMPLETE: begin
                // Operation complete, return to idle
                cpu_next_state = CPU_IDLE;
            end
        endcase
        
        // If NoC has priority, stall CPU state machine
        if (noc_has_priority && cpu_state != CPU_IDLE) begin
            cpu_next_state = cpu_state;
        end
    end
    
    // NoC State Machine
    always_ff @(posedge clk or negedge reset_n) begin
        if (~reset_n) begin
            noc_state <= NOC_IDLE;
        end else if (flush) begin
            noc_state <= NOC_IDLE;
        end else begin
            noc_state <= noc_next_state;
        end
    end
    
    // NoC Next State Logic
    always_comb begin
        noc_next_state = noc_state;
        
        case (noc_state)
            NOC_IDLE: begin
                if (noc_req_valid) begin
                    noc_next_state = NOC_PROCESS_REQUEST;
                end
            end
            
            NOC_PROCESS_REQUEST: begin
                case (noc_req_type)
                    NOC_READ_REQ, NOC_READ_EXCL_REQ: begin
                        if (noc_cache_hit) begin
                            noc_next_state = NOC_SEND_RESPONSE;
                        end else begin
                            // No hit, no need to respond
                            noc_next_state = NOC_IDLE;
                        end
                    end
                    
                    NOC_INVALIDATE_REQ: begin
                        if (noc_cache_hit) begin
                            // Need to invalidate and send ack
                            noc_next_state = NOC_SEND_RESPONSE;
                        end else begin
                            // No hit, still send ack
                            noc_next_state = NOC_SEND_RESPONSE;
                        end
                    end
                    
                    default: begin
                        // Unknown request type
                        noc_next_state = NOC_IDLE;
                    end
                endcase
            end
            
            NOC_WAIT_ACKS: begin
                if (all_acks_received) begin
                    noc_next_state = NOC_UPDATE_STATE;
                end
            end
            
            NOC_SEND_RESPONSE: begin
                // After sending response, update state if needed
                noc_next_state = NOC_UPDATE_STATE;
            end
            
            NOC_UPDATE_STATE: begin
                // Go back to idle after updating state
                noc_next_state = NOC_IDLE;
            end
        endcase
    end
    
    // Output and internal state updates
    always_ff @(posedge clk or negedge reset_n) begin
        if (~reset_n) begin
            // Reset all outputs and internal state
            cpu_resp_valid <= 1'b0;
            cpu_resp_data <= '0;
            cpu_resp_ready <= 1'b0;
            mem_req_valid <= 1'b0;
            mem_req_addr <= '0;
            mem_req_ready <= 1'b0;
            noc_resp_valid <= 1'b0;
            noc_resp_type <= '0;
            noc_resp_addr <= '0;
            noc_resp_dest <= '0;
            noc_resp_data <= '0;
            dir_req_valid <= 1'b0;
            dir_req_addr <= '0;
            dir_req_type <= '0;
            
            refill_word_count <= '0;
            refill_buffer <= '0;
            expected_acks <= '0;
            received_acks <= '0;
            pending_coherence_state <= INVALID;
            
            // Initialize cache and LRU
            for (int set = 0; set < SETS; set++) begin
                for (int w = 0; w < ASSOCIATIVITY; w++) begin
                    cache_mem[set][w].valid <= 1'b0;
                    cache_mem[set][w].coherence_state <= INVALID;
                    cache_mem[set][w].tag <= '0;
                    cache_mem[set][w].data <= '0;
                    lru_counter[set][w] <= w[3:0];  // Initialize LRU counters
                end
            end
        end else begin
            // Default values
            cpu_resp_valid <= 1'b0;
            mem_req_valid <= 1'b0;
            noc_resp_valid <= 1'b0;
            dir_req_valid <= 1'b0;
            
            // CPU state machine actions
            case (cpu_state)
                CPU_IDLE: begin
                    cpu_resp_ready <= 1'b1;
                end
                
                CPU_TAG_CHECK: begin
                    if (cpu_cache_hit) begin
                        // Hit - return data to CPU
                        cpu_resp_valid <= 1'b1;
                        cpu_resp_data <= selected_word;
                        cpu_resp_ready <= 1'b1;
                        
                        // Update LRU counters for the hit way
                        for (int w = 0; w < ASSOCIATIVITY; w++) begin
                            if (cpu_hit_way[w]) begin
                                // Move accessed way to MRU
                                lru_counter[cpu_index][w] <= ASSOCIATIVITY - 1;
                            end else if (lru_counter[cpu_index][w] > lru_counter[cpu_index][cpu_hit_way_idx]) begin
                                // Decrement counters for more recently used ways
                                lru_counter[cpu_index][w] <= lru_counter[cpu_index][w] - 1'b1;
                            end
                        end
                    end else begin
                        // Miss - not ready to accept new requests
                        cpu_resp_ready <= 1'b0;
                    end
                end
                
                CPU_REFILL_REQUEST: begin
                    // Send memory request
                    mem_req_valid <= 1'b1;
                    // Align to cache line boundary
                    mem_req_addr <= {cpu_req_addr[47:OFFSET_BITS], {OFFSET_BITS{1'b0}}};
                    mem_req_ready <= 1'b1;
                    
                    // Request to directory if needed
                    if (exclusive_inst_fetch) begin
                        dir_req_valid <= 1'b1;
                        dir_req_addr <= {cpu_req_addr[47:OFFSET_BITS], {OFFSET_BITS{1'b0}}};
                        dir_req_type <= 3'b001; // Request exclusive
                    end
                    
                    // Prepare for refill
                    refill_word_count <= '0;
                    refill_buffer <= '0;
                    cpu_resp_ready <= 1'b0;
                end
                
                CPU_REFILL_WAIT: begin
                    // Process memory response
                    if (mem_resp_valid) begin
                        // Store in refill buffer
                        for (int w = 0; w < 8; w++) begin
                            refill_buffer[w*64 +: 64] <= mem_resp_data[w];
                        end
                        
                        if (mem_resp_last) begin
                            // Determine coherence state for new line
                            if (exclusive_inst_fetch) begin
                                pending_coherence_state <= EXCLUSIVE;
                            end else begin
                                pending_coherence_state <= SHARED;
                            end
                        end
                    end
                end
                
                CPU_COHERENCE_WAIT: begin
                    // Check if all acknowledgments received
                    if (all_acks_received) begin
                        // Proceed to update coherence state
                    end
                end
                
                CPU_COMPLETE: begin
                    // Write data to cache and update coherence state on miss
                    if (!cpu_cache_hit && cpu_state != CPU_IDLE) begin
                        cache_mem[cpu_index][replace_way_idx].valid <= 1'b1;
                        cache_mem[cpu_index][replace_way_idx].tag <= cpu_tag;
                        cache_mem[cpu_index][replace_way_idx].data <= refill_buffer;
                        cache_mem[cpu_index][replace_way_idx].coherence_state <= pending_coherence_state;
                        
                        // Update LRU - set replaced way as MRU
                        for (int w = 0; w < ASSOCIATIVITY; w++) begin
                            if (w == replace_way_idx) begin
                                lru_counter[cpu_index][w] <= ASSOCIATIVITY - 1;
                            end else if (lru_counter[cpu_index][w] > 0) begin
                                // Decrement counters for all other ways
                                lru_counter[cpu_index][w] <= lru_counter[cpu_index][w] - 1'b1;
                            end
                        end
                        
                        // Return instruction to CPU
                        cpu_resp_valid <= 1'b1;
                        cpu_resp_data <= refill_buffer[cpu_word_offset*32 +: 32];
                    end
                    
                    // Ready for new requests
                    cpu_resp_ready <= 1'b1;
                end
            endcase
            
            // NoC state machine actions
            case (noc_state)
                NOC_IDLE: begin
                    // Nothing to do in idle
                end
                
                NOC_PROCESS_REQUEST: begin
                    case (noc_req_type)
                        NOC_READ_REQ: begin
                            if (noc_cache_hit) begin
                                // Prepare response for read request
                                noc_resp_valid <= 1'b1;
                                noc_resp_type <= NOC_DATA_RESP;
                                noc_resp_addr <= noc_req_addr;
                                noc_resp_dest <= noc_req_source;
                                
                                // Get 64-bit data aligned to the requested word
                                // Note: This is a simplification - real implementation would
                                // need to handle block transfers based on line size
                                noc_resp_data <= cache_mem[noc_index][noc_hit_way_idx].data[noc_word_offset*32 +: 64];
                                
                                // Update coherence state if needed
                                if (cache_mem[noc_index][noc_hit_way_idx].coherence_state == EXCLUSIVE ||
                                    cache_mem[noc_index][noc_hit_way_idx].coherence_state == MODIFIED) begin
                                    // Downgrade to shared when another core reads
                                    pending_coherence_state <= SHARED;
                                end else begin
                                    // No state change for already shared lines
                                    pending_coherence_state <= cache_mem[noc_index][noc_hit_way_idx].coherence_state;
                                }
                            end
                        end
                        
                        NOC_READ_EXCL_REQ: begin
                            if (noc_cache_hit) begin
                                // Another core wants exclusive access - must invalidate
                                noc_resp_valid <= 1'b1;
                                
                                if (cache_mem[noc_index][noc_hit_way_idx].coherence_state == MODIFIED ||
                                    cache_mem[noc_index][noc_hit_way_idx].coherence_state == OWNED) begin
                                    // If line is modified, need to send the data too
                                    noc_resp_type <= NOC_DATA_RESP;
                                    noc_resp_data <= cache_mem[noc_index][noc_hit_way_idx].data[noc_word_offset*32 +: 64];
                                end else begin
                                    // Otherwise just acknowledge
                                    noc_resp_type <= NOC_ACK_RESP;
                                }
                                
                                noc_resp_addr <= noc_req_addr;
                                noc_resp_dest <= noc_req_source;
                                
                                // Will invalidate in UPDATE_STATE
                                pending_coherence_state <= INVALID;
                            end
                        end
                        
                        NOC_INVALIDATE_REQ: begin
                            // Always send ACK for invalidate
                            noc_resp_valid <= 1'b1;
                            noc_resp_type <= NOC_ACK_RESP;
                            noc_resp_addr <= noc_req_addr;
                            noc_resp_dest <= noc_req_source;
                            
                            if (noc_cache_hit) begin
                                // Will invalidate in UPDATE_STATE
                                pending_coherence_state <= INVALID;
                            end
                        end
                    endcase
                end
                
                NOC_WAIT_ACKS: begin
                    // Count acknowledgments
                    if (noc_resp_valid && noc_resp_type == NOC_ACK_RESP) begin
                        received_acks <= received_acks + 1'b1;
                    end
                end
                
                NOC_SEND_RESPONSE: begin
                    // Response signals already set in PROCESS_REQUEST
                    // Just clear the valid signal
                    noc_resp_valid <= 1'b0;
                end
                
                NOC_UPDATE_STATE: begin
                    if (noc_cache_hit) begin
                        // Update coherence state for hit line
                        cache_mem[noc_index][noc_hit_way_idx].coherence_state <= pending_coherence_state;
                        
                        // If invalidating, mark as invalid
                        if (pending_coherence_state == INVALID) begin
                            cache_mem[noc_index][noc_hit_way_idx].valid <= 1'b0;
                        end
                    end
                    
                    // Reset tracking variables
                    expected_acks <= '0;
                    received_acks <= '0;
                end
            endcase
            
            // Handle cache flush
            if (flush) begin
                for (int set = 0; set < SETS; set++) begin
                    for (int w = 0; w < ASSOCIATIVITY; w++) begin
                        cache_mem[set][w].valid <= 1'b0;
                        cache_mem[set][w].coherence_state <= INVALID;
                    end
                end
                cpu_resp_ready <= 1'b1;
                mem_req_valid <= 1'b0;
                noc_resp_valid <= 1'b0;
            end
        end
    end
    
    // Debug functions - useful for verification
    // Convert coherence state to string for debugging
    function automatic string state_to_string(logic [2:0] state);
        case (state)
            INVALID:   return "I";
            SHARED:    return "S";
            EXCLUSIVE: return "E";
            OWNED:     return "O";
            MODIFIED:  return "M";
            default:   return "?";
        endcase
    endfunction
    
endmodule