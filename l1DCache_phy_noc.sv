module l1dcache(
    input  logic        clk,
    input  logic        reset_n,                    // Active low reset
    
    // CPU Request Interface
    input  logic        cpu_req_valid,              // CPU request valid
    input  logic [47:0] cpu_req_addr,               // Physical Address from CPU
    input  logic        cpu_req_write,              // 1 = write, 0 = read
    input  logic [7:0]  cpu_req_byte_en,            // Byte enables for writes
    input  logic [63:0] cpu_req_data,               // Data for writes
    input  logic        cpu_req_size,               // 0 = 32-bit, 1 = 64-bit
    input  logic        cpu_req_atomic,             // Atomic operation
    input  logic [3:0]  cpu_req_atomic_op,          // Atomic operation type
    input  logic        cpu_req_exclusive,          // Request exclusive access
    input  logic        flush,                      // Trigger to flush the cache
    
    // CPU Response Interface
    output logic        cpu_resp_valid,             // Response to CPU valid
    output logic [63:0] cpu_resp_data,              // Data to CPU
    output logic        cpu_resp_ready,             // Ready to accept new CPU requests
    output logic        cpu_resp_error,             // Error in processing request
    
    // Memory/NoC request interface
    output logic        mem_req_valid,              // Request to memory/NoC
    output logic        mem_req_write,              // Memory write request
    output logic [47:0] mem_req_addr,               // Physical address to memory/NoC
    output logic [63:0] mem_req_data[0:7],          // Data to write to memory (line)
    output logic        mem_req_ready,              // Ready to receive mem responses
    output logic        mem_req_atomic,             // Atomic memory operation
    
    // Memory/NoC response interface
    input  logic        mem_resp_valid,             // Data from Memory/NoC available
    input  logic [63:0] mem_resp_data[0:7],         // Data from memory/NoC (full line)
    input  logic        mem_resp_last,              // Last data transfer flag
    input  logic        mem_resp_error,             // Error from memory/NoC
    
    // NoC coherence interface inputs
    input  logic        noc_req_valid,              // Incoming coherence request valid
    input  logic [2:0]  noc_req_type,               // Request type
    input  logic [47:0] noc_req_addr,               // Request address
    input  logic [2:0]  noc_req_source,             // Source core/cache ID
    input  logic [2:0]  noc_req_dest,               // Destination (only for multicast)
    input  logic [63:0] noc_req_data,               // Data if needed
    
    // NoC coherence interface outputs
    output logic        noc_resp_valid,             // Outgoing coherence response valid
    output logic [2:0]  noc_resp_type,              // Response type
    output logic [47:0] noc_resp_addr,              // Response address
    output logic [2:0]  noc_resp_dest,              // Destination core/cache ID
    output logic [63:0] noc_resp_data,              // Response data if needed
    
    // Directory interface (optional)
    input  logic        dir_resp_valid,             // Directory response valid
    input  logic [7:0]  dir_resp_sharers,           // Bit vector of sharers
    input  logic [2:0]  dir_resp_owner,             // Owner core ID
    output logic        dir_req_valid,              // Directory request valid
    output logic [47:0] dir_req_addr,               // Directory request address
    output logic [2:0]  dir_req_type,               // Directory request type
    
    // Configuration
    input  logic [2:0]  core_id                     // Core ID for this cache
);
    // Cache config
    localparam int CACHE_SIZE     = 256 * 1024;     // 256KB L1 D-Cache
    localparam int LINE_SIZE      = 64;             // 64B cache line size
    localparam int ASSOCIATIVITY  = 8;              // 8-way set associative
    localparam int SETS           = CACHE_SIZE/(LINE_SIZE * ASSOCIATIVITY);
    
    localparam int OFFSET_BITS    = $clog2(LINE_SIZE);
    localparam int INDEX_BITS     = $clog2(SETS);
    localparam int TAG_BITS       = 48 - INDEX_BITS - OFFSET_BITS;
    
    // Word offset bits (8 bytes or 64 bits per word)
    localparam int WORD_OFFSET_BITS = $clog2(LINE_SIZE/8);
    
    // MOESI state encoding
    localparam logic [2:0] INVALID   = 3'b000;
    localparam logic [2:0] SHARED    = 3'b001;
    localparam logic [2:0] EXCLUSIVE = 3'b010;
    localparam logic [2:0] OWNED     = 3'b011;
    localparam logic [2:0] MODIFIED  = 3'b100;
    
    // NoC Message Types
    localparam logic [2:0] NOC_READ_REQ       = 3'b000; // Read request
    localparam logic [2:0] NOC_READ_EXCL_REQ  = 3'b001; // Read exclusive request
    localparam logic [2:0] NOC_WRITE_REQ      = 3'b010; // Write request
    localparam logic [2:0] NOC_INVALIDATE_REQ = 3'b011; // Invalidate request
    localparam logic [2:0] NOC_DATA_RESP      = 3'b100; // Data response
    localparam logic [2:0] NOC_DATA_EXCL_RESP = 3'b101; // Data response with exclusivity
    localparam logic [2:0] NOC_ACK_RESP       = 3'b110; // Acknowledgment
    
    // Atomic operation types
    localparam logic [3:0] ATOMIC_LDADD       = 4'b0000; // Load-Add
    localparam logic [3:0] ATOMIC_LDAPR       = 4'b0001; // Load-Acquire Pair Register
    localparam logic [3:0] ATOMIC_STLR        = 4'b0010; // Store-Release Register
    localparam logic [3:0] ATOMIC_SWAP        = 4'b0011; // Swap
    localparam logic [3:0] ATOMIC_LDAXR       = 4'b0100; // Load-Acquire Exclusive Register
    localparam logic [3:0] ATOMIC_STLXR       = 4'b0101; // Store-Release Exclusive Register
    
    // Address components extraction
    logic [INDEX_BITS-1:0] cpu_index;
    logic [OFFSET_BITS-1:0] cpu_offset;
    logic [WORD_OFFSET_BITS-1:0] cpu_word_offset;
    logic [TAG_BITS-1:0] cpu_tag;
    
    // Extract address components from CPU address
    assign cpu_index = cpu_req_addr[OFFSET_BITS+INDEX_BITS-1:OFFSET_BITS];
    assign cpu_offset = cpu_req_addr[OFFSET_BITS-1:0];
    assign cpu_word_offset = cpu_req_addr[OFFSET_BITS-1:3]; // 8B words
    assign cpu_tag = cpu_req_addr[47:OFFSET_BITS+INDEX_BITS];
    
    // Same for NoC request address
    logic [INDEX_BITS-1:0] noc_index;
    logic [OFFSET_BITS-1:0] noc_offset;
    logic [WORD_OFFSET_BITS-1:0] noc_word_offset;
    logic [TAG_BITS-1:0] noc_tag;
    
    assign noc_index = noc_req_addr[OFFSET_BITS+INDEX_BITS-1:OFFSET_BITS];
    assign noc_offset = noc_req_addr[OFFSET_BITS-1:0];
    assign noc_word_offset = noc_req_addr[OFFSET_BITS-1:3];
    assign noc_tag = noc_req_addr[47:OFFSET_BITS+INDEX_BITS];
    
    // Cache line definition with coherence state
    typedef struct packed {
        logic valid;
        logic dirty;                        // Dirty bit for write-back policy
        logic [2:0] coherence_state;        // MOESI state
        logic [TAG_BITS-1:0] tag;
        logic [LINE_SIZE*8-1:0] data;       // 64 bytes = 512 bits of data
    } cache_line_t;
    
    // Cache storage
    cache_line_t cache_mem[SETS-1:0][ASSOCIATIVITY-1:0];
    
    // LRU tracking - use 4 bits for up to 16-way associativity
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
    
    // Write buffer for store data
    logic [63:0] write_data;                // Data to write
    logic [7:0] write_byte_en;              // Byte enables for write
    
    // Line refill tracking
    logic [2:0] refill_word_count;          // Track words during refill
    logic [LINE_SIZE*8-1:0] refill_buffer;  // Buffer for line being filled
    
    // Selected data from cache
    logic [63:0] cache_word[ASSOCIATIVITY-1:0]; // Selected word from each way
    logic [63:0] selected_word;                 // Selected output word
    
    // Atomic operation logic
    logic [63:0] atomic_result;             // Result of atomic operation
    logic atomic_exclusive_monitor;         // Exclusive access monitor
    logic [47:0] exclusive_addr;            // Address for exclusive access
    
    // MSHRs (Miss Status Holding Registers) - simplified for clarity
    typedef struct packed {
        logic valid;
        logic [47:0] addr;
        logic write;
        logic [63:0] write_data;
        logic [7:0] byte_en;
        logic atomic;
        logic [3:0] atomic_op;
    } mshr_t;
    
    mshr_t mshr;  // Just one MSHR for simplicity
    
    // Priority control
    logic noc_has_priority;  // NoC request has priority over CPU
    
    // State machines for CPU and NoC operations
    typedef enum logic [3:0] {
        CPU_IDLE,
        CPU_TAG_CHECK,
        CPU_WRITE_HIT,
        CPU_EVICT_LINE,
        CPU_REFILL_REQUEST,
        CPU_REFILL_WAIT,
        CPU_COHERENCE_WAIT,
        CPU_ATOMIC_OP,
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
    
    // Write-back buffer
    logic wb_buffer_valid;
    logic [47:0] wb_buffer_addr;
    logic [LINE_SIZE*8-1:0] wb_buffer_data;
    
//    function automatic logic [31:0] extract_word(
//        input logic [LINE_SIZE*8-1:0] data,
//        input logic [WORD_OFFSET_BITS-1:0] word_offset
//    );
//        logic [31:0] result;
//        result = data[word_offset*32 +: 32];
//        return result;
//    endfunction
    
//    function automatic logic [63:0] extract_dword(
//        input logic [LINE_SIZE*8-1:0] data,
//        input logic [WORD_OFFSET_BITS-1:0] word_offset
//    );
//        logic [63:0] result;
//        result = data[word_offset*64 +: 64];
//        return result;
//    endfunction
    
    // Hit detection logic for CPU requests
    genvar way;
    generate
        for (way = 0; way < ASSOCIATIVITY; way++) begin : hit_logic
            assign cpu_hit_way[way] = cache_mem[cpu_index][way].valid && 
                                      (cache_mem[cpu_index][way].tag == cpu_tag) && 
                                      (cache_mem[cpu_index][way].coherence_state != INVALID);
                                      
            // Extract the appropriate word based on the word offset
            // assign cache_word[way] = cache_mem[cpu_index][way].data[cpu_word_offset*64 +: 64];
             assign cache_word[way] = extract_word(cache_mem[cpu_index][way].data, cpu_word_offset);
            
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
    
    // Check if we need to write back a dirty line
    function automatic logic need_writeBack(logic [INDEX_BITS-1:0] index, logic [3:0] way_idx);
        return cache_mem[index][way_idx].valid && 
               cache_mem[index][way_idx].dirty && 
              (cache_mem[index][way_idx].coherence_state == MODIFIED || 
               cache_mem[index][way_idx].coherence_state == OWNED);
    endfunction
    
    // Atomic operation execution
    function automatic logic [63:0] execute_atomic_op(
        input logic [3:0] op,
        input logic [63:0] existing_data,
        input logic [63:0] new_data
    );
        case (op)
            ATOMIC_LDADD: return existing_data + new_data;
            ATOMIC_SWAP:  return new_data;
            default:      return existing_data; // Default for other ops
        endcase
    endfunction
    
    // Helper function: can we write to this line based on coherence state?
    function automatic logic can_write(logic [2:0] state);
        return state == MODIFIED || state == EXCLUSIVE;
    endfunction
    
    // ========================================================================
    // CPU State Machine
    // ========================================================================
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
                    if (cpu_req_write) begin
                        // Check coherence state for writes
                        if (can_write(cache_mem[cpu_index][cpu_hit_way_idx].coherence_state)) begin
                            cpu_next_state = CPU_WRITE_HIT;
                        end else begin
                            // Need to upgrade state before writing
                            cpu_next_state = CPU_COHERENCE_WAIT;
                        end
                    end else if (cpu_req_atomic) begin
                        cpu_next_state = CPU_ATOMIC_OP;
                    end else begin
                        // Simple read hit
                        cpu_next_state = CPU_COMPLETE;
                    end
                end else begin
                    // Check if we need to evict a dirty line
                    if (need_writeBack(cpu_index, replace_way_idx)) begin
                        cpu_next_state = CPU_EVICT_LINE;
                    end else begin
                        // Clean miss, go directly to refill
                        cpu_next_state = CPU_REFILL_REQUEST;
                    end
                end
            end
            
            CPU_WRITE_HIT: begin
                // Write completes in one cycle
                cpu_next_state = CPU_COMPLETE;
            end
            
            CPU_EVICT_LINE: begin
                // Wait for write-back to complete
                if (mem_req_ready && mem_req_valid) begin
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
                    if (cpu_req_write) begin
                        cpu_next_state = CPU_WRITE_HIT;
                    end else if (cpu_req_atomic) begin
                        cpu_next_state = CPU_ATOMIC_OP;
                    end else begin
                        cpu_next_state = CPU_COMPLETE;
                    end
                end
            end
            
            CPU_COHERENCE_WAIT: begin
                // Waiting for coherence operation to complete
                if (all_acks_received) begin
                    if (cpu_req_write) begin
                        cpu_next_state = CPU_WRITE_HIT;
                    end else begin
                        cpu_next_state = CPU_COMPLETE;
                    end
                end
            end
            
            CPU_ATOMIC_OP: begin
                // Atomic operations complete in one cycle after data is available
                cpu_next_state = CPU_COMPLETE;
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
    
    // ========================================================================
    // NoC State Machine
    // ========================================================================
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
                    
                    NOC_WRITE_REQ: begin
                        if (noc_cache_hit) begin
                            // Need to update or invalidate based on protocol
                            noc_next_state = NOC_SEND_RESPONSE;
                        end else begin
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
    
    // ========================================================================
    // Data Path and Control Logic
    // ========================================================================
    always_ff @(posedge clk or negedge reset_n) begin
        if (~reset_n) begin
            // Reset all outputs and internal state
            cpu_resp_valid <= 1'b0;
            cpu_resp_data <= '0;
            cpu_resp_ready <= 1'b0;
            cpu_resp_error <= 1'b0;
            
            mem_req_valid <= 1'b0;
            mem_req_write <= 1'b0;
            mem_req_addr <= '0;
            mem_req_ready <= 1'b0;
            mem_req_atomic <= 1'b0;
            
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
            
            write_data <= '0;
            write_byte_en <= '0;
            
            atomic_result <= '0;
            atomic_exclusive_monitor <= 1'b0;
            exclusive_addr <= '0;
            
            mshr.valid <= 1'b0;
            mshr.addr <= '0;
            mshr.write <= 1'b0;
            mshr.write_data <= '0;
            mshr.byte_en <= '0;
            mshr.atomic <= 1'b0;
            mshr.atomic_op <= '0;
            
            wb_buffer_valid <= 1'b0;
            wb_buffer_addr <= '0;
            wb_buffer_data <= '0;
            
            // Initialize cache and LRU
            for (int set = 0; set < SETS; set++) begin
                for (int w = 0; w < ASSOCIATIVITY; w++) begin
                    cache_mem[set][w].valid <= 1'b0;
                    cache_mem[set][w].dirty <= 1'b0;
                    cache_mem[set][w].coherence_state <= INVALID;
                    cache_mem[set][w].tag <= '0;
                    cache_mem[set][w].data <= '0;
                    lru_counter[set][w] <= w[3:0];  // Initialize LRU counters
                end
            end
        end else begin
            // Default values for outputs that need to be cleared every cycle
            cpu_resp_valid <= 1'b0;
            mem_req_valid <= 1'b0;
            noc_resp_valid <= 1'b0;
            dir_req_valid <= 1'b0;
            
            // ================================================================
            // CPU State Machine Actions
            // ================================================================
            case (cpu_state)
                CPU_IDLE: begin
                    // Ready to accept new requests
                    cpu_resp_ready <= 1'b1;
                    
                    // If new request, store in MSHR
                    if (cpu_req_valid && cpu_next_state == CPU_TAG_CHECK) begin
                        mshr.valid <= 1'b1;
                        mshr.addr <= cpu_req_addr;
                        mshr.write <= cpu_req_write;
                        mshr.write_data <= cpu_req_data;
                        mshr.byte_en <= cpu_req_byte_en;
                        mshr.atomic <= cpu_req_atomic;
                        mshr.atomic_op <= cpu_req_atomic_op;
                    end
                end
                
                CPU_TAG_CHECK: begin
                    if (cpu_cache_hit) begin
                        // Hit path
                        if (!mshr.write && !mshr.atomic) begin
                            // Read hit, return data immediately
                            cpu_resp_valid <= 1'b1;
                            
                            // Handle 32-bit vs 64-bit reads
                            if (cpu_req_size == 1'b0) begin
                                // 32-bit read - place in lower 32 bits
                                cpu_resp_data <= {32'b0, selected_word[31:0]};
                            end else begin
                                // 64-bit read
                                cpu_resp_data <= selected_word;
                            end
                            
                            // Update LRU for hit way
                            for (int w = 0; w < ASSOCIATIVITY; w++) begin
                                if (cpu_hit_way[w]) begin
                                    // Move accessed way to MRU
                                    lru_counter[cpu_index][w] <= ASSOCIATIVITY - 1;
                                end else if (lru_counter[cpu_index][w] > lru_counter[cpu_index][cpu_hit_way_idx]) begin
                                    // Decrement counters for more recently used ways
                                    lru_counter[cpu_index][w] <= lru_counter[cpu_index][w] - 1'b1;
                                end
                            end
                        end
                        
                        // For writes, need to check coherence state
                        if (mshr.write && 
                            !can_write(cache_mem[cpu_index][cpu_hit_way_idx].coherence_state)) begin
                            // Need to upgrade to exclusive or modified
                            dir_req_valid <= 1'b1;
                            dir_req_addr <= mshr.addr;
                            dir_req_type <= 3'b001; // Request exclusive
                            
                            // Track expected acknowledgments
                            if (dir_resp_valid) begin
                                expected_acks <= dir_resp_sharers;
                                received_acks <= '0;
                            end
                        end
                    end else begin
                        // Miss path - not ready to accept new requests
                        cpu_resp_ready <= 1'b0;
                    end
                end
                
                CPU_WRITE_HIT: begin
                    // Perform the write to cache
                    for (int i = 0; i < 8; i++) begin
                        if (mshr.byte_en[i]) begin
                            cache_mem[cpu_index][cpu_hit_way_idx].data[cpu_word_offset*64 + i*8 +: 8] <= 
                                mshr.write_data[i*8 +: 8];
                        end
                    end
                    
                    // Mark line as dirty
                    cache_mem[cpu_index][cpu_hit_way_idx].dirty <= 1'b1;
                    
                    // Update coherence state to modified
                    cache_mem[cpu_index][cpu_hit_way_idx].coherence_state <= MODIFIED;
                    
                    // Update LRU
                    for (int w = 0; w < ASSOCIATIVITY; w++) begin
                        if (w == cpu_hit_way_idx) begin
                            lru_counter[cpu_index][w] <= ASSOCIATIVITY - 1;
                        end else if (lru_counter[cpu_index][w] > lru_counter[cpu_index][cpu_hit_way_idx]) begin
                            lru_counter[cpu_index][w] <= lru_counter[cpu_index][w] - 1'b1;
                        end
                    end
                end
                
                CPU_EVICT_LINE: begin
                    // Write back dirty line
                    if (!wb_buffer_valid) begin
                        // Prepare write-back buffer
                        wb_buffer_valid <= 1'b1;
                        wb_buffer_addr <= {cache_mem[cpu_index][replace_way_idx].tag, 
                                           cpu_index, 
                                           {OFFSET_BITS{1'b0}}};
                        wb_buffer_data <= cache_mem[cpu_index][replace_way_idx].data;
                    end
                    
                    // Send write-back request to memory
                    if (wb_buffer_valid) begin
                        mem_req_valid <= 1'b1;
                        mem_req_write <= 1'b1;
                        mem_req_addr <= wb_buffer_addr;
                        
                        // Break cache line into words for memory
                        for (int w = 0; w < 8; w++) begin
                            mem_req_data[w] <= wb_buffer_data[w*64 +: 64];
                        end
                        
                        // Clear buffer once request is accepted
                        if (mem_req_ready) begin
                            wb_buffer_valid <= 1'b0;
                            // Also invalidate the cache line
                            cache_mem[cpu_index][replace_way_idx].valid <= 1'b0;
                            cache_mem[cpu_index][replace_way_idx].dirty <= 1'b0;
                            cache_mem[cpu_index][replace_way_idx].coherence_state <= INVALID;
                        end
                    end
                end
                
                CPU_REFILL_REQUEST: begin
                    // Send memory request for the missing line
                    mem_req_valid <= 1'b1;
                    mem_req_write <= 1'b0;
                    // Align to cache line boundary
                    mem_req_addr <= {mshr.addr[47:OFFSET_BITS], {OFFSET_BITS{1'b0}}};
                    mem_req_ready <= 1'b1;
                    
                    // Determine the coherence state we need
                    if (mshr.write || mshr.atomic || mshr.atomic_op == ATOMIC_LDAXR) begin
                        // Need exclusive access for writes/atomics
                        dir_req_valid <= 1'b1;
                        dir_req_addr <= mshr.addr;
                        dir_req_type <= 3'b001; // Request exclusive
                        pending_coherence_state <= EXCLUSIVE;
                    end else begin
                        // Regular read
                        pending_coherence_state <= SHARED;
                    end
                    
                    // Reset refill tracking
                    refill_word_count <= '0;
                    refill_buffer <= '0;
                end
                
                CPU_REFILL_WAIT: begin
                    // Process memory response
                    if (mem_resp_valid) begin
                        // Store data in refill buffer
                        for (int w = 0; w < 8; w++) begin
                            refill_buffer[w*64 +: 64] <= mem_resp_data[w];
                        end
                        
                        if (mem_resp_error) begin
                            // Handle memory error
                            cpu_resp_error <= 1'b1;
                        end
                    end
                end
                
                CPU_COHERENCE_WAIT: begin
                    // Wait for all acknowledgments
                    if (noc_resp_valid && noc_resp_type == NOC_ACK_RESP) begin
                        received_acks <= received_acks + 1'b1;
                    end
                    
                    // Once all acks received, update coherence state
                    if (all_acks_received) begin
                        if (mshr.write) begin
                            // For writes, we need MODIFIED state
                            cache_mem[cpu_index][cpu_hit_way_idx].coherence_state <= MODIFIED;
                        end else if (mshr.atomic) begin
                            // For atomics, depends on the specific operation
                            case (mshr.atomic_op)
                                ATOMIC_LDAXR: begin
                                    // Set exclusive monitor
                                    atomic_exclusive_monitor <= 1'b1;
                                    exclusive_addr <= mshr.addr;
                                    cache_mem[cpu_index][cpu_hit_way_idx].coherence_state <= EXCLUSIVE;
                                end
                                ATOMIC_STLXR: begin
                                    // Check exclusive monitor
                                    if (atomic_exclusive_monitor && exclusive_addr == mshr.addr) begin
                                        cache_mem[cpu_index][cpu_hit_way_idx].coherence_state <= MODIFIED;
                                        cache_mem[cpu_index][cpu_hit_way_idx].dirty <= 1'b1;
                                    end
                                    // Clear monitor after use
                                    atomic_exclusive_monitor <= 1'b0;
                                end
                                default: begin
                                    // For other atomics, go to MODIFIED
                                    cache_mem[cpu_index][cpu_hit_way_idx].coherence_state <= MODIFIED;
                                end
                            endcase
                        end
                    end
                end
                
                CPU_ATOMIC_OP: begin
                    // Execute atomic operation
                    case (mshr.atomic_op)
                        ATOMIC_LDADD: begin
                            // Load value, add, store result
                            atomic_result <= execute_atomic_op(ATOMIC_LDADD, 
                                                             selected_word, 
                                                             mshr.write_data);
                            
                            // Update cache with result
                            cache_mem[cpu_index][cpu_hit_way_idx].data[cpu_word_offset*64 +: 64] <= 
                                execute_atomic_op(ATOMIC_LDADD, selected_word, mshr.write_data);
                            
                            // Mark as modified and dirty
                            cache_mem[cpu_index][cpu_hit_way_idx].coherence_state <= MODIFIED;
                            cache_mem[cpu_index][cpu_hit_way_idx].dirty <= 1'b1;
                        end
                        
                        ATOMIC_LDAPR: begin
                            // Load-Acquire acts as a memory barrier
                            atomic_result <= selected_word;
                            // Doesn't change coherence state
                        end
                        
                        ATOMIC_STLR: begin
                            // Store-Release acts as a memory barrier
                            cache_mem[cpu_index][cpu_hit_way_idx].data[cpu_word_offset*64 +: 64] <= 
                                mshr.write_data;
                            
                            // Mark as modified and dirty
                            cache_mem[cpu_index][cpu_hit_way_idx].coherence_state <= MODIFIED;
                            cache_mem[cpu_index][cpu_hit_way_idx].dirty <= 1'b1;
                        end
                        
                        ATOMIC_SWAP: begin
                            // Atomically swap values
                            atomic_result <= selected_word;
                            
                            // Update cache with new value
                            cache_mem[cpu_index][cpu_hit_way_idx].data[cpu_word_offset*64 +: 64] <= 
                                mshr.write_data;
                            
                            // Mark as modified and dirty
                            cache_mem[cpu_index][cpu_hit_way_idx].coherence_state <= MODIFIED;
                            cache_mem[cpu_index][cpu_hit_way_idx].dirty <= 1'b1;
                        end
                        
                        ATOMIC_LDAXR: begin
                            // Load-Exclusive
                            atomic_result <= selected_word;
                            atomic_exclusive_monitor <= 1'b1;
                            exclusive_addr <= mshr.addr;
                        end
                        
                        ATOMIC_STLXR: begin
                            // Store-Exclusive if monitor still set
                            if (atomic_exclusive_monitor && exclusive_addr == mshr.addr) begin
                                cache_mem[cpu_index][cpu_hit_way_idx].data[cpu_word_offset*64 +: 64] <= 
                                    mshr.write_data;
                                
                                // Mark as modified and dirty
                                cache_mem[cpu_index][cpu_hit_way_idx].coherence_state <= MODIFIED;
                                cache_mem[cpu_index][cpu_hit_way_idx].dirty <= 1'b1;
                                
                                // Success
                                atomic_result <= 64'h0;
                            end else begin
                                // Failure
                                atomic_result <= 64'h1;
                            end
                            
                            // Clear monitor
                            atomic_exclusive_monitor <= 1'b0;
                        end
                    endcase
                    
                    // Update LRU
                    for (int w = 0; w < ASSOCIATIVITY; w++) begin
                        if (w == cpu_hit_way_idx) begin
                            lru_counter[cpu_index][w] <= ASSOCIATIVITY - 1;
                        end else if (lru_counter[cpu_index][w] > lru_counter[cpu_index][cpu_hit_way_idx]) begin
                            lru_counter[cpu_index][w] <= lru_counter[cpu_index][w] - 1'b1;
                        end
                    end
                end
                
                CPU_COMPLETE: begin
                    // For a miss, install the refilled line
                    if (!cpu_cache_hit && mem_resp_valid) begin
                        // Write refill data to cache
                        cache_mem[cpu_index][replace_way_idx].valid <= 1'b1;
                        cache_mem[cpu_index][replace_way_idx].tag <= cpu_tag;
                        cache_mem[cpu_index][replace_way_idx].data <= refill_buffer;
                        cache_mem[cpu_index][replace_way_idx].coherence_state <= pending_coherence_state;
                        cache_mem[cpu_index][replace_way_idx].dirty <= mshr.write;
                        
                        // Update LRU
                        for (int w = 0; w < ASSOCIATIVITY; w++) begin
                            if (w == replace_way_idx) begin
                                lru_counter[cpu_index][w] <= ASSOCIATIVITY - 1;
                            end else if (lru_counter[cpu_index][w] > 0) begin
                                lru_counter[cpu_index][w] <= lru_counter[cpu_index][w] - 1'b1;
                            end
                        end
                        
                        // For reads, return refilled data
                        if (!mshr.write) begin
                            cpu_resp_valid <= 1'b1;
                            
                            if (mshr.atomic) begin
                                // Return atomic result
                                cpu_resp_data <= atomic_result;
                            end else begin
                                // Handle 32-bit vs 64-bit reads
                                if (cpu_req_size == 1'b0) begin
                                    // 32-bit read
                                    cpu_resp_data <= {32'b0, refill_buffer[cpu_word_offset*64 +: 32]};
                                end else begin
                                    // 64-bit read
                                    cpu_resp_data <= refill_buffer[cpu_word_offset*64 +: 64];
                                end
                            end
                        end
                    end
                    
                    // For write/atomic completions
                    if ((mshr.write || mshr.atomic) && cpu_cache_hit) begin
                        cpu_resp_valid <= 1'b1;
                        
                        if (mshr.atomic) begin
                            // Return original value for atomic ops
                            cpu_resp_data <= atomic_result;
                        end else begin
                            // For standard writes, no data returned
                            cpu_resp_data <= '0;
                        end
                    end
                    
                    // Mark request as complete
                    mshr.valid <= 1'b0;
                    
                    // Ready for new requests
                    cpu_resp_ready <= 1'b1;
                end
            endcase
            
            // ================================================================
            // NoC State Machine Actions
            // ================================================================
            case (noc_state)
                NOC_IDLE: begin
                    // Nothing to do
                end
                
                NOC_PROCESS_REQUEST: begin
                    case (noc_req_type)
                        NOC_READ_REQ: begin
                            if (noc_cache_hit) begin
                                // Prepare data response for read request
                                noc_resp_valid <= 1'b1;
                                noc_resp_type <= NOC_DATA_RESP;
                                noc_resp_addr <= noc_req_addr;
                                noc_resp_dest <= noc_req_source;
                                
                                // Get data aligned to the requested word
                                noc_resp_data <= cache_mem[noc_index][noc_hit_way_idx].data[noc_word_offset*64 +: 64];
                                
                                // Update coherence state if needed
                                if (cache_mem[noc_index][noc_hit_way_idx].coherence_state == EXCLUSIVE ||
                                    cache_mem[noc_index][noc_hit_way_idx].coherence_state == MODIFIED) begin
                                    // Downgrade to shared when another core reads
                                    pending_coherence_state <= SHARED;
                                end else begin
                                    // No state change for already shared lines
                                    pending_coherence_state <= cache_mem[noc_index][noc_hit_way_idx].coherence_state;
                                end
                            end
                        end
                        
                        NOC_READ_EXCL_REQ: begin
                            if (noc_cache_hit) begin
                                // Another core wants exclusive access
                                noc_resp_valid <= 1'b1;
                                
                                if (cache_mem[noc_index][noc_hit_way_idx].coherence_state == MODIFIED ||
                                    cache_mem[noc_index][noc_hit_way_idx].coherence_state == OWNED) begin
                                    // If line is modified, send the data
                                    noc_resp_type <= NOC_DATA_RESP;
                                    noc_resp_data <= cache_mem[noc_index][noc_hit_way_idx].data[noc_word_offset*64 +: 64];
                                end else begin
                                    // Otherwise just acknowledge
                                    noc_resp_type <= NOC_ACK_RESP;
                                end
                                
                                noc_resp_addr <= noc_req_addr;
                                noc_resp_dest <= noc_req_source;
                                
                                // Will invalidate in UPDATE_STATE
                                pending_coherence_state <= INVALID;
                                
                                // If line is dirty, need to write back first
                                if (cache_mem[noc_index][noc_hit_way_idx].dirty) begin
                                    // Schedule write-back (in a real implementation,
                                    // would need more robust handling here)
                                    wb_buffer_valid <= 1'b1;
                                    wb_buffer_addr <= {cache_mem[noc_index][noc_hit_way_idx].tag,
                                                      noc_index,
                                                      {OFFSET_BITS{1'b0}}};
                                    wb_buffer_data <= cache_mem[noc_index][noc_hit_way_idx].data;
                                end
                            end
                        end
                        
                        NOC_WRITE_REQ: begin
                            if (noc_cache_hit) begin
                                // Another core is writing
                                noc_resp_valid <= 1'b1;
                                noc_resp_type <= NOC_ACK_RESP;
                                noc_resp_addr <= noc_req_addr;
                                noc_resp_dest <= noc_req_source;
                                
                                // Will invalidate in UPDATE_STATE
                                pending_coherence_state <= INVALID;
                                
                                // If dirty, schedule write-back
                                if (cache_mem[noc_index][noc_hit_way_idx].dirty) begin
                                    wb_buffer_valid <= 1'b1;
                                    wb_buffer_addr <= {cache_mem[noc_index][noc_hit_way_idx].tag,
                                                      noc_index,
                                                      {OFFSET_BITS{1'b0}}};
                                    wb_buffer_data <= cache_mem[noc_index][noc_hit_way_idx].data;
                                end
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
                                
                                // If dirty, schedule write-back
                                if (cache_mem[noc_index][noc_hit_way_idx].dirty) begin
                                    wb_buffer_valid <= 1'b1;
                                    wb_buffer_addr <= {cache_mem[noc_index][noc_hit_way_idx].tag,
                                                      noc_index,
                                                      {OFFSET_BITS{1'b0}}};
                                    wb_buffer_data <= cache_mem[noc_index][noc_hit_way_idx].data;
                                end
                            end
                        end
                    endcase
                end
                
                NOC_WAIT_ACKS: begin
                    // This state is for when we're waiting for acks from other caches
                    if (noc_resp_valid && noc_resp_type == NOC_ACK_RESP) begin
                        received_acks <= received_acks + 1'b1;
                    end
                end
                
                NOC_SEND_RESPONSE: begin
                    // Response signals already set in PROCESS_REQUEST
                    noc_resp_valid <= 1'b0; // Clear after one cycle
                end
                
                NOC_UPDATE_STATE: begin
                    if (noc_cache_hit) begin
                        // Update coherence state based on request
                        cache_mem[noc_index][noc_hit_way_idx].coherence_state <= pending_coherence_state;
                        
                        // If invalidating, mark as invalid
                        if (pending_coherence_state == INVALID) begin
                            cache_mem[noc_index][noc_hit_way_idx].valid <= 1'b0;
                            cache_mem[noc_index][noc_hit_way_idx].dirty <= 1'b0;
                        end
                    end
                    
                    // Handle any pending write-backs
                    if (wb_buffer_valid && !mem_req_valid) begin
                        mem_req_valid <= 1'b1;
                        mem_req_write <= 1'b1;
                        mem_req_addr <= wb_buffer_addr;
                        
                        // Break cache line into words for memory
                        for (int w = 0; w < 8; w++) begin
                            mem_req_data[w] <= wb_buffer_data[w*64 +: 64];
                        end
                        
                        // Clear buffer once request is accepted
                        if (mem_req_ready) begin
                            wb_buffer_valid <= 1'b0;
                        end
                    end
                    
                    // Reset tracking
                    expected_acks <= '0;
                    received_acks <= '0;
                end
            endcase
            
            // Handle cache flush
            if (flush) begin
                // Process all dirty lines first
                // In reality, this would be more complex to avoid blocking
                wb_buffer_valid <= 1'b0;
                
                for (int set = 0; set < SETS; set++) begin
                    for (int w = 0; w < ASSOCIATIVITY; w++) begin
                        if (cache_mem[set][w].valid && cache_mem[set][w].dirty) begin
                            // Schedule write-back (simplified)
                            wb_buffer_valid <= 1'b1;
                            wb_buffer_addr <= {cache_mem[set][w].tag, 
                                               set[INDEX_BITS-1:0], 
                                               {OFFSET_BITS{1'b0}}};
                            wb_buffer_data <= cache_mem[set][w].data;
                            
                            // Clear once scheduled
                            cache_mem[set][w].valid <= 1'b0;
                            cache_mem[set][w].dirty <= 1'b0;
                            cache_mem[set][w].coherence_state <= INVALID;
                        end else begin
                            // Just invalidate clean lines
                            cache_mem[set][w].valid <= 1'b0;
                            cache_mem[set][w].coherence_state <= INVALID;
                        end
                    end
                end
                
                // Reset state signals
                atomic_exclusive_monitor <= 1'b0;
                cpu_resp_ready <= 1'b1;
                mshr.valid <= 1'b0;
            end
        end
    end
    
    // Helper functions for debugging
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
    
    function automatic string cpu_state_to_string(cpu_state_t state);
        case (state)
            CPU_IDLE:           return "IDLE";
            CPU_TAG_CHECK:      return "TAG_CHECK";
            CPU_WRITE_HIT:      return "WRITE_HIT";
            CPU_EVICT_LINE:     return "EVICT_LINE";
            CPU_REFILL_REQUEST: return "REFILL_REQ";
            CPU_REFILL_WAIT:    return "REFILL_WAIT";
            CPU_COHERENCE_WAIT: return "COH_WAIT";
            CPU_ATOMIC_OP:      return "ATOMIC_OP";
            CPU_COMPLETE:       return "COMPLETE";
            default:            return "UNKNOWN";
        endcase
    endfunction
    
endmodule