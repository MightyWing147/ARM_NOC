module l2cache_unified #(
    parameter int CACHE_SIZE     = 2 * 1024 * 1024,  // 2MB L2 Cache
    parameter int LINE_SIZE      = 64,               // 64B cache line size
    parameter int ASSOCIATIVITY  = 16,               // 16-way set associative
    parameter int MSHR_ENTRIES   = 16                // Number of MSHR entries
)(
    input  logic        clk,
    input  logic        reset_n,                    // Active low reset
    
    // L1 Interface (Instruction Cache)
    input  logic        l1i_req_valid,              // L1-I request valid
    input  logic [47:0] l1i_req_addr,               // Physical Address from L1-I
    input  logic        l1i_req_write,              // Always 0 for I-Cache
    input  logic [63:0] l1i_req_data,               // Not used for I-Cache
    output logic        l1i_resp_valid,             // Response to L1-I valid
    output logic [63:0] l1i_resp_data[0:7],         // Data to L1-I (complete cache line)
    output logic        l1i_resp_ready,             // Ready to accept new L1-I requests
    
    // L1 Interface (Data Cache)
    input  logic        l1d_req_valid,              // L1-D request valid
    input  logic [47:0] l1d_req_addr,               // Physical Address from L1-D
    input  logic        l1d_req_write,              // 1 = write, 0 = read
    input  logic [63:0] l1d_req_data[0:7],          // Data for writes (complete cache line)
    output logic        l1d_resp_valid,             // Response to L1-D valid
    output logic [63:0] l1d_resp_data[0:7],         // Data to L1-D (complete cache line)
    output logic        l1d_resp_ready,             // Ready to accept new L1-D requests
    
    // Memory/Main Memory interface
    output logic        mem_req_valid,              // Request to memory
    output logic        mem_req_write,              // Memory write request
    output logic [47:0] mem_req_addr,               // Physical address to memory
    output logic [63:0] mem_req_data[0:7],          // Data to write to memory (line)
    output logic        mem_req_ready,              // Ready to receive mem responses
    
    // Memory response interface
    input  logic        mem_resp_valid,             // Data from Memory available
    input  logic [63:0] mem_resp_data[0:7],         // Data from memory
    input  logic        mem_resp_last,              // Last data transfer flag
    input  logic        mem_resp_error,             // Error from memory
    
    // NoC coherence interface inputs
    input  logic        noc_req_valid,              // Incoming coherence request valid
    input  logic [2:0]  noc_req_type,               // Request type
    input  logic [47:0] noc_req_addr,               // Request address
    input  logic [2:0]  noc_req_source,             // Source core/cache ID
    input  logic [2:0]  noc_req_dest,               // Destination (only for multicast)
    input  logic [63:0] noc_req_data[0:7],          // Data if needed (full line)
    
    // NoC coherence interface outputs
    output logic        noc_resp_valid,             // Outgoing coherence response valid
    output logic [2:0]  noc_resp_type,              // Response type
    output logic [47:0] noc_resp_addr,              // Response address
    output logic [2:0]  noc_resp_dest,              // Destination core/cache ID
    output logic [63:0] noc_resp_data[0:7],         // Response data if needed (full line)
    
    // Directory interface from L2 to directory
    input  logic        dir_resp_valid,             // Directory response valid
    input  logic [7:0]  dir_resp_sharers,           // Bit vector of sharers
    input  logic [2:0]  dir_resp_owner,             // Owner core ID
    output logic        dir_req_valid,              // Directory request valid
    output logic [47:0] dir_req_addr,               // Directory request address
    output logic [2:0]  dir_req_type,               // Directory request type
    
    // Snoop responses from L1 caches
    input  logic        l1i_snoop_valid,            // L1-I snoop response valid
    input  logic [2:0]  l1i_snoop_type,             // L1-I snoop response type
    input  logic [47:0] l1i_snoop_addr,             // L1-I snoop address
    input  logic [63:0] l1i_snoop_data[0:7],        // L1-I snoop data (if modified)
    output logic        l1i_snoop_req_valid,        // L1-I snoop request valid
    output logic [2:0]  l1i_snoop_req_type,         // L1-I snoop request type
    output logic [47:0] l1i_snoop_req_addr,         // L1-I snoop request address
    
    input  logic        l1d_snoop_valid,            // L1-D snoop response valid
    input  logic [2:0]  l1d_snoop_type,             // L1-D snoop response type
    input  logic [47:0] l1d_snoop_addr,             // L1-D snoop address
    input  logic [63:0] l1d_snoop_data[0:7],        // L1-D snoop data (if modified)
    output logic        l1d_snoop_req_valid,        // L1-D snoop request valid
    output logic [2:0]  l1d_snoop_req_type,         // L1-D snoop request type
    output logic [47:0] l1d_snoop_req_addr,         // L1-D snoop request address
    
    // Configuration
    input  logic [2:0]  core_id,                    // Core ID for this cache
    input  logic        flush                       // Trigger to flush the cache
);
    // Derived parameters
    localparam int SETS           = CACHE_SIZE/(LINE_SIZE * ASSOCIATIVITY);
    localparam int OFFSET_BITS    = $clog2(LINE_SIZE);
    localparam int INDEX_BITS     = $clog2(SETS);
    localparam int TAG_BITS       = 48 - INDEX_BITS - OFFSET_BITS;
    
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
    
    // Snoop response types
    localparam logic [2:0] SNOOP_NOHIT        = 3'b000; // Line not in L1
    localparam logic [2:0] SNOOP_HIT_CLEAN    = 3'b001; // Line in L1, clean
    localparam logic [2:0] SNOOP_HIT_MODIFIED = 3'b010; // Line in L1, modified
    
    // MSHR (Miss Status Holding Register) states
    localparam logic [1:0] MSHR_IDLE          = 2'b00;
    localparam logic [1:0] MSHR_WAITING_L1    = 2'b01; // Waiting for L1 snoop
    localparam logic [1:0] MSHR_WAITING_MEM   = 2'b10; // Waiting for memory
    localparam logic [1:0] MSHR_WAITING_DIR   = 2'b11; // Waiting for directory
    
    // State machine states
    typedef enum logic [3:0] {
        IDLE,
        TAG_CHECK,
        EVICT_PREPARE,
        EVICT_WRITE,
        REFILL_REQUEST,
        REFILL_WAIT,
        COHERENCE_REQUEST,
        COHERENCE_WAIT,
        SNOOP_L1,
        SNOOP_WAIT,
        UPDATE_STATE,
        SERVICE_L1I,
        SERVICE_L1D,
        COMPLETE
    } cache_state_t;
    
    // Cache line definition with coherence state
    typedef struct packed {
        logic valid;
        logic dirty;                        // Dirty bit for write-back policy
        logic [2:0] coherence_state;        // MOESI state
        logic [TAG_BITS-1:0] tag;
        logic [LINE_SIZE*8-1:0] data;       // 64 bytes = 512 bits of data
    } cache_line_t;
    
    // MSHR structure to track outstanding requests
    typedef struct packed {
        logic valid;
        logic [1:0] state;                  // MSHR state
        logic [47:0] addr;                  // Request address
        logic write;                        // Write request
        logic is_l1i;                       // Request from L1I (vs L1D)
        logic [2:0] coherence_state;        // Target coherence state
        logic [7:0] sharers;                // Tracked sharers
        logic l1i_snooped;                  // L1I snoop complete
        logic l1d_snooped;                  // L1D snoop complete
        logic [2:0] l1i_snoop_result;       // L1I snoop result
        logic [2:0] l1d_snoop_result;       // L1D snoop result
        logic [3:0] way;                    // Way for hit/replacement
    } mshr_t;
    
    // Separate write data array for MSHR
    logic [63:0] mshr_write_data[0:7];
    
    // Address extraction for different interfaces
    // L1I request
    logic [INDEX_BITS-1:0] l1i_index;
    logic [TAG_BITS-1:0] l1i_tag;
    assign l1i_index = l1i_req_addr[OFFSET_BITS+INDEX_BITS-1:OFFSET_BITS];
    assign l1i_tag = l1i_req_addr[47:OFFSET_BITS+INDEX_BITS];
    
    // L1D request
    logic [INDEX_BITS-1:0] l1d_index;
    logic [TAG_BITS-1:0] l1d_tag;
    assign l1d_index = l1d_req_addr[OFFSET_BITS+INDEX_BITS-1:OFFSET_BITS];
    assign l1d_tag = l1d_req_addr[47:OFFSET_BITS+INDEX_BITS];
    
    // NoC request
    logic [INDEX_BITS-1:0] noc_index;
    logic [TAG_BITS-1:0] noc_tag;
    assign noc_index = noc_req_addr[OFFSET_BITS+INDEX_BITS-1:OFFSET_BITS];
    assign noc_tag = noc_req_addr[47:OFFSET_BITS+INDEX_BITS];
    
    // MSHR address
    logic [INDEX_BITS-1:0] mshr_index;
    logic [TAG_BITS-1:0] mshr_tag;
    assign mshr_index = mshr.addr[OFFSET_BITS+INDEX_BITS-1:OFFSET_BITS];
    assign mshr_tag = mshr.addr[47:OFFSET_BITS+INDEX_BITS];
    
    // Internal storage
    cache_line_t cache_mem[SETS-1:0][ASSOCIATIVITY-1:0];
    logic [3:0] lru_counter[SETS-1:0][ASSOCIATIVITY-1:0]; // 4 bits for 16-way
    mshr_t mshr;
    
    // State machine
    cache_state_t state, next_state;
    
    // Hit detection logic
    logic [ASSOCIATIVITY-1:0] l1i_hit_way;
    logic [ASSOCIATIVITY-1:0] l1d_hit_way;
    logic [ASSOCIATIVITY-1:0] noc_hit_way;
    logic [ASSOCIATIVITY-1:0] mshr_hit_way;
    
    logic l1i_cache_hit;
    logic l1d_cache_hit;
    logic noc_cache_hit;
    logic mshr_cache_hit;
    
    logic [3:0] l1i_hit_way_idx;
    logic [3:0] l1d_hit_way_idx;
    logic [3:0] noc_hit_way_idx;
    logic [3:0] mshr_hit_way_idx;
    
    // Replacement policy
    logic [3:0] replace_way_idx;
    
    // Writeback buffer
    logic wb_buffer_valid;
    logic [47:0] wb_buffer_addr;
    logic [63:0] wb_buffer_data[0:7];
    
    // Arbitration logic - priority order (highest to lowest)
    // 1. NoC coherence requests (to prevent deadlocks)
    // 2. L1D requests (data accesses are typically more latency-sensitive)
    // 3. L1I requests (instruction fetches can be prefetched/pipelined)
    logic service_noc;
    logic service_l1d;
    logic service_l1i;
    
    assign service_noc = noc_req_valid;
    assign service_l1d = l1d_req_valid && !service_noc && state == IDLE;
    assign service_l1i = l1i_req_valid && !service_noc && !service_l1d && state == IDLE;
    
    // Generate hit detection logic
    genvar way;
    generate
        for (way = 0; way < ASSOCIATIVITY; way++) begin : hit_logic
            // L1I hit detection
            assign l1i_hit_way[way] = cache_mem[l1i_index][way].valid && 
                                     (cache_mem[l1i_index][way].tag == l1i_tag) && 
                                     (cache_mem[l1i_index][way].coherence_state != INVALID);
            
            // L1D hit detection
            assign l1d_hit_way[way] = cache_mem[l1d_index][way].valid && 
                                     (cache_mem[l1d_index][way].tag == l1d_tag) && 
                                     (cache_mem[l1d_index][way].coherence_state != INVALID);
            
            // NoC hit detection
            assign noc_hit_way[way] = cache_mem[noc_index][way].valid && 
                                     (cache_mem[noc_index][way].tag == noc_tag) && 
                                     (cache_mem[noc_index][way].coherence_state != INVALID);
                                     
            // MSHR hit detection (used during various operations)
            assign mshr_hit_way[way] = cache_mem[mshr_index][way].valid && 
                                      (cache_mem[mshr_index][way].tag == mshr_tag) && 
                                      (cache_mem[mshr_index][way].coherence_state != INVALID);
        end
    endgenerate
    
    // Overall hit detection
    assign l1i_cache_hit = |l1i_hit_way && l1i_req_valid;
    assign l1d_cache_hit = |l1d_hit_way && l1d_req_valid;
    assign noc_cache_hit = |noc_hit_way && noc_req_valid;
    assign mshr_cache_hit = |mshr_hit_way && mshr.valid;
    
    // Convert one-hot hit way to binary index
    always_comb begin
        l1i_hit_way_idx = '0;
        l1d_hit_way_idx = '0;
        noc_hit_way_idx = '0;
        mshr_hit_way_idx = '0;
        
        for (int i = 0; i < ASSOCIATIVITY; i++) begin
            if (l1i_hit_way[i]) l1i_hit_way_idx = i[3:0];
            if (l1d_hit_way[i]) l1d_hit_way_idx = i[3:0];
            if (noc_hit_way[i]) noc_hit_way_idx = i[3:0];
            if (mshr_hit_way[i]) mshr_hit_way_idx = i[3:0];
        end
    end
    
    // Find LRU way for replacement
    always_comb begin
        replace_way_idx = '0;
        for (int i = 0; i < ASSOCIATIVITY; i++) begin
            if (lru_counter[mshr_index][i] == '0) begin
                replace_way_idx = i[3:0];
                break;
            end
        end
    end
    
    // Check if line needs writeback
    function automatic logic need_writeBack(
        input logic [INDEX_BITS-1:0] index,
        input logic [3:0] way_idx
    );
        return cache_mem[index][way_idx].valid && 
               cache_mem[index][way_idx].dirty && 
              (cache_mem[index][way_idx].coherence_state == MODIFIED || 
               cache_mem[index][way_idx].coherence_state == OWNED);
    endfunction
    
    // State machine - sequential logic
    always_ff @(posedge clk or negedge reset_n) begin
        if (~reset_n) begin
            state <= IDLE;
        end else if (flush) begin
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
                if (service_noc) begin
                    // Handle NoC coherence request
                    next_state = COHERENCE_REQUEST;
                end else if (service_l1d) begin
                    // Handle L1D request
                    next_state = TAG_CHECK;
                    mshr.is_l1i = 1'b0;
                end else if (service_l1i) begin
                    // Handle L1I request
                    next_state = TAG_CHECK;
                    mshr.is_l1i = 1'b1;
                end
            end
            
            TAG_CHECK: begin
                if ((mshr.is_l1i && l1i_cache_hit) ||
                    (!mshr.is_l1i && l1d_cache_hit)) begin
                    // Hit path
                    if (mshr.is_l1i) begin
                        // L1I hits are always reads
                        next_state = SERVICE_L1I;
                    end else if (l1d_req_write) begin
                        // L1D write hit - need to check coherence first
                        // For a write hit in L2, we need to invalidate any copies in L1
                        next_state = SNOOP_L1;
                    end else begin
                        // L1D read hit
                        next_state = SERVICE_L1D;
                    end
                end else begin
                    // Miss path - check if need to evict
                    if (need_writeBack(mshr.is_l1i ? l1i_index : l1d_index, replace_way_idx)) begin
                        next_state = EVICT_PREPARE;
                    end else begin
                        // Clean miss
                        next_state = REFILL_REQUEST;
                    end
                end
            end
            
            EVICT_PREPARE: begin
                // Prepare line for eviction
                next_state = EVICT_WRITE;
            end
            
            EVICT_WRITE: begin
                // Write back dirty line to memory
                if (mem_req_ready && mem_req_valid) begin
                    next_state = REFILL_REQUEST;
                end
            end
            
            REFILL_REQUEST: begin
                // Send memory request and also check directory
                next_state = REFILL_WAIT;
            end
            
            REFILL_WAIT: begin
                // Wait for memory response
                if (mem_resp_valid && mem_resp_last) begin
                    // Before servicing, may need to invalidate copies in L1
                    if (mshr.write) begin
                        next_state = SNOOP_L1;
                    end else if (mshr.is_l1i) begin
                        next_state = SERVICE_L1I;
                    end else begin
                        next_state = SERVICE_L1D;
                    end
                end
            end
            
            COHERENCE_REQUEST: begin
                // Process NoC coherence request
                // Many coherence requests require snooping L1 caches
                next_state = SNOOP_L1;
            end
            
            COHERENCE_WAIT: begin
                // Wait for coherence protocol operation to complete
                next_state = UPDATE_STATE;
            end
            
            SNOOP_L1: begin
                // Send snoop requests to L1 caches
                next_state = SNOOP_WAIT;
            end
            
            SNOOP_WAIT: begin
                // Wait for L1 snoop responses
                if (mshr.l1i_snooped && mshr.l1d_snooped) begin
                    // All snoop responses received
                    if (state == COHERENCE_REQUEST) begin
                        // Complete NoC request
                        next_state = UPDATE_STATE;
                    end else if (mshr.write) begin
                        // For write requests
                        next_state = mshr.is_l1i ? SERVICE_L1I : SERVICE_L1D;
                    end else begin
                        // For invalidation-only operations
                        next_state = UPDATE_STATE;
                    end
                end
            end
            
            UPDATE_STATE: begin
                // Update cache state based on coherence operation
                next_state = COMPLETE;
            end
            
            SERVICE_L1I: begin
                // Send response to L1I
                next_state = COMPLETE;
            end
            
            SERVICE_L1D: begin
                // Send response to L1D
                next_state = COMPLETE;
            end
            
            COMPLETE: begin
                // Operation complete, return to idle
                next_state = IDLE;
            end
        endcase
    end
    
    // Main datapath and control logic
    always_ff @(posedge clk or negedge reset_n) begin
        if (~reset_n) begin
            // Reset all outputs and internal state
            l1i_resp_valid <= 1'b0;
            l1i_resp_data <= '{default: '0};
            l1i_resp_ready <= 1'b0;
            
            l1d_resp_valid <= 1'b0;
            l1d_resp_data <= '{default: '0};
            l1d_resp_ready <= 1'b0;
            
            mem_req_valid <= 1'b0;
            mem_req_write <= 1'b0;
            mem_req_addr <= '0;
            mem_req_data <= '{default: '0};
            mem_req_ready <= 1'b0;
            
            noc_resp_valid <= 1'b0;
            noc_resp_type <= '0;
            noc_resp_addr <= '0;
            noc_resp_dest <= '0;
            noc_resp_data <= '{default: '0};
            
            l1i_snoop_req_valid <= 1'b0;
            l1i_snoop_req_type <= '0;
            l1i_snoop_req_addr <= '0;
            
            l1d_snoop_req_valid <= 1'b0;
            l1d_snoop_req_type <= '0;
            l1d_snoop_req_addr <= '0;
            
            dir_req_valid <= 1'b0;
            dir_req_addr <= '0;
            dir_req_type <= '0;
            
            wb_buffer_valid <= 1'b0;
            wb_buffer_addr <= '0;
            wb_buffer_data <= '{default: '0};
            
            mshr.valid <= 1'b0;
            mshr.state <= MSHR_IDLE;
            mshr.addr <= '0;
            mshr.write <= 1'b0;
            mshr.is_l1i <= 1'b0;
            mshr.coherence_state <= INVALID;
            mshr.sharers <= '0;
            mshr.l1i_snooped <= 1'b0;
            mshr.l1d_snooped <= 1'b0;
            mshr.l1i_snoop_result <= '0;
            mshr.l1d_snoop_result <= '0;
            mshr.way <= '0;
            
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
            // Default values for signals that need to be cleared each cycle
            l1i_resp_valid <= 1'b0;
            l1d_resp_valid <= 1'b0;
            mem_req_valid <= 1'b0;
            noc_resp_valid <= 1'b0;
            dir_req_valid <= 1'b0;
            l1i_snoop_req_valid <= 1'b0;
            l1d_snoop_req_valid <= 1'b0;
            
            // Process L1 snoop responses
            if (l1i_snoop_valid) begin
                mshr.l1i_snooped <= 1'b1;
                mshr.l1i_snoop_result <= l1i_snoop_type;
                
                // If L1I has modified data, capture it
                if (l1i_snoop_type == SNOOP_HIT_MODIFIED) begin
                    for (int w = 0; w < 8; w++) begin
                        mshr_write_data[w] <= l1i_snoop_data[w];
                    end
                end
            end
            
            if (l1d_snoop_valid) begin
                mshr.l1d_snooped <= 1'b1;
                mshr.l1d_snoop_result <= l1d_snoop_type;
                
                // If L1D has modified data, capture it
                if (l1d_snoop_type == SNOOP_HIT_MODIFIED) begin
                    for (int w = 0; w < 8; w++) begin
                        mshr_write_data[w] <= l1d_snoop_data[w];
                    end
                end
            end
            
            // Main state machine
            case (state)
                IDLE: begin
                    // Ready to accept new requests
                    l1i_resp_ready <= 1'b1;
                    l1d_resp_ready <= 1'b1;
                    
                    // Initialize MSHR for new request
                    if (service_noc) begin
                        // NoC request
                        mshr.valid <= 1'b1;
                        mshr.state <= MSHR_IDLE;
                        mshr.addr <= noc_req_addr;
                        mshr.write <= (noc_req_type == NOC_WRITE_REQ);
                        mshr.is_l1i <= 1'b0; // NoC requests never from I-cache
                        
                        // Store data if it's a write request
                        if (noc_req_type == NOC_WRITE_REQ) begin
                            for (int w = 0; w < 8; w++) begin
                                mshr_write_data[w] <= noc_req_data[w];
                            end
                        end
                        
                        // Determine target coherence state based on request type
                        case (noc_req_type)
                            NOC_READ_REQ:      mshr.coherence_state <= SHARED;
                            NOC_READ_EXCL_REQ: mshr.coherence_state <= EXCLUSIVE;
                            NOC_WRITE_REQ:     mshr.coherence_state <= MODIFIED;
                            NOC_INVALIDATE_REQ: mshr.coherence_state <= INVALID;
                            default:           mshr.coherence_state <= INVALID;
                        endcase
                        
                        // Reset snoop tracking
                        mshr.l1i_snooped <= 1'b0;
                        mshr.l1d_snooped <= 1'b0;
                        mshr.l1i_snoop_result <= SNOOP_NOHIT;
                        mshr.l1d_snoop_result <= SNOOP_NOHIT;
                    end else if (service_l1i) begin
                        // L1I request (always read)
                        mshr.valid <= 1'b1;
                        mshr.state <= MSHR_IDLE;
                        mshr.addr <= l1i_req_addr;
                        mshr.write <= 1'b0; // I-cache requests are always reads
                        mshr.is_l1i <= 1'b1;
                        
                        // Determine target coherence state (SHARED for I-cache)
                        mshr.coherence_state <= SHARED;
                        
                        // Reset snoop tracking
                        mshr.l1i_snooped <= 1'b0;
                        mshr.l1d_snooped <= 1'b0;
                        mshr.l1i_snoop_result <= SNOOP_NOHIT;
                        mshr.l1d_snoop_result <= SNOOP_NOHIT;
                    end
                end
                
                TAG_CHECK: begin
                    // Not ready to accept new requests during processing
                    l1i_resp_ready <= 1'b0;
                    l1d_resp_ready <= 1'b0;
                    
                    if ((mshr.is_l1i && l1i_cache_hit) || (!mshr.is_l1i && l1d_cache_hit)) begin
                        // Hit case
                        mshr.way <= mshr.is_l1i ? l1i_hit_way_idx : l1d_hit_way_idx;
                    end else begin
                        // Miss case
                        mshr.way <= replace_way_idx;
                        
                        // Store address in MSHR
                        mshr.addr <= mshr.is_l1i ? l1i_req_addr : l1d_req_addr;
                    end
                end
                
                EVICT_PREPARE: begin
                    // Prepare dirty line for writeback
                    if (!wb_buffer_valid) begin
                        wb_buffer_valid <= 1'b1;
                        wb_buffer_addr <= {cache_mem[mshr_index][mshr.way].tag,
                                           mshr_index,
                                           {OFFSET_BITS{1'b0}}};
                        
                        // Convert data format from cache line to word array
                        for (int w = 0; w < 8; w++) begin
                            wb_buffer_data[w] <= cache_mem[mshr_index][mshr.way].data[w*64 +: 64];
                        end
                    end
                end
                
                EVICT_WRITE: begin
                    // Write back dirty line to memory
                    if (wb_buffer_valid) begin
                        mem_req_valid <= 1'b1;
                        mem_req_write <= 1'b1;
                        mem_req_addr <= wb_buffer_addr;
                        mem_req_data <= wb_buffer_data;
                        
                        // Clear buffer when request is accepted
                        if (mem_req_ready) begin
                            wb_buffer_valid <= 1'b0;
                            
                            // Mark cache line as invalid after writeback starts
                            cache_mem[mshr_index][mshr.way].valid <= 1'b0;
                            cache_mem[mshr_index][mshr.way].dirty <= 1'b0;
                            cache_mem[mshr_index][mshr.way].coherence_state <= INVALID;
                        end
                    end
                end
                
                REFILL_REQUEST: begin
                    // Send memory request for the missing line
                    mem_req_valid <= 1'b1;
                    mem_req_write <= 1'b0;
                    // Align to cache line boundary
                    mem_req_addr <= {mshr.addr[47:OFFSET_BITS], {OFFSET_BITS{1'b0}}};
                    mem_req_ready <= 1'b1;
                    
                    // Also check directory to get coherence information
                    dir_req_valid <= 1'b1;
                    dir_req_addr <= mshr.addr;
                    
                    // Request type based on the access
                    if (mshr.write) begin
                        // Need exclusive access for writes
                        dir_req_type <= 3'b001; // Request exclusive
                    end else begin
                        // Read access
                        dir_req_type <= 3'b000; // Request shared
                    end
                    
                    mshr.state <= MSHR_WAITING_MEM;
                end
                
                REFILL_WAIT: begin
                    // Wait for memory response
                    if (mem_resp_valid) begin
                        // Store response in cache
                        cache_mem[mshr_index][mshr.way].valid <= 1'b1;
                        cache_mem[mshr_index][mshr.way].tag <= mshr_tag;
                        
                        // Convert data format from word array to cache line
                        for (int w = 0; w < 8; w++) begin
                            cache_mem[mshr_index][mshr.way].data[w*64 +: 64] <= mem_resp_data[w];
                        end
                        
                        // Update coherence state
                        cache_mem[mshr_index][mshr.way].coherence_state <= mshr.coherence_state;
                        
                        // Mark as dirty if it's a write
                        cache_mem[mshr_index][mshr.way].dirty <= mshr.write;
                        
                        // Update LRU
                        for (int w = 0; w < ASSOCIATIVITY; w++) begin
                            if (w == mshr.way) begin
                                lru_counter[mshr_index][w] <= ASSOCIATIVITY - 1;
                            end else if (lru_counter[mshr_index][w] > 0) begin
                                lru_counter[mshr_index][w] <= lru_counter[mshr_index][w] - 1'b1;
                            end
                        end
                        
                        // If directory response received, update sharers
                        if (dir_resp_valid) begin
                            mshr.sharers <= dir_resp_sharers;
                        end
                    end
                end
                
                COHERENCE_REQUEST: begin
                    // Process NoC coherence request
                    if (noc_cache_hit) begin
                        // Cache hit for coherence request
                        mshr.way <= noc_hit_way_idx;
                        
                        case (noc_req_type)
                            NOC_READ_REQ: begin
                                // Read request - respond with data
                                noc_resp_valid <= 1'b1;
                                noc_resp_type <= NOC_DATA_RESP;
                                noc_resp_addr <= noc_req_addr;
                                noc_resp_dest <= noc_req_source;
                                
                                // Convert data from cache line to word array
                                for (int w = 0; w < 8; w++) begin
                                    noc_resp_data[w] <= cache_mem[noc_index][noc_hit_way_idx].data[w*64 +: 64];
                                end
                                
                                // Update coherence state if necessary
                                if (cache_mem[noc_index][noc_hit_way_idx].coherence_state == EXCLUSIVE ||
                                    cache_mem[noc_index][noc_hit_way_idx].coherence_state == MODIFIED) begin
                                    cache_mem[noc_index][noc_hit_way_idx].coherence_state <= SHARED;
                                    cache_mem[noc_index][noc_hit_way_idx].dirty <= 1'b0;
                                end
                            end
                            
                            NOC_READ_EXCL_REQ: begin
                                // Read exclusive request - respond with data and invalidate local copy
                                noc_resp_valid <= 1'b1;
                                noc_resp_type <= NOC_DATA_EXCL_RESP;
                                noc_resp_addr <= noc_req_addr;
                                noc_resp_dest <= noc_req_source;
                                
                                // Convert data from cache line to word array
                                for (int w = 0; w < 8; w++) begin
                                    noc_resp_data[w] <= cache_mem[noc_index][noc_hit_way_idx].data[w*64 +: 64];
                                end
                                
                                // Will invalidate after checking if L1 has copies
                            end
                            
                            NOC_WRITE_REQ: begin
                                // Write request - update local copy and invalidate L1 copies
                                // First respond with ACK
                                noc_resp_valid <= 1'b1;
                                noc_resp_type <= NOC_ACK_RESP;
                                noc_resp_addr <= noc_req_addr;
                                noc_resp_dest <= noc_req_source;
                                
                                // Update data with the write data
                                for (int w = 0; w < 8; w++) begin
                                    cache_mem[noc_index][noc_hit_way_idx].data[w*64 +: 64] <= noc_req_data[w];
                                end
                                
                                // Mark as modified and dirty
                                cache_mem[noc_index][noc_hit_way_idx].coherence_state <= MODIFIED;
                                cache_mem[noc_index][noc_hit_way_idx].dirty <= 1'b1;
                            end
                            
                            NOC_INVALIDATE_REQ: begin
                                // Invalidate request - respond with ACK
                                noc_resp_valid <= 1'b1;
                                noc_resp_type <= NOC_ACK_RESP;
                                noc_resp_addr <= noc_req_addr;
                                noc_resp_dest <= noc_req_source;
                                
                                // Will invalidate after checking if L1 has copies
                            end
                        endcase
                    end else begin
                        // Cache miss for coherence request
                        noc_resp_valid <= 1'b1;
                        noc_resp_type <= NOC_ACK_RESP;
                        noc_resp_addr <= noc_req_addr;
                        noc_resp_dest <= noc_req_source;
                    end
                end
                
                COHERENCE_WAIT: begin
                    // Wait for coherence protocol operation to complete
                    if (dir_resp_valid) begin
                        mshr.sharers <= dir_resp_sharers;
                    end
                end
                
                SNOOP_L1: begin
                    // Send snoop requests to L1 caches for coherence
                    if (!mshr.l1i_snooped) begin
                        l1i_snoop_req_valid <= 1'b1;
                        l1i_snoop_req_addr <= mshr.addr;
                        
                        // Set appropriate snoop type based on request
                        if (noc_req_type == NOC_INVALIDATE_REQ || 
                            noc_req_type == NOC_READ_EXCL_REQ ||
                            mshr.write) begin
                            // Need to invalidate
                            l1i_snoop_req_type <= 3'b001; // Invalidate
                        end else begin
                            // Just checking for hits
                            l1i_snoop_req_type <= 3'b000; // Read
                        end
                    end
                    
                    if (!mshr.l1d_snooped) begin
                        l1d_snoop_req_valid <= 1'b1;
                        l1d_snoop_req_addr <= mshr.addr;
                        
                        // Set appropriate snoop type based on request
                        if (noc_req_type == NOC_INVALIDATE_REQ || 
                            noc_req_type == NOC_READ_EXCL_REQ ||
                            mshr.write) begin
                            // Need to invalidate
                            l1d_snoop_req_type <= 3'b001; // Invalidate
                        end else begin
                            // Just checking for hits
                            l1d_snoop_req_type <= 3'b000; // Read
                        end
                    end
                end
                
                SNOOP_WAIT: begin
                    // Wait for L1 snoop responses
                    // Already handled in the response processing at the beginning
                end
                
                UPDATE_STATE: begin
                    // Update cache line state based on coherence operation
                    if (noc_cache_hit && 
                        (noc_req_type == NOC_INVALIDATE_REQ || noc_req_type == NOC_READ_EXCL_REQ)) begin
                        // Invalidate local copy
                        cache_mem[noc_index][noc_hit_way_idx].valid <= 1'b0;
                        cache_mem[noc_index][noc_hit_way_idx].dirty <= 1'b0;
                        cache_mem[noc_index][noc_hit_way_idx].coherence_state <= INVALID;
                    end
                    
                    // Update directory if needed
                    if (mshr.write) begin
                        dir_req_valid <= 1'b1;
                        dir_req_addr <= mshr.addr;
                        dir_req_type <= 3'b010; // Update ownership
                    end
                    
                    // Reset MSHR state
                    mshr.l1i_snooped <= 1'b0;
                    mshr.l1d_snooped <= 1'b0;
                end
                
                SERVICE_L1I: begin
                    // Send response to L1I
                    l1i_resp_valid <= 1'b1;
                    
                    // Get data from the cache line
                    for (int w = 0; w < 8; w++) begin
                        l1i_resp_data[w] <= mshr_cache_hit ? 
                            cache_mem[mshr_index][mshr.way].data[w*64 +: 64] : 
                            mem_resp_data[w];
                    end
                    
                    // Update LRU on hit
                    if (mshr_cache_hit) begin
                        for (int w = 0; w < ASSOCIATIVITY; w++) begin
                            if (w == mshr.way) begin
                                lru_counter[mshr_index][w] <= ASSOCIATIVITY - 1;
                            end else if (lru_counter[mshr_index][w] > lru_counter[mshr_index][mshr.way]) begin
                                lru_counter[mshr_index][w] <= lru_counter[mshr_index][w] - 1'b1;
                            end
                        end
                    end
                end
                
                SERVICE_L1D: begin
                    // Send response to L1D
                    l1d_resp_valid <= 1'b1;
                    
                    // Get data from the cache line
                    for (int w = 0; w < 8; w++) begin
                        l1d_resp_data[w] <= mshr_cache_hit ? 
                            cache_mem[mshr_index][mshr.way].data[w*64 +: 64] : 
                            mem_resp_data[w];
                    end
                    
                    // For write requests, update the cache line
                    if (mshr.write) begin
                        for (int w = 0; w < 8; w++) begin
                            cache_mem[mshr_index][mshr.way].data[w*64 +: 64] <= mshr_write_data[w];
                        end
                        
                        // Mark as dirty and modified
                        cache_mem[mshr_index][mshr.way].dirty <= 1'b1;
                        cache_mem[mshr_index][mshr.way].coherence_state <= MODIFIED;
                        
                        // Update directory to reflect ownership change
                        dir_req_valid <= 1'b1;
                        dir_req_addr <= mshr.addr;
                        dir_req_type <= 3'b010; // Update ownership
                    end
                    
                    // Update LRU on hit
                    if (mshr_cache_hit) begin
                        for (int w = 0; w < ASSOCIATIVITY; w++) begin
                            if (w == mshr.way) begin
                                lru_counter[mshr_index][w] <= ASSOCIATIVITY - 1;
                            end else if (lru_counter[mshr_index][w] > lru_counter[mshr_index][mshr.way]) begin
                                lru_counter[mshr_index][w] <= lru_counter[mshr_index][w] - 1'b1;
                            end
                        end
                    end
                end
                
                COMPLETE: begin
                    // Clean up MSHR state
                    mshr.valid <= 1'b0;
                    mshr.state <= MSHR_IDLE;
                    
                    // Ready for new requests
                    l1i_resp_ready <= 1'b1;
                    l1d_resp_ready <= 1'b1;
                end
            endcase
            
            // Handle cache flush
            if (flush) begin
                // Process all dirty lines (simplified)
                for (int set = 0; set < SETS; set++) begin
                    for (int w = 0; w < ASSOCIATIVITY; w++) begin
                        if (cache_mem[set][w].valid && cache_mem[set][w].dirty) begin
                            // In a real implementation, would need to write back
                            // Rather than invalidating immediately
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
                
                // Reset MSHR and buffer state
                mshr.valid <= 1'b0;
                mshr.state <= MSHR_IDLE;
                wb_buffer_valid <= 1'b0;
                
                // Reset interface signals
                l1i_resp_ready <= 1'b1;
                l1d_resp_ready <= 1'b1;
                l1i_resp_valid <= 1'b0;
                l1d_resp_valid <= 1'b0;
                mem_req_valid <= 1'b0;
                noc_resp_valid <= 1'b0;
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
    
    function automatic string cache_state_to_string(cache_state_t state);
        case (state)
            IDLE:             return "IDLE";
            TAG_CHECK:        return "TAG_CHECK";
            EVICT_PREPARE:    return "EVICT_PREP";
            EVICT_WRITE:      return "EVICT_WRITE";
            REFILL_REQUEST:   return "REFILL_REQ";
            REFILL_WAIT:      return "REFILL_WAIT";
            COHERENCE_REQUEST: return "COH_REQ";
            COHERENCE_WAIT:   return "COH_WAIT";
            SNOOP_L1:         return "SNOOP_L1";
            SNOOP_WAIT:       return "SNOOP_WAIT";
            UPDATE_STATE:     return "UPDATE_STATE";
            SERVICE_L1I:      return "SERVICE_L1I";
            SERVICE_L1D:      return "SERVICE_L1D";
            COMPLETE:         return "COMPLETE";
            default:          return "UNKNOWN";
        endcase
    endfunction
    
endmodule