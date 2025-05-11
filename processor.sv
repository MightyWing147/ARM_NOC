module arm_core #(
    parameter CORE_ID = 0
)(
    input  logic        clk,
    input  logic        reset_n,
    
    // Memory interface (D-Cache)
    output logic [47:0] mem_addr,
    output logic        mem_read,
    output logic        mem_write,
    output logic [63:0] mem_wdata,
    output logic [7:0]  mem_byte_en,
    input  logic [63:0] mem_rdata,
    input  logic        mem_ready,
    
    // Instruction fetch interface (I-Cache)
    output logic [47:0] instr_addr,
    output logic        instr_read,
    input  logic [31:0] instr_rdata,
    input  logic        instr_ready,
    
    // Debug interface
    input  logic        debug_halt,
    input  logic        debug_run,
    input  logic        debug_step,
    input  logic [47:0] debug_pc,
    input  logic        debug_pc_wr_en,
    
    // Status interface
    output logic [3:0]  core_state,
    output logic        halted,
    
    // Interrupt interface
    input  logic [3:0]  interrupt
);

    // -----------------------------------------------------------------------
    // Internal Signals and Parameters
    // -----------------------------------------------------------------------
    // Cache interfaces
    logic [47:0] l1i_addr;
    logic        l1i_read;
    logic        l1i_write;  // Always 0 for instruction cache
    logic [63:0] l1i_wdata;  // Not used for instruction cache
    logic [7:0]  l1i_byte_en;// Not used for instruction cache
    logic [63:0] l1i_rdata;
    logic        l1i_ready;
    
    logic [47:0] l1d_addr;
    logic        l1d_read;
    logic        l1d_write;
    logic [63:0] l1d_wdata;
    logic [7:0]  l1d_byte_en;
    logic [63:0] l1d_rdata;
    logic        l1d_ready;
    
    // Network Interface to L1 Cache
    // I-Cache request interface
    logic [47:0] l1i_ni_req_addr;
    logic [63:0] l1i_ni_req_data;
    logic [7:0]  l1i_ni_req_type;
    logic        l1i_ni_req_valid;
    logic        l1i_ni_req_ready;
    
    // I-Cache response interface
    logic [63:0] l1i_ni_resp_data;
    logic [7:0]  l1i_ni_resp_type;
    logic        l1i_ni_resp_valid;
    logic        l1i_ni_resp_ready;
    
    // D-Cache request interface
    logic [47:0] l1d_ni_req_addr;
    logic [63:0] l1d_ni_req_data;
    logic [7:0]  l1d_ni_req_type;
    logic        l1d_ni_req_valid;
    logic        l1d_ni_req_ready;
    
    // D-Cache response interface
    logic [63:0] l1d_ni_resp_data;
    logic [7:0]  l1d_ni_resp_type;
    logic        l1d_ni_resp_valid;
    logic        l1d_ni_resp_ready;
    
    // Coherence interface for I-Cache
    logic [47:0] l1i_coh_req_addr;
    logic [2:0]  l1i_coh_req_type;
    logic [7:0]  l1i_coh_req_target;
    logic        l1i_coh_req_valid;
    logic        l1i_coh_req_ready;
    
    logic [47:0] l1i_coh_resp_addr;
    logic [2:0]  l1i_coh_resp_type;
    logic [7:0]  l1i_coh_resp_source;
    logic [63:0] l1i_coh_resp_data;
    logic        l1i_coh_resp_valid;
    logic        l1i_coh_resp_ready;
    
    // Coherence interface for D-Cache
    logic [47:0] l1d_coh_req_addr;
    logic [2:0]  l1d_coh_req_type;
    logic [7:0]  l1d_coh_req_target;
    logic        l1d_coh_req_valid;
    logic        l1d_coh_req_ready;
    
    logic [47:0] l1d_coh_resp_addr;
    logic [2:0]  l1d_coh_resp_type;
    logic [7:0]  l1d_coh_resp_source;
    logic [63:0] l1d_coh_resp_data;
    logic        l1d_coh_resp_valid;
    logic        l1d_coh_resp_ready;
    
    // NoC router interface signals
    logic [2:0][127:0] router_flit_out;      // Output flits to NoC (3 virtual channels)
    logic [2:0]        router_valid_out;     // Valid signals for output flits
    logic [2:0]        router_ready_in;      // Ready signals from NoC router
    
    logic [2:0][127:0] router_flit_in;       // Input flits from NoC
    logic [2:0]        router_valid_in;      // Valid signals for input flits
    logic [2:0]        router_ready_out;     // Ready signals to NoC router
    
    // I-Cache router interface signals
    logic [2:0][127:0] i_router_flit_out;
    logic [2:0]        i_router_valid_out;
    logic [2:0]        i_router_ready_in;
    
    logic [2:0][127:0] i_router_flit_in;
    logic [2:0]        i_router_valid_in;
    logic [2:0]        i_router_ready_out;
    
    // D-Cache router interface signals
    logic [2:0][127:0] d_router_flit_out;
    logic [2:0]        d_router_valid_out;
    logic [2:0]        d_router_ready_in;
    
    logic [2:0][127:0] d_router_flit_in;
    logic [2:0]        d_router_valid_in;
    logic [2:0]        d_router_ready_out;
    
    // Arbiter state and control
    typedef enum logic [1:0] {
        ARB_IDLE,
        ARB_ICACHE,
        ARB_DCACHE
    } arbiter_state_t;
    
    arbiter_state_t arb_state, next_arb_state;
    logic [1:0] arb_priority_counter;
    
    // -----------------------------------------------------------------------
    // ARM Core Instantiation
    // -----------------------------------------------------------------------
    arm_core #(
        .CORE_ID(CORE_ID)
    ) core_inst (
        .clk(clk),
        .reset_n(reset_n),
        
        // Memory interface
        .mem_addr(l1d_addr),
        .mem_read(l1d_read),
        .mem_write(l1d_write),
        .mem_wdata(l1d_wdata),
        .mem_byte_en(l1d_byte_en),
        .mem_rdata(l1d_rdata),
        .mem_ready(l1d_ready),
        
        // Instruction fetch interface
        .instr_addr(l1i_addr),
        .instr_read(l1i_read),
        .instr_rdata(instr_rdata),   // Connect directly to top-level interface
        .instr_ready(l1i_ready),
        
        // Debugging interface
        .debug_halt(debug_halt),
        .debug_run(debug_run),
        .debug_step(debug_step),
        .debug_pc(debug_pc),
        .debug_pc_wr_en(debug_pc_wr_en),
        
        // Status interface
        .core_state(core_state),
        .halted(halted),
        
        // Interrupt interface
        .interrupt(interrupt)
    );
    
    // -----------------------------------------------------------------------
    // L1 Instruction Cache Instantiation
    // -----------------------------------------------------------------------
    l1icache l1i_cache_inst (
        .clk(clk),
        .reset_n(reset_n),
        
        // CPU interface
        .cpu_req_valid(l1i_read),
        .cpu_req_addr(l1i_addr),
        .flush(1'b0),  // No flush signal connected
        
        // Memory/NoC data interface
        .mem_resp_valid(l1i_ni_resp_valid),
        .mem_resp_data({8{l1i_ni_resp_data[31:0]}}),  // Replicate 32-bit data for line
        .mem_resp_last(1'b1),  // Simplified for now
        
        // CPU interface outputs
        .cpu_resp_valid(l1i_ready),
        .cpu_resp_data(instr_rdata),  // 32-bit instruction output
        .cpu_resp_ready(1'b1),        // Always ready for CPU responses
        
        // Memory/NoC request interface
        .mem_req_valid(l1i_ni_req_valid),
        .mem_req_addr(l1i_ni_req_addr),
        .mem_req_ready(l1i_ni_req_ready),
        
        // NoC coherence interface inputs
        .noc_req_valid(l1i_coh_resp_valid),
        .noc_req_type(l1i_coh_resp_type),
        .noc_req_addr(l1i_coh_resp_addr),
        .noc_req_source(l1i_coh_resp_source),
        .noc_req_dest(8'h0),  // Not used in response
        .noc_req_data(l1i_coh_resp_data),
        
        // NoC coherence interface outputs
        .noc_resp_valid(l1i_coh_req_valid),
        .noc_resp_type(l1i_coh_req_type),
        .noc_resp_addr(l1i_coh_req_addr),
        .noc_resp_dest(l1i_coh_req_target),
        .noc_resp_data(64'h0),  // No data for most coherence responses
        
        // Directory interface (optional)
        .dir_resp_valid(1'b0),        // Not directly connected
        .dir_resp_sharers(8'h0),      // Not directly connected
        .dir_resp_owner(3'h0),        // Not directly connected
        .dir_req_valid(),             // Not directly connected
        .dir_req_addr(),              // Not directly connected
        .dir_req_type(),              // Not directly connected
        
        // Configuration
        .core_id(CORE_ID[2:0]),
        .exclusive_inst_fetch(1'b0)   // Don't get exclusive access for instructions
    );
    
    // -----------------------------------------------------------------------
    // L1 Data Cache Instantiation
    // -----------------------------------------------------------------------
    l1dcache l1d_cache_inst (
        .clk(clk),
        .reset_n(reset_n),
        
        // CPU interface
        .cpu_req_valid(l1d_read || l1d_write),
        .cpu_req_addr(l1d_addr),
        .cpu_req_write(l1d_write),
        .cpu_req_byte_en(l1d_byte_en),
        .cpu_req_data(l1d_wdata),
        .cpu_req_size(1'b1),            // 1 = 64-bit, 0 = 32-bit
        .cpu_req_atomic(1'b0),          // No atomic operations for now
        .cpu_req_atomic_op(4'h0),       // No atomic operations for now
        .cpu_req_exclusive(1'b0),       // No exclusive access for now
        .flush(1'b0),                   // No flush for now
        
        .cpu_resp_valid(l1d_ready),
        .cpu_resp_data(l1d_rdata),
        .cpu_resp_ready(1'b1),          // Always ready to receive CPU responses
        .cpu_resp_error(),              // Not connected for now
        
        // Memory/NoC interface
        .mem_req_valid(l1d_ni_req_valid),
        .mem_req_write(l1d_write),
        .mem_req_addr(l1d_ni_req_addr),
        .mem_req_data({8{l1d_wdata}}),  // Replicate data for cache line
        .mem_req_ready(l1d_ni_req_ready),
        .mem_req_atomic(1'b0),          // No atomic operations for now
        
        .mem_resp_valid(l1d_ni_resp_valid),
        .mem_resp_data({8{l1d_ni_resp_data}}),  // Replicate for full cache line
        .mem_resp_last(1'b1),           // Simplified for now
        .mem_resp_error(1'b0),          // No errors for now
        
        // NoC coherence interface
        .noc_req_valid(l1d_coh_resp_valid),
        .noc_req_type(l1d_coh_resp_type),
        .noc_req_addr(l1d_coh_resp_addr),
        .noc_req_source(l1d_coh_resp_source),
        .noc_req_dest(8'h0),            // Not used in response
        .noc_req_data(l1d_coh_resp_data),
        
        .noc_resp_valid(l1d_coh_req_valid),
        .noc_resp_type(l1d_coh_req_type),
        .noc_resp_addr(l1d_coh_req_addr),
        .noc_resp_dest(l1d_coh_req_target),
        .noc_resp_data(64'h0),          // No data for most coherence responses
        
        // Directory interface
        .dir_resp_valid(1'b0),          // Not directly connected
        .dir_resp_sharers(8'h0),        // Not directly connected
        .dir_resp_owner(3'h0),          // Not directly connected
        .dir_req_valid(),               // Not directly connected
        .dir_req_addr(),                // Not directly connected
        .dir_req_type(),                // Not directly connected
        
        // Configuration
        .core_id(CORE_ID[2:0])
    );
    
    // -----------------------------------------------------------------------
    // Network Interface for I-Cache
    // -----------------------------------------------------------------------
    network_interface #(
        .CORE_ID({CORE_ID, 1'b0}),      // Differentiate I-cache (even ID)
        .ADDR_WIDTH(48),
        .DATA_WIDTH(64),
        .FLIT_WIDTH(128),
        .VC_COUNT(3)
    ) l1i_ni_inst (
        .clk(clk),
        .reset_n(reset_n),
        
        // Cache Controller Interface
        .cache_req_addr(l1i_ni_req_addr),
        .cache_req_data(l1i_ni_req_data),
        .cache_req_type(l1i_ni_req_type),
        .cache_req_valid(l1i_ni_req_valid),
        .cache_req_ready(l1i_ni_req_ready),
        
        .cache_resp_data(l1i_ni_resp_data),
        .cache_resp_type(l1i_ni_resp_type),
        .cache_resp_valid(l1i_ni_resp_valid),
        .cache_resp_ready(l1i_ni_resp_ready),
        
        // Coherence Interface
        .coh_req_addr(l1i_coh_req_addr),
        .coh_req_type(l1i_coh_req_type),
        .coh_req_target(l1i_coh_req_target),
        .coh_req_valid(l1i_coh_req_valid),
        .coh_req_ready(l1i_coh_req_ready),
        
        .coh_resp_addr(l1i_coh_resp_addr),
        .coh_resp_type(l1i_coh_resp_type),
        .coh_resp_source(l1i_coh_resp_source),
        .coh_resp_data(l1i_coh_resp_data),
        .coh_resp_valid(l1i_coh_resp_valid),
        .coh_resp_ready(l1i_coh_resp_ready),
        
        // NoC Router Interface - will be connected to arbiter
        .router_flit_out(i_router_flit_out),
        .router_valid_out(i_router_valid_out),
        .router_ready_in(i_router_ready_in),
        
        .router_flit_in(i_router_flit_in),
        .router_valid_in(i_router_valid_in),
        .router_ready_out(i_router_ready_out)
    );
    
    // -----------------------------------------------------------------------
    // Network Interface for D-Cache
    // -----------------------------------------------------------------------
    network_interface #(
        .CORE_ID({CORE_ID, 1'b1}),      // Differentiate D-cache (odd ID)
        .ADDR_WIDTH(48),
        .DATA_WIDTH(64),
        .FLIT_WIDTH(128),
        .VC_COUNT(3)
    ) l1d_ni_inst (
        .clk(clk),
        .reset_n(reset_n),
        
        // Cache Controller Interface
        .cache_req_addr(l1d_ni_req_addr),
        .cache_req_data(l1d_ni_req_data),
        .cache_req_type(l1d_ni_req_type),
        .cache_req_valid(l1d_ni_req_valid),
        .cache_req_ready(l1d_ni_req_ready),
        
        .cache_resp_data(l1d_ni_resp_data),
        .cache_resp_type(l1d_ni_resp_type),
        .cache_resp_valid(l1d_ni_resp_valid),
        .cache_resp_ready(l1d_ni_resp_ready),
        
        // Coherence Interface
        .coh_req_addr(l1d_coh_req_addr),
        .coh_req_type(l1d_coh_req_type),
        .coh_req_target(l1d_coh_req_target),
        .coh_req_valid(l1d_coh_req_valid),
        .coh_req_ready(l1d_coh_req_ready),
        
        .coh_resp_addr(l1d_coh_resp_addr),
        .coh_resp_type(l1d_coh_resp_type),
        .coh_resp_source(l1d_coh_resp_source),
        .coh_resp_data(l1d_coh_resp_data),
        .coh_resp_valid(l1d_coh_resp_valid),
        .coh_resp_ready(l1d_coh_resp_ready),
        
        // NoC Router Interface - will be connected to arbiter
        .router_flit_out(d_router_flit_out),
        .router_valid_out(d_router_valid_out),
        .router_ready_in(d_router_ready_in),
        
        .router_flit_in(d_router_flit_in),
        .router_valid_in(d_router_valid_in),
        .router_ready_out(d_router_ready_out)
    );
    
    // -----------------------------------------------------------------------
    // NoC Interface Arbiter
    // -----------------------------------------------------------------------
    // Arbiter state machine to multiplex between I-cache and D-cache traffic
    always_ff @(posedge clk or negedge reset_n) begin
        if (~reset_n) begin
            arb_state <= ARB_IDLE;
            arb_priority_counter <= 2'b00;
        end else begin
            arb_state <= next_arb_state;
            
            // Update priority counter - used for fairness
            if (arb_state != next_arb_state) begin
                arb_priority_counter <= arb_priority_counter + 1'b1;
            end
        end
    end
    
    // Arbiter next state logic
    always_comb begin
        next_arb_state = arb_state;
        
        case (arb_state)
            ARB_IDLE: begin
                // Default priority is D-cache, but alternate based on counter for fairness
                if (arb_priority_counter[0]) begin
                    // Give priority to I-cache
                    if (|i_router_valid_out) begin
                        next_arb_state = ARB_ICACHE;
                    end else if (|d_router_valid_out) begin
                        next_arb_state = ARB_DCACHE;
                    end
                end else begin
                    // Give priority to D-cache
                    if (|d_router_valid_out) begin
                        next_arb_state = ARB_DCACHE;
                    end else if (|i_router_valid_out) begin
                        next_arb_state = ARB_ICACHE;
                    end
                end
            end
            
            ARB_ICACHE: begin
                // Stay with I-cache if it still has data and D-cache is not high priority
                if (|i_router_valid_out) begin
                    // If D-cache has been waiting, check if it should preempt
                    if (|d_router_valid_out && arb_priority_counter[1]) begin
                        next_arb_state = ARB_DCACHE;
                    end
                end else begin
                    // I-cache has no more data
                    if (|d_router_valid_out) begin
                        next_arb_state = ARB_DCACHE;
                    end else begin
                        next_arb_state = ARB_IDLE;
                    end
                end
            end
            
            ARB_DCACHE: begin
                // Stay with D-cache if it still has data and I-cache is not high priority
                if (|d_router_valid_out) begin
                    // If I-cache has been waiting, check if it should preempt
                    if (|i_router_valid_out && arb_priority_counter[1]) begin
                        next_arb_state = ARB_ICACHE;
                    end
                end else begin
                    // D-cache has no more data
                    if (|i_router_valid_out) begin
                        next_arb_state = ARB_ICACHE;
                    end else begin
                        next_arb_state = ARB_IDLE;
                    end
                end
            end
        endcase
    end
    
    // Arbiter output muxing - connect appropriate FIFOs based on state
    always_comb begin
        // Default assignments - no connections
        i_router_ready_in = 3'b000;
        d_router_ready_in = 3'b000;
        router_flit_out = '{default: '0};
        router_valid_out = 3'b000;
        
        i_router_flit_in = '{default: '0};
        i_router_valid_in = 3'b000;
        d_router_flit_in = '{default: '0};
        d_router_valid_in = 3'b000;
        
        case (arb_state)
            ARB_ICACHE: begin
                // Connect I-cache to NoC
                router_flit_out = i_router_flit_out;
                router_valid_out = i_router_valid_out;
                i_router_ready_in = router_ready_in;
                
                i_router_flit_in = router_flit_in;
                i_router_valid_in = router_valid_in;
                router_ready_out = i_router_ready_out;
            end
            
            ARB_DCACHE: begin
                // Connect D-cache to NoC
                router_flit_out = d_router_flit_out;
                router_valid_out = d_router_valid_out;
                d_router_ready_in = router_ready_in;
                
                d_router_flit_in = router_flit_in;
                d_router_valid_in = router_valid_in;
                router_ready_out = d_router_ready_out;
            end
            
            default: begin // ARB_IDLE
                // Accept incoming flits and demux based on address
                // This allows responses to come in even when not actively sending
                if (router_valid_in != 3'b000) begin
                    // Check first flit to determine destination (I-cache or D-cache)
                    // In each flit, bit [64] can indicate I or D cache (0 for I, 1 for D)
                    // This is an example - adapt based on your actual packet format
                    if (router_flit_in[0][64]) begin
                        // Send to D-cache
                        d_router_flit_in = router_flit_in;
                        d_router_valid_in = router_valid_in;
                        router_ready_out = d_router_ready_out;
                    end else begin
                        // Send to I-cache
                        i_router_flit_in = router_flit_in;
                        i_router_valid_in = router_valid_in;
                        router_ready_out = i_router_ready_out;
                    end
                end else begin
                    // No incoming traffic
                    router_ready_out = 3'b111; // Always ready in idle state
                end
            end
        endcase
    end
    
    // -----------------------------------------------------------------------
    // External Memory Interface Connections
    // -----------------------------------------------------------------------
    // Connect processor signals to external interface
    assign mem_addr = l1d_addr;
    assign mem_read = l1d_read;
    assign mem_write = l1d_write;
    assign mem_wdata = l1d_wdata;
    assign mem_byte_en = l1d_byte_en;
    
    assign instr_addr = l1i_addr;
    assign instr_read = l1i_read;
    
endmodule