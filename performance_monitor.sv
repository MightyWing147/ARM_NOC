module performance_monitor #(
    parameter NUM_CORES = 4
)(
    input  logic clk,
    input  logic reset_n,
    
    // Core event inputs
    input  logic [NUM_CORES-1:0] core_instruction_completed,
    input  logic [NUM_CORES-1:0] core_cache_hit_l1i,
    input  logic [NUM_CORES-1:0] core_cache_hit_l1d,
    input  logic [NUM_CORES-1:0] core_cache_miss_l1i,
    input  logic [NUM_CORES-1:0] core_cache_miss_l1d,
    
    // L2 cache events
    input  logic l2_cache_hit,
    input  logic l2_cache_miss,
    
    // NoC events
    input  logic noc_flit_injected,
    input  logic noc_flit_ejected,
    input  logic [3:0] noc_congestion_level,  // Higher values indicate more congestion
    
    // Memory controller events
    input  logic mem_read_start,
    input  logic mem_read_complete,
    input  logic mem_write_start,
    input  logic mem_write_complete,
    
    // Coherence directory events
    input  logic dir_lookup,
    input  logic dir_hit,
    input  logic dir_miss,
    input  logic dir_invalidation,
    
    // Performance counter outputs
    output logic [31:0] total_instructions,
    output logic [31:0] instructions_per_core[NUM_CORES-1:0],
    output logic [31:0] l1i_hit_rate,  // As percentage (0-100)
    output logic [31:0] l1d_hit_rate,  // As percentage (0-100)
    output logic [31:0] l2_hit_rate,   // As percentage (0-100)
    output logic [31:0] avg_memory_latency,
    output logic [31:0] noc_average_latency,
    output logic [31:0] coherence_traffic_percentage,  // % of NoC traffic for coherence
    
    // Control interface
    input  logic start_measurement,
    input  logic stop_measurement,
    input  logic reset_counters
);
    // Counter registers
    logic [31:0] instr_count[NUM_CORES-1:0];
    logic [31:0] instr_count_total;
    
    logic [31:0] l1i_hit_count;
    logic [31:0] l1i_miss_count;
    logic [31:0] l1d_hit_count;
    logic [31:0] l1d_miss_count;
    logic [31:0] l2_hit_count;
    logic [31:0] l2_miss_count;
    
    logic [31:0] mem_read_latency_sum;
    logic [31:0] mem_read_count;
    logic [31:0] mem_read_outstanding;
    logic [31:0] mem_read_cycles;
    
    logic [31:0] noc_flit_count;
    logic [31:0] noc_flit_latency_sum;
    logic [31:0] noc_flit_outstanding;
    
    logic [31:0] coherence_flit_count;
    
    // Measurement control
    logic measuring;
    
    // Counter logic
    always_ff @(posedge clk or negedge reset_n) begin
        if (~reset_n) begin
            // Reset all counters
            for (int i = 0; i < NUM_CORES; i++) begin
                instr_count[i] <= 32'h0;
            end
            instr_count_total <= 32'h0;
            
            l1i_hit_count <= 32'h0;
            l1i_miss_count <= 32'h0;
            l1d_hit_count <= 32'h0;
            l1d_miss_count <= 32'h0;
            l2_hit_count <= 32'h0;
            l2_miss_count <= 32'h0;
            
            mem_read_latency_sum <= 32'h0;
            mem_read_count <= 32'h0;
            mem_read_outstanding <= 32'h0;
            mem_read_cycles <= 32'h0;
            
            noc_flit_count <= 32'h0;
            noc_flit_latency_sum <= 32'h0;
            noc_flit_outstanding <= 32'h0;
            
            coherence_flit_count <= 32'h0;
            
            measuring <= 1'b0;
        end else begin
            // Control measurement window
            if (start_measurement) begin
                measuring <= 1'b1;
            end
            
            if (stop_measurement) begin
                measuring <= 1'b0;
            end
            
            if (reset_counters) begin
                // Reset all counters but keep measuring state
                for (int i = 0; i < NUM_CORES; i++) begin
                    instr_count[i] <= 32'h0;
                end
                instr_count_total <= 32'h0;
                
                l1i_hit_count <= 32'h0;
                l1i_miss_count <= 32'h0;
                l1d_hit_count <= 32'h0;
                l1d_miss_count <= 32'h0;
                l2_hit_count <= 32'h0;
                l2_miss_count <= 32'h0;
                
                mem_read_latency_sum <= 32'h0;
                mem_read_count <= 32'h0;
                mem_read_outstanding <= 32'h0;
                mem_read_cycles <= 32'h0;
                
                noc_flit_count <= 32'h0;
                noc_flit_latency_sum <= 32'h0;
                noc_flit_outstanding <= 32'h0;
                
                coherence_flit_count <= 32'h0;
            end
            
            // Only update counters if measuring
            if (measuring) begin
                // Core instruction counters
                for (int i = 0; i < NUM_CORES; i++) begin
                    if (core_instruction_completed[i]) begin
                        instr_count[i] <= instr_count[i] + 1;
                        instr_count_total <= instr_count_total + 1;
                    end
                end
                
                // Cache hit/miss counters
                for (int i = 0; i < NUM_CORES; i++) begin
                    if (core_cache_hit_l1i[i]) begin
                        l1i_hit_count <= l1i_hit_count + 1;
                    end
                    if (core_cache_miss_l1i[i]) begin
                        l1i_miss_count <= l1i_miss_count + 1;
                    end
                    if (core_cache_hit_l1d[i]) begin
                        l1d_hit_count <= l1d_hit_count + 1;
                    end
                    if (core_cache_miss_l1d[i]) begin
                        l1d_miss_count <= l1d_miss_count + 1;
                    end
                end
                
                // L2 cache counters
                if (l2_cache_hit) begin
                    l2_hit_count <= l2_hit_count + 1;
                end
                if (l2_cache_miss) begin
                    l2_miss_count <= l2_miss_count + 1;
                end
                
                // Memory latency tracking
                if (mem_read_start) begin
                    mem_read_outstanding <= mem_read_outstanding + 1;
                    mem_read_count <= mem_read_count + 1;
                end
                
                if (mem_read_complete) begin
                    mem_read_outstanding <= mem_read_outstanding - 1;
                end
                
                if (mem_read_outstanding > 0) begin
                    mem_read_cycles <= mem_read_cycles + 1;
                    mem_read_latency_sum <= mem_read_latency_sum + mem_read_outstanding;
                end
                
                // NoC flit tracking
                if (noc_flit_injected) begin
                    noc_flit_count <= noc_flit_count + 1;
                    noc_flit_outstanding <= noc_flit_outstanding + 1;
                end
                
                if (noc_flit_ejected) begin
                    noc_flit_outstanding <= noc_flit_outstanding - 1;
                end
                
                // Track NoC latency based on outstanding flits
                if (noc_flit_outstanding > 0) begin
                    noc_flit_latency_sum <= noc_flit_latency_sum + noc_flit_outstanding;
                end
                
                // Coherence traffic tracking
                if (dir_lookup) begin
                    coherence_flit_count <= coherence_flit_count + 1;
                end
            end
        end
    end
    
    // Calculate output metrics
    always_comb begin
        // Copy per-core instruction counts
        for (int i = 0; i < NUM_CORES; i++) begin
            instructions_per_core[i] = instr_count[i];
        end
        
        // Total instructions
        total_instructions = instr_count_total;
        
        // Cache hit rates (as percentages)
        if (l1i_hit_count + l1i_miss_count > 0) begin
            l1i_hit_rate = (l1i_hit_count * 100) / (l1i_hit_count + l1i_miss_count);
        end else begin
            l1i_hit_rate = 0;
        end
        
        if (l1d_hit_count + l1d_miss_count > 0) begin
            l1d_hit_rate = (l1d_hit_count * 100) / (l1d_hit_count + l1d_miss_count);
        end else begin
            l1d_hit_rate = 0;
        end
        
        if (l2_hit_count + l2_miss_count > 0) begin
            l2_hit_rate = (l2_hit_count * 100) / (l2_hit_count + l2_miss_count);
        end else begin
            l2_hit_rate = 0;
        end
        
        // Average memory latency
        if (mem_read_count > 0) begin
            avg_memory_latency = mem_read_latency_sum / mem_read_count;
        end else begin
            avg_memory_latency = 0;
        end
        
        // NoC average latency
        if (noc_flit_count > 0) begin
            noc_average_latency = noc_flit_latency_sum / noc_flit_count;
        end else begin
            noc_average_latency = 0;
        end
        
        // Coherence traffic percentage
        if (noc_flit_count > 0) begin
            coherence_traffic_percentage = (coherence_flit_count * 100) / noc_flit_count;
        end else begin
            coherence_traffic_percentage = 0;
        end
    end
endmodule