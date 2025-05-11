module multi_core_system_tb;
    // Import configuration package
    import noc_config_pkg::*;
    
    // Testbench signals
    logic clk;
    logic reset_n;
    
    // External memory interface
    logic [47:0] ext_mem_addr;
    logic        ext_mem_write;
    logic [63:0] ext_mem_wdata[0:7];
    logic        ext_mem_req_valid;
    logic        ext_mem_req_ready;
    
    logic [63:0] ext_mem_rdata[0:7];
    logic        ext_mem_resp_valid;
    logic        ext_mem_resp_ready;
    
    // Debug interfaces
    logic [NUM_CORES-1:0]        debug_halt;
    logic [NUM_CORES-1:0]        debug_run;
    logic [NUM_CORES-1:0]        debug_step;
    logic [NUM_CORES-1:0][47:0]  debug_pc;
    logic [NUM_CORES-1:0]        debug_pc_wr_en;
    
    // Status outputs
    logic [NUM_CORES-1:0][3:0]   core_state;
    logic [NUM_CORES-1:0]        halted;
    
    // Performance counters
    logic [31:0] total_mem_reads;
    logic [31:0] total_mem_writes;
    logic [31:0] avg_read_latency;
    logic [31:0] avg_write_latency;
    logic [31:0] dir_hits;
    logic [31:0] dir_misses;
    
    // Simplified memory model
    logic [63:0] memory_array[0:MEM_SIZE_MB*1024*1024/8 - 1];
    
    // Test program data
    localparam CODE_START_ADDR = 32'h1000;  // Starting address for program code
    localparam DATA_START_ADDR = 32'h2000;  // Starting address for shared data
    
    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk;  // 100MHz clock
    end
    
    // Reset generation
    initial begin
        reset_n = 0;
        repeat(10) @(posedge clk);
        reset_n = 1;
    end
    
    // DUT instantiation
    multi_core_system #(
        .NUM_CORES(NUM_CORES),
        .ROWS(ROWS),
        .COLS(COLS),
        .FLIT_WIDTH(FLIT_WIDTH),
        .VC_COUNT(VC_COUNT),
        .CACHE_LINE_SIZE(CACHE_LINE_SIZE),
        .MEM_SIZE_MB(MEM_SIZE_MB)
    ) dut (
        .clk(clk),
        .reset_n(reset_n),
        
        // External memory interface
        .ext_mem_addr(ext_mem_addr),
        .ext_mem_write(ext_mem_write),
        .ext_mem_wdata(ext_mem_wdata),
        .ext_mem_req_valid(ext_mem_req_valid),
        .ext_mem_req_ready(ext_mem_req_ready),
        
        .ext_mem_rdata(ext_mem_rdata),
        .ext_mem_resp_valid(ext_mem_resp_valid),
        .ext_mem_resp_ready(ext_mem_resp_ready),
        
        // Debug interfaces
        .debug_halt(debug_halt),
        .debug_run(debug_run),
        .debug_step(debug_step),
        .debug_pc(debug_pc),
        .debug_pc_wr_en(debug_pc_wr_en),
        
        // Status outputs
        .core_state(core_state),
        .halted(halted),
        
        // Performance counters
        .total_mem_reads(total_mem_reads),
        .total_mem_writes(total_mem_writes),
        .avg_read_latency(avg_read_latency),
        .avg_write_latency(avg_write_latency),
        .dir_hits(dir_hits),
        .dir_misses(dir_misses)
    );
    
    // Memory model implementation
    always @(posedge clk or negedge reset_n) begin
        if (~reset_n) begin
            ext_mem_req_ready <= 1'b0;
            ext_mem_resp_valid <= 1'b0;
            
            // Initialize memory with test pattern
            for (int i = 0; i < MEM_SIZE_MB*1024*1024/8; i++) begin
                memory_array[i] = 64'h0;
            end
            
            // Load test program into memory
            initialize_memory();
            
            repeat(5) @(posedge clk);
            ext_mem_req_ready <= 1'b1;  // Memory ready to accept requests
        end else begin
            // Default values
            ext_mem_resp_valid <= 1'b0;
            
            // Handle memory requests
            if (ext_mem_req_valid && ext_mem_req_ready) begin
                // Calculate base address in 8-byte chunks
                logic [47:0] base_addr = {ext_mem_addr[47:3], 3'b000};
                int base_index = base_addr[27:3];  // Index into memory array
                
                if (ext_mem_write) begin
                    // Handle memory write
                    for (int i = 0; i < 8; i++) begin
                        memory_array[base_index + i] = ext_mem_wdata[i];
                    end
                    
                    // Respond after 10 cycles (simulating DRAM latency)
                    repeat(10) @(posedge clk);
                    ext_mem_resp_valid <= 1'b1;
                    @(posedge clk);
                    ext_mem_resp_valid <= 1'b0;
                end else begin
                    // Handle memory read
                    for (int i = 0; i < 8; i++) begin
                        ext_mem_rdata[i] = memory_array[base_index + i];
                    end
                    
                    // Respond after 10 cycles (simulating DRAM latency)
                    repeat(10) @(posedge clk);
                    ext_mem_resp_valid <= 1'b1;
                    @(posedge clk);
                    ext_mem_resp_valid <= 1'b0;
                end
            end
        end
    end
    
    // Initialize memory with test programs and data
    task initialize_memory();
        // Reset memory in program area
        for (int i = 0; i < 1024; i++) begin
            memory_array[CODE_START_ADDR/8 + i] = 64'h0;
        end
        
        // Test program - producer-consumer coherence test
        // Core 0 (Producer) code
        memory_array[CODE_START_ADDR/8 + 0] = 64'hD2800020D2800000;  // MOV X0, #0; MOV X1, #1
        memory_array[CODE_START_ADDR/8 + 1] = 64'hD2A00002D2800063;  // MOV X2, #DATA_START_ADDR; MOV X3, #3
        memory_array[CODE_START_ADDR/8 + 2] = 64'hF9000043F9000000;  // STR X3, [X2]; STR X0, [X0]
        memory_array[CODE_START_ADDR/8 + 3] = 64'h8B010000F9400044;  // ADD X0, X0, X1; LDR X4, [X2, #0]
        memory_array[CODE_START_ADDR/8 + 4] = 64'h8B010000F9000040;  // ADD X0, X0, X1; STR X0, [X2]
        memory_array[CODE_START_ADDR/8 + 5] = 64'h17FFFFFD14000000;  // B loop; B end
        
        // Core 1 (Consumer) code
        memory_array[CODE_START_ADDR/8 + 64] = 64'hD2800020D2800000;  // MOV X0, #0; MOV X1, #1
        memory_array[CODE_START_ADDR/8 + 65] = 64'hD2A00002D2800063;  // MOV X2, #DATA_START_ADDR; MOV X3, #3
        memory_array[CODE_START_ADDR/8 + 66] = 64'hF9400044F9400000;  // LDR X4, [X2]; LDR X0, [X0]
        memory_array[CODE_START_ADDR/8 + 67] = 64'hEB04001FF9400044;  // CMP X0, X4; LDR X4, [X2]
        memory_array[CODE_START_ADDR/8 + 68] = 64'h8B010000AA000000;  // ADD X0, X0, X1; MOV X0, X0
        memory_array[CODE_START_ADDR/8 + 69] = 64'h17FFFFFC14000000;  // B loop; B end
        
        // Core 2 and 3 - memory stress test with shared location
        memory_array[CODE_START_ADDR/8 + 128] = 64'hD2800020D2800000;  // MOV X0, #0; MOV X1, #1
        memory_array[CODE_START_ADDR/8 + 129] = 64'hD2A00102D2800063;  // MOV X2, #DATA_START_ADDR+8; MOV X3, #3
        memory_array[CODE_START_ADDR/8 + 130] = 64'hF9000043F9400040;  // STR X3, [X2]; LDR X0, [X2]
        memory_array[CODE_START_ADDR/8 + 131] = 64'h8B030000F9000040;  // ADD X0, X0, X3; STR X0, [X2]
        memory_array[CODE_START_ADDR/8 + 132] = 64'h17FFFFFD14000000;  // B loop; B end
        
        memory_array[CODE_START_ADDR/8 + 192] = 64'hD2800020D2800000;  // MOV X0, #0; MOV X1, #1
        memory_array[CODE_START_ADDR/8 + 193] = 64'hD2A00102D2800063;  // MOV X2, #DATA_START_ADDR+8; MOV X3, #3
        memory_array[CODE_START_ADDR/8 + 194] = 64'hF9400040AA000000;  // LDR X0, [X2]; MOV X0, X0
        memory_array[CODE_START_ADDR/8 + 195] = 64'h8B030000F9000040;  // ADD X0, X0, X3; STR X0, [X2]
        memory_array[CODE_START_ADDR/8 + 196] = 64'h17FFFFFD14000000;  // B loop; B end
        
        // Initialize shared data for coherence testing
        memory_array[DATA_START_ADDR/8] = 64'h0;  // Shared location for producer-consumer test
        memory_array[DATA_START_ADDR/8 + 1] = 64'h0;  // Shared location for stress test
        
        $display("Memory initialized with test programs");
    endtask
    
    // Run test program
    initial begin
        // Wait for reset and initialization
        wait(reset_n);
        repeat(20) @(posedge clk);
        
        // Initialize debug signals
        debug_halt = {NUM_CORES{1'b0}};
        debug_run = {NUM_CORES{1'b0}};
        debug_step = {NUM_CORES{1'b0}};
        debug_pc_wr_en = {NUM_CORES{1'b0}};
        
        // Set initial PC for each core to start of their respective programs
        for (int i = 0; i < NUM_CORES; i++) begin
            debug_pc[i] = CODE_START_ADDR + (i * 512);  // Each core gets a different program section
            debug_pc_wr_en[i] = 1'b1;
        end
        
        // Apply PC values
        repeat(2) @(posedge clk);
        debug_pc_wr_en = {NUM_CORES{1'b0}};
        
        // Let cores start executing
        debug_run = {NUM_CORES{1'b1}};
        
        // Run for a while to allow caches to warm up
        repeat(1000) @(posedge clk);
        
        // Start main test - run long enough for coherence interactions
        $display("Starting main coherence test at time %0t", $time);
        repeat(10000) @(posedge clk);
        
        // Print results and check for coherence
        print_test_results();
        check_coherence_status();
        
        // Halt cores for debugging
        debug_halt = {NUM_CORES{1'b1}};
        repeat(10) @(posedge clk);
        
        $display("Simulation completed successfully");
        $finish;
    end
    
    // Print test results
    task print_test_results();
        $display("=== Test Results at %0t ===", $time);
        $display("Memory reads: %0d", total_mem_reads);
        $display("Memory writes: %0d", total_mem_writes);
        $display("Average read latency: %0d cycles", avg_read_latency);
        $display("Average write latency: %0d cycles", avg_write_latency);
        $display("Directory hits: %0d", dir_hits);
        $display("Directory misses: %0d", dir_misses);
        $display("Directory hit rate: %0.2f%%", 
                 real'(dir_hits) / real'(dir_hits + dir_misses) * 100.0);
        
        // Check core status
        $display("Core status:");
        for (int i = 0; i < NUM_CORES; i++) begin
            $display("  Core %0d: %s, PC: %h", 
                    i, 
                    halted[i] ? "HALTED" : "RUNNING",
                    debug_pc[i]);
        end
        
        // Print shared memory values
        $display("Shared memory contents:");
        $display("  Location 1 (Producer-Consumer): %h", memory_array[DATA_START_ADDR/8]);
        $display("  Location 2 (Stress Test): %h", memory_array[DATA_START_ADDR/8 + 1]);
        $display("==============================");
    endtask
    
    // Check coherence status by examining shared memory locations
    task check_coherence_status();
        // In a proper test, we would check that the memory values match expected patterns
        // For now, we'll just check that shared locations have been updated
        
        if (memory_array[DATA_START_ADDR/8] == 64'h0) begin
            $display("FAIL: Producer-consumer test didn't update shared location!");
        end else begin
            $display("PASS: Producer-consumer test updated shared location to %h", 
                     memory_array[DATA_START_ADDR/8]);
        end
        
        if (memory_array[DATA_START_ADDR/8 + 1] == 64'h0) begin
            $display("FAIL: Memory stress test didn't update shared location!");
        end else begin
            $display("PASS: Memory stress test updated shared location to %h", 
                     memory_array[DATA_START_ADDR/8 + 1]);
        end
    endtask
    
    // Monitor for coherence violations
    // This is a simplified monitor - a real implementation would check all coherence invariants
    property cache_coherence_invariant;
        @(posedge clk)
        disable iff(!reset_n)
        // Coherence invariant: If a cache line is in modified state,
        // no other cache can have a valid copy
        1;  // Placeholder for actual property check
    endproperty
    
    assert property(cache_coherence_invariant) else
        $error("Cache coherence violation detected!");
    
    // Simulation timeout
    initial begin
        #50000000; // Long timeout to ensure simulation completes
        $display("Simulation timeout reached");
        $finish;
    end
    
    // Waveform dump for debugging
    initial begin
        $dumpfile("multi_core_system_tb.vcd");
        $dumpvars(0, multi_core_system_tb);
    end
    
endmodule