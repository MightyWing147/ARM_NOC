module clock_reset_manager(
    input  logic ext_clk,
    input  logic ext_reset_n,
    
    output logic core_clk,
    output logic core_reset_n,
    
    output logic mem_clk,
    output logic mem_reset_n,
    
    output logic noc_clk,
    output logic noc_reset_n
);
    // PLL parameters for clock generation
    // In a real implementation, these would control clock dividers/multipliers
    parameter CORE_CLK_DIV = 1;    // Core clock division factor
    parameter MEM_CLK_DIV = 2;     // Memory clock division factor
    parameter NOC_CLK_DIV = 1;     // NoC clock division factor
    
    // Clock generation - in real hardware this would be a PLL
    // For simulation, we'll use simple clock dividers
    
    // Core clock (typically the highest frequency)
    logic [3:0] core_clk_counter;
    
    // Memory clock (typically half of core clock)
    logic [3:0] mem_clk_counter;
    
    // NoC clock (can be same as core or different)
    logic [3:0] noc_clk_counter;
    
    // Reset synchronizers (3-stage for metastability protection)
    logic [2:0] core_reset_sync;
    logic [2:0] mem_reset_sync;
    logic [2:0] noc_reset_sync;
    
    // Clock generation logic
    always_ff @(posedge ext_clk or negedge ext_reset_n) begin
        if (~ext_reset_n) begin
            core_clk_counter <= 4'h0;
            mem_clk_counter <= 4'h0;
            noc_clk_counter <= 4'h0;
            core_clk <= 1'b0;
            mem_clk <= 1'b0;
            noc_clk <= 1'b0;
        end else begin
            // Core clock generation
            if (CORE_CLK_DIV == 1) begin
                core_clk <= ext_clk;  // Pass through
            end else begin
                core_clk_counter <= core_clk_counter + 1;
                if (core_clk_counter >= (CORE_CLK_DIV - 1)) begin
                    core_clk_counter <= 4'h0;
                    core_clk <= ~core_clk;
                end
            end
            
            // Memory clock generation
            if (MEM_CLK_DIV == 1) begin
                mem_clk <= ext_clk;  // Pass through
            end else begin
                mem_clk_counter <= mem_clk_counter + 1;
                if (mem_clk_counter >= (MEM_CLK_DIV - 1)) begin
                    mem_clk_counter <= 4'h0;
                    mem_clk <= ~mem_clk;
                end
            end
            
            // NoC clock generation
            if (NOC_CLK_DIV == 1) begin
                noc_clk <= ext_clk;  // Pass through
            end else begin
                noc_clk_counter <= noc_clk_counter + 1;
                if (noc_clk_counter >= (NOC_CLK_DIV - 1)) begin
                    noc_clk_counter <= 4'h0;
                    noc_clk <= ~noc_clk;
                end
            end
        end
    end
    
    // Reset synchronizers for each clock domain
    always_ff @(posedge core_clk or negedge ext_reset_n) begin
        if (~ext_reset_n) begin
            core_reset_sync <= 3'b000;
        end else begin
            core_reset_sync <= {core_reset_sync[1:0], 1'b1};
        end
    end
    
    always_ff @(posedge mem_clk or negedge ext_reset_n) begin
        if (~ext_reset_n) begin
            mem_reset_sync <= 3'b000;
        end else begin
            mem_reset_sync <= {mem_reset_sync[1:0], 1'b1};
        end
    end
    
    always_ff @(posedge noc_clk or negedge ext_reset_n) begin
        if (~ext_reset_n) begin
            noc_reset_sync <= 3'b000;
        end else begin
            noc_reset_sync <= {noc_reset_sync[1:0], 1'b1};
        end
    end
    
    // Output the synchronized reset signals
    assign core_reset_n = core_reset_sync[2];
    assign mem_reset_n = mem_reset_sync[2];
    assign noc_reset_n = noc_reset_sync[2];
endmodule