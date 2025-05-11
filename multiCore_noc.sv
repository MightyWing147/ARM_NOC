module multicore_system #(
    parameter NUM_CORES = 4,
    parameter ROWS = 2,
    parameter COLS = 2, // ROWS × COLS must equal NUM_CORES
    parameter FLIT_WIDTH = 128,
    parameter VC_COUNT = 3
)(
    input  logic clk,
    input  logic reset_n,
    
    // Memory interface
    output logic mem_req_valid,
    output logic mem_req_write,
    output logic [47:0] mem_req_addr,
    output logic [63:0] mem_req_data,
    input  logic mem_req_ready,
    
    input  logic mem_resp_valid,
    input  logic [63:0] mem_resp_data,
    input  logic mem_resp_last,
    output logic mem_resp_ready
);
    // NoC interface signals
    logic [NUM_CORES-1:0][FLIT_WIDTH-1:0] core_to_ni_flit;
    logic [NUM_CORES-1:0][VC_COUNT-1:0] core_to_ni_valid;
    logic [NUM_CORES-1:0][VC_COUNT-1:0] ni_to_core_ready;
    
    logic [NUM_CORES-1:0][FLIT_WIDTH-1:0] ni_to_core_flit;
    logic [NUM_CORES-1:0][VC_COUNT-1:0] ni_to_core_valid;
    logic [NUM_CORES-1:0][VC_COUNT-1:0] core_to_ni_ready;
    
    // Cache controller interface signals
    logic [NUM_CORES-1:0][47:0] cache_req_addr;
    logic [NUM_CORES-1:0][63:0] cache_req_data;
    logic [NUM_CORES-1:0][7:0] cache_req_type;
    logic [NUM_CORES-1:0] cache_req_valid;
    logic [NUM_CORES-1:0] cache_req_ready;
    
    logic [NUM_CORES-1:0][63:0] cache_resp_data;
    logic [NUM_CORES-1:0][7:0] cache_resp_type;
    logic [NUM_CORES-1:0] cache_resp_valid;
    logic [NUM_CORES-1:0] cache_resp_ready;
    
    // Coherence interface signals
    logic [NUM_CORES-1:0][47:0] coh_req_addr;
    logic [NUM_CORES-1:0][2:0] coh_req_type;
    logic [NUM_CORES-1:0][7:0] coh_req_target;
    logic [NUM_CORES-1:0] coh_req_valid;
    logic [NUM_CORES-1:0] coh_req_ready;
    
    logic [NUM_CORES-1:0][47:0] coh_resp_addr;
    logic [NUM_CORES-1:0][2:0] coh_resp_type;
    logic [NUM_CORES-1:0][7:0] coh_resp_source;
    logic [NUM_CORES-1:0][63:0] coh_resp_data;
    logic [NUM_CORES-1:0] coh_resp_valid;
    logic [NUM_CORES-1:0] coh_resp_ready;
    
    // Core CPU interface signals
    logic [NUM_CORES-1:0][47:0] cpu_addr;
    logic [NUM_CORES-1:0] cpu_read;
    logic [NUM_CORES-1:0] cpu_write;
    logic [NUM_CORES-1:0][63:0] cpu_wdata;
    logic [NUM_CORES-1:0][7:0] cpu_byte_en;
    logic [NUM_CORES-1:0][63:0] cpu_rdata;
    logic [NUM_CORES-1:0] cpu_ready;
    
    // NoC mesh instantiation
    noc_mesh #(
        .ROWS(ROWS),
        .COLS(COLS),
        .FLIT_WIDTH(FLIT_WIDTH),
        .VC_COUNT(VC_COUNT)
    ) noc_inst (
        .clk(clk),
        .reset_n(reset_n),
        .local_flit_in(core_to_ni_flit),
        .local_valid_in(core_to_ni_valid),
        .local_ready_out(ni_to_core_ready),
        .local_flit_out(ni_to_core_flit),
        .local_valid_out(ni_to_core_valid),
        .local_ready_in(core_to_ni_ready)
    );
    
    // Instantiate cores, caches, and network interfaces
    genvar i;
    generate
        for (i = 0; i < NUM_CORES; i++) begin : core_gen
            // ARM core instantiation
            arm_core #(
                .CORE_ID(i)
            ) core_inst (
                .clk(clk),
                .reset_n(reset_n),
                
                // Memory interface
                .mem_addr(cpu_addr[i]),
                .mem_read(cpu_read[i]),
                .mem_write(cpu_write[i]),
                .mem_wdata(cpu_wdata[i]),
                .mem_byte_en(cpu_byte_en[i]),
                .mem_rdata(cpu_rdata[i]),
                .mem_ready(cpu_ready[i]),
                
                // Other core signals
                // ...
            );
            
            // L1 D-Cache with coherence
            l1dcache_coherent dcache_inst (
                .clk(clk),
                .reset_n(reset_n),
                
                // CPU interface
                .cpu_addr(cpu_addr[i]),
                .cpu_read(cpu_read[i]),
                .cpu_write(cpu_write[i]),
                .cpu_wdata(cpu_wdata[i]),
                .cpu_byte_en(cpu_byte_en[i]),
                .cpu_rdata(cpu_rdata[i]),
                .cpu_ready(cpu_ready[i]),
                
                // Network Interface connection
                .ni_req_addr(cache_req_addr[i]),
                .ni_req_data(cache_req_data[i]),
                .ni_req_type(cache_req_type[i]),
                .ni_req_valid(cache_req_valid[i]),
                .ni_req_ready(cache_req_ready[i]),
                
                .ni_resp_data(cache_resp_data[i]),
                .ni_resp_type(cache_resp_type[i]),
                .ni_resp_valid(cache_resp_valid[i]),
                .ni_resp_ready(cache_resp_ready[i]),
                
                // Coherence interface
                .coh_req_addr(coh_req_addr[i]),
                .coh_req_type(coh_req_type[i]),
                .coh_req_target(coh_req_target[i]),
                .coh_req_valid(coh_req_valid[i]),
                .coh_req_ready(coh_req_ready[i]),
                
                .coh_resp_addr(coh_resp_addr[i]),
                .coh_resp_type(coh_resp_type[i]),
                .coh_resp_source(coh_resp_source[i]),
                .coh_resp_data(coh_resp_data[i]),
                .coh_resp_valid(coh_resp_valid[i]),
                .coh_resp_ready(coh_resp_ready[i])
            );
            
            // Network Interface
            network_interface #(
                .CORE_ID(i),
                .ADDR_WIDTH(48),
                .DATA_WIDTH(64),
                .FLIT_WIDTH(FLIT_WIDTH),
                .VC_COUNT(VC_COUNT)
            ) ni_inst (
                .clk(clk),
                .reset_n(reset_n),
                
                // Cache Controller Interface
                .cache_req_addr(cache_req_addr[i]),
                .cache_req_data(cache_req_data[i]),
                .cache_req_type(cache_req_type[i]),
                .cache_req_valid(cache_req_valid[i]),
                .cache_req_ready(cache_req_ready[i]),
                
                .cache_resp_data(cache_resp_data[i]),
                .cache_resp_type(cache_resp_type[i]),
                .cache_resp_valid(cache_resp_valid[i]),
                .cache_resp_ready(cache_resp_ready[i]),
                
                // Coherence Interface
                .coh_req_addr(coh_req_addr[i]),
                .coh_req_type(coh_req_type[i]),
                .coh_req_target(coh_req_target[i]),
                .coh_req_valid(coh_req_valid[i]),
                .coh_req_ready(coh_req_ready[i]),
                
                .coh_resp_addr(coh_resp_addr[i]),
                .coh_resp_type(coh_resp_type[i]),
                .coh_resp_source(coh_resp_source[i]),
                .coh_resp_data(coh_resp_data[i]),
                .coh_resp_valid(coh_resp_valid[i]),
                .coh_resp_ready(coh_resp_ready[i]),
                
                // NoC Router Interface
                .router_flit_out(core_to_ni_flit[i]),
                .router_valid_out(core_to_ni_valid[i]),
                .router_ready_in(ni_to_core_ready[i]),
                
                .router_flit_in(ni_to_core_flit[i]),
                .router_valid_in(ni_to_core_valid[i]),
                .router_ready_out(core_to_ni_ready[i])
            );
        end
    endgenerate
    
    // Memory controller interface
    // ...
endmodule