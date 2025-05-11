module memory_controller #(
    parameter ADDR_WIDTH = 48,
    parameter DATA_WIDTH = 64,
    parameter CACHE_LINE_SIZE = 64,
    parameter MEM_SIZE_MB = 1024    // 1GB memory
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
    input  logic [63:0] noc_resp_data[0:7],  // Full cache line data
    input  logic        noc_resp_valid,
    output logic        noc_resp_ready,
    
    // External Memory Interface
    output logic [47:0] mem_addr,
    output logic        mem_write,
    output logic [63:0] mem_wdata[0:7],
    output logic        mem_req_valid,
    input  logic        mem_req_ready,
    
    input  logic [63:0] mem_rdata[0:7],
    input  logic        mem_resp_valid,
    output logic        mem_resp_ready,
    
    // Statistics
    output logic [31:0] read_count,
    output logic [31:0] write_count,
    output logic [31:0] read_latency_sum,
    output logic [31:0] write_latency_sum
);
    // Constants for coherence protocol operations
    localparam REQ_READ          = 3'b000;  // Read request
    localparam REQ_READ_EXCL     = 3'b001;  // Read exclusive request
    localparam REQ_WRITE         = 3'b010;  // Write request
    localparam REQ_INVALIDATE    = 3'b011;  // Invalidate request
    localparam RESP_DATA         = 3'b100;  // Data response
    localparam RESP_DATA_EXCL    = 3'b101;  // Data response with exclusivity
    localparam RESP_ACK          = 3'b110;  // Acknowledgment
    
    // Memory controller states
    typedef enum logic [2:0] {
        IDLE,
        PROCESS_READ,
        PROCESS_WRITE,
        WAIT_MEMORY,
        SEND_RESPONSE
    } mem_state_t;
    
    mem_state_t state, next_state;
    
    // Request buffer
    logic [47:0] req_addr;
    logic [2:0]  req_type;
    logic [7:0]  req_source;
    logic [63:0] req_data[0:7];  // Full cache line
    
    // Response data buffer
    logic [63:0] resp_data[0:7];
    
    // Performance tracking
    logic [31:0] current_latency;
    
    // For address alignment to cache line boundaries
    localparam OFFSET_BITS = $clog2(CACHE_LINE_SIZE);
    
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
                if (noc_resp_valid) begin
                    if (noc_resp_type == REQ_READ || noc_resp_type == REQ_READ_EXCL) begin
                        next_state = PROCESS_READ;
                    end else if (noc_resp_type == REQ_WRITE) begin
                        next_state = PROCESS_WRITE;
                    end
                end
            end
            
            PROCESS_READ, PROCESS_WRITE: begin
                next_state = WAIT_MEMORY;
            end
            
            WAIT_MEMORY: begin
                if (mem_resp_valid) begin
                    next_state = SEND_RESPONSE;
                end
            end
            
            SEND_RESPONSE: begin
                if (noc_req_ready) begin
                    next_state = IDLE;
                end
            end
        endcase
    end
    
    // Main controller logic
    always_ff @(posedge clk or negedge reset_n) begin
        if (~reset_n) begin
            // Reset signals
            noc_req_valid <= 1'b0;
            noc_resp_ready <= 1'b0;
            mem_req_valid <= 1'b0;
            mem_resp_ready <= 1'b0;
            mem_write <= 1'b0;
            
            read_count <= 32'h0;
            write_count <= 32'h0;
            read_latency_sum <= 32'h0;
            write_latency_sum <= 32'h0;
            current_latency <= 32'h0;
            
            // Clear buffers
            for (int i = 0; i < 8; i++) begin
                req_data[i] <= 64'h0;
                resp_data[i] <= 64'h0;
                mem_wdata[i] <= 64'h0;
            end
        end else begin
            // Default values
            noc_req_valid <= 1'b0;
            noc_resp_ready <= 1'b1;  // Always ready to receive new requests
            mem_req_valid <= 1'b0;
            mem_resp_ready <= 1'b1;  // Always ready to receive memory responses
            
            // Increment latency counter when waiting for memory
            if (state == WAIT_MEMORY) begin
                current_latency <= current_latency + 1;
            end
            
            // Process state machine
            case (state)
                IDLE: begin
                    // Reset latency counter
                    current_latency <= 32'h0;
                    
                    // Capture new request
                    if (noc_resp_valid && noc_resp_ready) begin
                        req_addr <= noc_resp_addr;
                        req_type <= noc_resp_type;
                        req_source <= noc_resp_source;
                        
                        // For write requests, capture data
                        if (noc_resp_type == REQ_WRITE) begin
                            for (int i = 0; i < 8; i++) begin
                                req_data[i] <= noc_resp_data[i];
                            end
                        end
                    end
                end
                
                PROCESS_READ: begin
                    // Initiate memory read
                    mem_req_valid <= 1'b1;
                    mem_write <= 1'b0;
                    // Align address to cache line boundary
                    mem_addr <= {req_addr[47:OFFSET_BITS], {OFFSET_BITS{1'b0}}};
                    
                    // Update statistics
                    read_count <= read_count + 1;
                end
                
                PROCESS_WRITE: begin
                    // Initiate memory write
                    mem_req_valid <= 1'b1;
                    mem_write <= 1'b1;
                    // Align address to cache line boundary
                    mem_addr <= {req_addr[47:OFFSET_BITS], {OFFSET_BITS{1'b0}}};
                    
                    // Pass write data to memory
                    for (int i = 0; i < 8; i++) begin
                        mem_wdata[i] <= req_data[i];
                    end
                    
                    // Update statistics
                    write_count <= write_count + 1;
                end
                
                WAIT_MEMORY: begin
                    // Wait for memory response
                    if (mem_resp_valid) begin
                        // For reads, capture returned data
                        if (!mem_write) begin
                            for (int i = 0; i < 8; i++) begin
                                resp_data[i] <= mem_rdata[i];
                            end
                            
                            // Update read latency statistics
                            read_latency_sum <= read_latency_sum + current_latency;
                        end else begin
                            // Update write latency statistics
                            write_latency_sum <= write_latency_sum + current_latency;
                        end
                    end
                end
                
                SEND_RESPONSE: begin
                    // Send response back via NoC
                    noc_req_valid <= 1'b1;
                    noc_req_addr <= req_addr;
                    noc_req_target <= req_source;
                    
                    // Response type depends on request type
                    if (req_type == REQ_READ) begin
                        noc_req_type <= RESP_DATA;
                        // Send first word of data for initial response
                        // Full data will be sent in subsequent transfers
                        noc_req_data <= resp_data[0];
                    end else if (req_type == REQ_READ_EXCL) begin
                        noc_req_type <= RESP_DATA_EXCL;
                        noc_req_data <= resp_data[0];
                    end else begin
                        // For writes and other operations
                        noc_req_type <= RESP_ACK;
                        noc_req_data <= 64'h0;
                    end
                end
            endcase
        end
    end
endmodule