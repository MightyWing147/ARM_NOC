module network_interface #(
    parameter CORE_ID = 0,
    parameter ADDR_WIDTH = 48,
    parameter DATA_WIDTH = 64,
    parameter FLIT_WIDTH = 128,
    parameter VC_COUNT = 3    // Virtual channels: request, response, coherence
)(
    input  logic clk,
    input  logic reset_n,
    
    // Cache Controller Interface
    input  logic [ADDR_WIDTH-1:0] cache_req_addr,
    input  logic [DATA_WIDTH-1:0] cache_req_data,
    input  logic [7:0] cache_req_type,  // Request type encoding
    input  logic cache_req_valid,
    output logic cache_req_ready,
    
    output logic [DATA_WIDTH-1:0] cache_resp_data,
    output logic [7:0] cache_resp_type,
    output logic cache_resp_valid,
    input  logic cache_resp_ready,
    
    // Coherence Interface
    input  logic [ADDR_WIDTH-1:0] coh_req_addr,
    input  logic [2:0] coh_req_type,   // Coherence message type
    input  logic [7:0] coh_req_target, // Target node(s)
    input  logic coh_req_valid,
    output logic coh_req_ready,
    
    output logic [ADDR_WIDTH-1:0] coh_resp_addr,
    output logic [2:0] coh_resp_type,
    output logic [7:0] coh_resp_source,
    output logic [DATA_WIDTH-1:0] coh_resp_data,
    output logic coh_resp_valid,
    input  logic coh_resp_ready,
    
    // NoC Router Interface
    output logic [VC_COUNT-1:0][FLIT_WIDTH-1:0] router_flit_out,
    output logic [VC_COUNT-1:0] router_valid_out,
    input  logic [VC_COUNT-1:0] router_ready_in,
    
    input  logic [VC_COUNT-1:0][FLIT_WIDTH-1:0] router_flit_in,
    input  logic [VC_COUNT-1:0] router_valid_in,
    output logic [VC_COUNT-1:0] router_ready_out
);
    // Packet format
    typedef struct packed {
        logic [7:0] dest_id;
        logic [7:0] source_id;
        logic [7:0] msg_type;
        logic [ADDR_WIDTH-1:0] addr;
        logic [DATA_WIDTH-1:0] data;
    } noc_packet_t;
    
    // Message type encodings
    localparam REQ_READ         = 8'h01;
    localparam REQ_WRITE        = 8'h02;
    localparam REQ_FETCH        = 8'h03;
    localparam RESP_READ_DATA   = 8'h81;
    localparam RESP_WRITE_ACK   = 8'h82;
    localparam RESP_FETCH_DATA  = 8'h83;
    
    // Coherence message types
    localparam COH_INVALIDATE   = 8'h10;
    localparam COH_SHARED_REQ   = 8'h11;
    localparam COH_EXCL_REQ     = 8'h12;
    localparam COH_WB_REQ       = 8'h13;
    localparam COH_INV_ACK      = 8'h90;
    localparam COH_DATA_RESP    = 8'h91;
    
    // Virtual channel assignments
    localparam VC_REQUEST     = 0;
    localparam VC_RESPONSE    = 1;
    localparam VC_COHERENCE   = 2;
    
    // Internal state
    noc_packet_t tx_packet;
    noc_packet_t rx_packet;
    
    logic [2:0] req_state;
    logic [2:0] resp_state;
    logic [2:0] coh_state;
    
    // States
    localparam IDLE     = 3'b000;
    localparam SENDING  = 3'b001;
    localparam WAITING  = 3'b010;
    
    // Directory node ID calculation based on address
    // Example: Simple modulo mapping
    function automatic logic [7:0] get_home_id(input logic [ADDR_WIDTH-1:0] addr);
        // Extract relevant bits from address (e.g., middle bits to avoid clustering)
        logic [7:0] address_hash;
        address_hash = addr[15:8] ^ addr[23:16];
        return address_hash % 8; // Assuming 8 nodes
    endfunction
    
    // Transmit packet assembly logic
    always_comb begin
        tx_packet = '0;
        tx_packet.source_id = CORE_ID;
        
        // Regular memory requests
        if (cache_req_valid) begin
            case (cache_req_type)
                REQ_READ, REQ_WRITE, REQ_FETCH: begin
                    tx_packet.addr = cache_req_addr;
                    tx_packet.data = cache_req_data;
                    tx_packet.msg_type = cache_req_type;
                    
                    // For memory requests, destination is based on address
                    // Higher bits of address determine memory controller location
                    tx_packet.dest_id = cache_req_addr[ADDR_WIDTH-1:ADDR_WIDTH-8];
                end
                default: tx_packet = '0;
            endcase
        end
        // Coherence requests
        else if (coh_req_valid) begin
            tx_packet.addr = coh_req_addr;
            
            // Map internal coherence type to NoC message type
            case (coh_req_type)
                3'b001: tx_packet.msg_type = COH_INVALIDATE;
                3'b010: tx_packet.msg_type = COH_SHARED_REQ;
                3'b011: tx_packet.msg_type = COH_EXCL_REQ;
                3'b100: tx_packet.msg_type = COH_WB_REQ;
                default: tx_packet.msg_type = 8'h00;
            endcase
            
            // For directory operations, destination is home node for this address
            if (coh_req_target == 8'hFF) // Special value for directory
                tx_packet.dest_id = get_home_id(coh_req_addr);
            else
                tx_packet.dest_id = coh_req_target;
        end
    end
    
    // Request channel state machine
    always_ff @(posedge clk or negedge reset_n) begin
        if (~reset_n) begin
            req_state <= IDLE;
            router_valid_out[VC_REQUEST] <= 1'b0;
            cache_req_ready <= 1'b1;
        end
        else begin
            case (req_state)
                IDLE: begin
                    if (cache_req_valid && (cache_req_type == REQ_READ || 
                                           cache_req_type == REQ_WRITE || 
                                           cache_req_type == REQ_FETCH)) begin
                        // Memory request
                        router_flit_out[VC_REQUEST] <= tx_packet;
                        router_valid_out[VC_REQUEST] <= 1'b1;
                        cache_req_ready <= 1'b0;
                        req_state <= SENDING;
                    end
                end
                
                SENDING: begin
                    if (router_ready_in[VC_REQUEST]) begin
                        router_valid_out[VC_REQUEST] <= 1'b0;
                        req_state <= WAITING;
                    end
                end
                
                WAITING: begin
                    // Wait for response to come back through response channel
                    // Reset state when response is processed
                    if (resp_state == IDLE) begin
                        cache_req_ready <= 1'b1;
                        req_state <= IDLE;
                    end
                end
                
                default: req_state <= IDLE;
            endcase
        end
    end
    
    // Similar state machines for coherence channel and response processing
    // ...
    
    // Process incoming packets
    always_ff @(posedge clk or negedge reset_n) begin
        if (~reset_n) begin
            cache_resp_valid <= 1'b0;
            coh_resp_valid <= 1'b0;
            router_ready_out <= '1; // Ready to receive on all VCs
        end
        else begin
            // Default values
            cache_resp_valid <= 1'b0;
            coh_resp_valid <= 1'b0;
            
            // Process regular responses
            if (router_valid_in[VC_RESPONSE]) begin
                rx_packet = router_flit_in[VC_RESPONSE];
                
                // Check if this response is for us
                if (rx_packet.dest_id == CORE_ID) begin
                    // Memory response handling
                    case (rx_packet.msg_type)
                        RESP_READ_DATA, RESP_FETCH_DATA: begin
                            cache_resp_data <= rx_packet.data;
                            cache_resp_type <= rx_packet.msg_type;
                            cache_resp_valid <= 1'b1;
                        end
                        
                        RESP_WRITE_ACK: begin
                            cache_resp_type <= rx_packet.msg_type;
                            cache_resp_valid <= 1'b1;
                        end
                    endcase
                end
            end
            
            // Process coherence messages
            if (router_valid_in[VC_COHERENCE]) begin
                rx_packet = router_flit_in[VC_COHERENCE];
                
                // Check if this message is for us
                if (rx_packet.dest_id == CORE_ID) begin
                    // Coherence handling
                    case (rx_packet.msg_type)
                        COH_INVALIDATE, COH_SHARED_REQ, COH_EXCL_REQ: begin
                            coh_resp_addr <= rx_packet.addr;
                            coh_resp_type <= rx_packet.msg_type[2:0];
                            coh_resp_source <= rx_packet.source_id;
                            coh_resp_valid <= 1'b1;
                        end
                        
                        COH_INV_ACK, COH_DATA_RESP: begin
                            coh_resp_addr <= rx_packet.addr;
                            coh_resp_type <= rx_packet.msg_type[2:0];
                            coh_resp_source <= rx_packet.source_id;
                            coh_resp_data <= rx_packet.data;
                            coh_resp_valid <= 1'b1;
                        end
                    endcase
                end
            end
            
            // Acknowledge receipt of packets
            for (int i = 0; i < VC_COUNT; i++) begin
                if (router_valid_in[i] && router_ready_out[i]) begin
                    router_ready_out[i] <= 1'b0; // Deassert ready for one cycle
                end
                else begin
                    router_ready_out[i] <= 1'b1; // Ready to receive new packets
                end
            end
        end
    end
endmodule