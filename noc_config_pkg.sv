package noc_config_pkg;
    // System Configuration
    parameter int NUM_CORES = 4;
    parameter int ROWS = 2;
    parameter int COLS = 2;
    
    // Memory Configuration
    parameter int ADDR_WIDTH = 48;
    parameter int DATA_WIDTH = 64;
    parameter int MEM_SIZE_MB = 1024;  // 1GB memory
    
    // Cache Configuration
    parameter int L1I_CACHE_SIZE_KB = 128;
    parameter int L1D_CACHE_SIZE_KB = 256;
    parameter int L2_CACHE_SIZE_MB = 2;
    parameter int L3_CACHE_SIZE_MB = 8;
    parameter int CACHE_LINE_SIZE = 64;  // bytes
    parameter int L1_ASSOCIATIVITY = 8;
    parameter int L2_ASSOCIATIVITY = 16;
    parameter int L3_ASSOCIATIVITY = 16;
    
    // NoC Configuration
    parameter int FLIT_WIDTH = 128;
    parameter int VC_COUNT = 3;
    parameter int BUFFER_DEPTH = 4;
    
    // Coherence Protocol
    parameter logic [2:0] INVALID   = 3'b000;
    parameter logic [2:0] SHARED    = 3'b001;
    parameter logic [2:0] EXCLUSIVE = 3'b010;
    parameter logic [2:0] OWNED     = 3'b011;
    parameter logic [2:0] MODIFIED  = 3'b100;
    
    // NoC Message Types
    parameter logic [2:0] REQ_READ          = 3'b000;  // Read request
    parameter logic [2:0] REQ_READ_EXCL     = 3'b001;  // Read exclusive request
    parameter logic [2:0] REQ_WRITE         = 3'b010;  // Write request
    parameter logic [2:0] REQ_INVALIDATE    = 3'b011;  // Invalidate request
    parameter logic [2:0] RESP_DATA         = 3'b100;  // Data response
    parameter logic [2:0] RESP_DATA_EXCL    = 3'b101;  // Data response with exclusivity
    parameter logic [2:0] RESP_ACK          = 3'b110;  // Acknowledgment
    
    // Common Cache Address Extraction Functions
    function automatic int get_offset_bits(int line_size);
        return $clog2(line_size);
    endfunction
    
    function automatic int get_index_bits(int size_bytes, int line_size, int associativity);
        return $clog2(size_bytes/(line_size * associativity));
    endfunction
    
    function automatic int get_tag_bits(int addr_width, int index_bits, int offset_bits);
        return addr_width - index_bits - offset_bits;
    endfunction
endpackage