module alu(
    input  logic [3:0]  EReg,        // Extended register mode
    input  logic [1:0]  SReg,        // Shift register mode
    input  logic        Sh,          // Shift flag
    input  logic [63:0] op_a,        // Operand A
    input  logic [63:0] op_b,        // Operand B
    input  logic [63:0] op_c,        // Operand C (for MADD/MSUB)
    input  logic [5:0]  opcode,      // Operation code
    input  logic        carry_in,    // Carry input for ADC/SBC
    output logic [63:0] result,      // Result output
    output logic        Z,           // Zero flag
    output logic        C,           // Carry flag
    output logic        V,           // Overflow flag
    output logic        N            // Negative flag
);

    // Internal signals
    logic [64:0] add_result;    // 65 bits to capture carry
    logic [64:0] sub_result;    // 65 bits to capture borrow
    logic [64:0] adc_result;    // 65 bits for add with carry
    logic [64:0] sbc_result;    // 65 bits for subtract with carry
    logic [64:0] neg_result;    // 65 bits for negation
    logic [63:0] div_result;    // Division result
    logic [63:0] sdiv_result;   // Signed division result
    logic [127:0] mul_result;   // 128-bit multiplication result
    logic [127:0] smul_result;  // Signed multiplication result
    logic [63:0] temp;          // Temporary register for operations
    logic [6:0]  immr, imms;    // Immediate shift values
    
    // ALU Operation Codes
    localparam ADD     = 6'b000000;  // Add
    localparam ADDS    = 6'b000001;  // Add and set flags
    localparam SUB     = 6'b000010;  // Subtract
    localparam SUBS    = 6'b000011;  // Subtract and set flags
    localparam ADC     = 6'b000100;  // Add with carry
    localparam ADCS    = 6'b000101;  // Add with carry and set flags
    localparam SBC     = 6'b000110;  // Subtract with carry
    localparam SBCS    = 6'b000111;  // Subtract with carry and set flags
    localparam NEG     = 6'b001000;  // Negate
    localparam AND     = 6'b001001;  // Bitwise AND
    localparam ORR     = 6'b001010;  // Bitwise OR
    localparam EOR     = 6'b001011;  // Bitwise XOR
    localparam LSL     = 6'b001100;  // Logical shift left
    localparam LSR     = 6'b001101;  // Logical shift right
    localparam NEGS    = 6'b001110;  // Negate and set flags
    localparam CMP     = 6'b001111;  // Compare (subtract and set flags but discard result)
    localparam CMN     = 6'b010000;  // Compare negative (add and set flags but discard result)
    localparam ANDS    = 6'b010001;  // Bitwise AND and set flags
    localparam MOV     = 6'b010010;  // Move
    localparam MVN     = 6'b010011;  // Move NOT
    localparam ORN     = 6'b010100;  // Bitwise OR NOT
    localparam EON     = 6'b010101;  // Bitwise XOR NOT
    localparam ASR     = 6'b010110;  // Arithmetic shift right
    localparam ROR     = 6'b010111;  // Rotate right
    localparam UXTB    = 6'b011000;  // Unsigned extend byte
    localparam UXTH    = 6'b011001;  // Unsigned extend halfword
    localparam UXTW    = 6'b011010;  // Unsigned extend word
    localparam UXTX    = 6'b011011;  // Copy (identity function)
    localparam SXTB    = 6'b011100;  // Signed extend byte
    localparam SXTH    = 6'b011101;  // Signed extend halfword
    localparam SXTW    = 6'b011110;  // Signed extend word
    localparam ADD_EXT = 6'b011111;  // ADD with extension option
    localparam SUB_EXT = 6'b100000;  // SUB with extension option
    localparam ADDS_EXT = 6'b100001; // ADDS with extension option
    localparam SUBS_EXT = 6'b100010; // SUBS with extension option
    localparam ADD_SFT = 6'b100011;  // ADD with shift option
    localparam ADDS_SFT = 6'b100100; // ADDS with shift option
    localparam SUB_SFT = 6'b100101;  // SUB with shift option
    localparam SUBS_SFT = 6'b100110; // SUBS with shift option
    localparam AND_SFT = 6'b100111;  // AND with shift option
    localparam ORR_SFT = 6'b101000;  // ORR with shift option
    localparam EOR_SFT = 6'b101001;  // EOR with shift option
    localparam EON_SFT = 6'b101010;  // EON with shift option
    localparam ANDS_SFT = 6'b101011; // ANDS with shift option
    localparam MADD    = 6'b101100;  // Multiply-Add
    localparam MUL     = 6'b101101;  // Multiply
    localparam MSUB    = 6'b101110;  // Multiply-Subtract
    localparam MNEG    = 6'b101111;  // Multiply-Negate
    localparam REV     = 6'b110000;  // Byte Reverse
    localparam REV16   = 6'b110001;  // Byte Reverse in halfwords
    localparam REV32   = 6'b110010;  // Byte Reverse in words
    localparam SDIV    = 6'b110011;  // Signed Divide
    localparam UDIV    = 6'b110100;  // Unsigned Divide
    localparam CCMN    = 6'b110101;  // Conditional Compare Negative
    localparam CCMP    = 6'b110110;  // Conditional Compare
    localparam CMP_EXT = 6'b110111;  // Compare with extension option
    localparam CMN_EXT = 6'b111000;  // Compare negative with extension option
    
    // Main ALU logic
    always_comb begin
        // Default values
        result = '0;
        N = 0;
        Z = 0;
        C = 0;
        V = 0;
        temp = '0;
        
        // Pre-compute common operations
        add_result = {1'b0, op_a} + {1'b0, op_b};
        sub_result = {1'b0, op_a} - {1'b0, op_b};
        adc_result = {1'b0, op_a} + {1'b0, op_b} + {64'b0, carry_in};
        sbc_result = {1'b0, op_a} - {1'b0, op_b} - {64'b0, ~carry_in};
        neg_result = {1'b0, 64'h0} - {1'b0, op_b};
        
        // Non-synthesizable division operations (for simulation only)
        // In a real implementation, these would be pipelined hardware dividers
        if (op_b == '0) begin
            div_result = '1;      // Return all ones for division by zero
            sdiv_result = '1;     // Return all ones for division by zero
        end else begin
            div_result = op_a / op_b;
            sdiv_result = $signed(op_a) / $signed(op_b);
        end
        
        // Multiplication operations (these would use DSP blocks in real hardware)
        mul_result = op_a * op_b;
        smul_result = $signed(op_a) * $signed(op_b);
        
        // ALU operation select
        case (opcode)
            // Basic Arithmetic
            ADD: begin
                // Add operation with optional shift
                if (Sh)
                    result = op_a + (op_b << 12);
                else
                    result = add_result[63:0];
            end
            
            ADDS: begin
                // Add and set flags with optional shift
                if (Sh)
                    result = op_a + (op_b << 12);
                else
                    result = add_result[63:0];
                    
                // Set flags
                N = result[63];
                Z = (result == 0);
                C = add_result[64];
                V = (~op_a[63] & ~op_b[63] & result[63]) | (op_a[63] & op_b[63] & ~result[63]);
            end
            
            SUB: begin
                // Subtract operation with optional shift
                if (Sh)
                    result = op_a - (op_b << 12);
                else
                    result = sub_result[63:0];
            end
            
            SUBS: begin
                // Subtract and set flags with optional shift
                if (Sh)
                    result = op_a - (op_b << 12);
                else
                    result = sub_result[63:0];
                    
                // Set flags
                N = result[63];
                Z = (result == 0);
                C = ~sub_result[64]; // ARM uses inverted borrow for Carry flag
                V = (~op_a[63] & op_b[63] & result[63]) | (op_a[63] & ~op_b[63] & ~result[63]);
            end
            
            ADC: begin
                result = adc_result[63:0];
            end
            
            ADCS: begin
                result = adc_result[63:0];
                // Set flags
                N = result[63];
                Z = (result == 0);
                C = adc_result[64];
                V = (~op_a[63] & ~op_b[63] & result[63]) | (op_a[63] & op_b[63] & ~result[63]);
            end
            
            SBC: begin
                result = sbc_result[63:0];
            end
            
            SBCS: begin
                result = sbc_result[63:0];
                // Set flags
                N = result[63];
                Z = (result == 0);
                C = ~sbc_result[64]; // ARM uses inverted borrow for Carry flag
                V = (~op_a[63] & op_b[63] & result[63]) | (op_a[63] & ~op_b[63] & ~result[63]);
            end
            
            NEG: begin
                temp = neg_result[63:0];
                // Apply shifting if requested
                case (SReg)
                    2'b00: result = temp << 6;
                    2'b01: result = temp >> 6;
                    2'b10: result = $signed(temp) >>> 6; // Arithmetic shift right
                    2'b11: result = (temp >> 6) | (temp << (64-6)); // Rotate right
                    default: result = temp;
                endcase
            end
            
            AND: begin
                temp = op_a & op_b;
                if (Sh)
                    result = temp << 12;
                else
                    result = temp;
            end
            
            ORR: begin
                temp = op_a | op_b;
                if (Sh)
                    result = temp << 12;
                else
                    result = temp;
            end
            
            EOR: begin
                temp = op_a ^ op_b;
                if (Sh)
                    result = temp << 12;
                else
                    result = temp;
            end
            
            LSL: begin
                result = op_a << op_b[5:0]; // Only low 6 bits specify shift amount
            end
            
            LSR: begin
                result = op_a >> op_b[5:0]; // Only low 6 bits specify shift amount
            end
            
            NEGS: begin
                temp = neg_result[63:0];
                // Apply shifting if requested
                case (SReg)
                    2'b00: result = temp << 6;
                    2'b01: result = temp >> 6;
                    2'b10: result = $signed(temp) >>> 6; // Arithmetic shift right
                    2'b11: result = (temp >> 6) | (temp << (64-6)); // Rotate right
                    default: result = temp;
                endcase
                
                // Set flags
                N = result[63];
                Z = (result == 0);
                C = neg_result[64];
                V = (op_b[63] & result[63]); // Overflow only when negating the smallest negative number
            end
            
            CMP: begin
                result = sub_result[63:0]; // Result is discarded but flags are set
                
                // Set flags
                N = result[63];
                Z = (result == 0);
                C = ~sub_result[64]; // ARM uses inverted borrow for Carry flag
                V = (~op_a[63] & op_b[63] & result[63]) | (op_a[63] & ~op_b[63] & ~result[63]);
            end
            
            CMN: begin
                result = add_result[63:0]; // Result is discarded but flags are set
                
                // Set flags
                N = result[63];
                Z = (result == 0);
                C = add_result[64];
                V = (~op_a[63] & ~op_b[63] & result[63]) | (op_a[63] & op_b[63] & ~result[63]);
            end
            
            ANDS: begin
                result = op_a & op_b;
                if (Sh)
                    result = result << 12;
                
                // Set flags
                N = result[63];
                Z = (result == 0);
                C = 0; // Cleared by logical operations
                V = 0; // Cleared by logical operations
            end
            
            MOV: begin
                result = op_b; // Direct move from operand B
            end
            
            MVN: begin
                result = ~op_b; // Bitwise NOT of operand B
            end
            
            ORN: begin
                result = op_a | ~op_b; // OR with NOT
            end
            
            EON: begin
                result = op_a ^ ~op_b; // XOR with NOT
            end
            
            ASR: begin
                result = $signed(op_a) >>> op_b[5:0]; // Arithmetic shift right
            end
            
            ROR: begin
                result = (op_a >> op_b[5:0]) | (op_a << (64 - op_b[5:0])); // Rotate right
            end
            
            UXTB: begin
                result = {56'd0, op_a[7:0]}; // Zero-extend byte
            end
            
            UXTH: begin
                result = {48'd0, op_a[15:0]}; // Zero-extend halfword
            end
            
            UXTW: begin
                result = {32'd0, op_a[31:0]}; // Zero-extend word
            end
            
            UXTX: begin
                result = op_a; // Identity (64-bit)
            end
            
            SXTB: begin
                result = {{56{op_a[7]}}, op_a[7:0]}; // Sign-extend byte
            end
            
            SXTH: begin
                result = {{48{op_a[15]}}, op_a[15:0]}; // Sign-extend halfword
            end
            
            SXTW: begin
                result = {{32{op_a[31]}}, op_a[31:0]}; // Sign-extend word
            end
            
            ADD_EXT: begin
                temp = add_result[63:0];
                
                // Apply extension operation based on EReg value
                case (EReg)
                    4'b0000: result = {56'd0, temp[7:0]};                // UXTB
                    4'b0001: result = {48'd0, temp[15:0]};               // UXTH
                    4'b0010: result = {32'd0, temp[31:0]};               // UXTW
                    4'b0011: result = temp;                              // UXTX
                    4'b0100: result = {{56{temp[7]}}, temp[7:0]};        // SXTB
                    4'b0101: result = {{48{temp[15]}}, temp[15:0]};      // SXTH
                    4'b0110: result = {{32{temp[31]}}, temp[31:0]};      // SXTW
                    4'b0111: result = temp;                              // SXTX
                    4'b1011: result = temp << 3;                         // LSL #3
                    default: result = temp;
                endcase
            end
            
            SUB_EXT: begin
                temp = sub_result[63:0];
                
                // Apply extension operation based on EReg value
                case (EReg)
                    4'b0000: result = {56'd0, temp[7:0]};                // UXTB
                    4'b0001: result = {48'd0, temp[15:0]};               // UXTH
                    4'b0010: result = {32'd0, temp[31:0]};               // UXTW
                    4'b0011: result = temp;                              // UXTX
                    4'b0100: result = {{56{temp[7]}}, temp[7:0]};        // SXTB
                    4'b0101: result = {{48{temp[15]}}, temp[15:0]};      // SXTH
                    4'b0110: result = {{32{temp[31]}}, temp[31:0]};      // SXTW
                    4'b0111: result = temp;                              // SXTX
                    4'b1011: result = temp << 3;                         // LSL #3
                    default: result = temp;
                endcase
            end
            
            ADDS_EXT: begin
                temp = add_result[63:0];
                
                // Apply extension operation based on EReg value
                case (EReg)
                    4'b0000: result = {56'd0, temp[7:0]};                // UXTB
                    4'b0001: result = {48'd0, temp[15:0]};               // UXTH
                    4'b0010: result = {32'd0, temp[31:0]};               // UXTW
                    4'b0011: result = temp;                              // UXTX
                    4'b0100: result = {{56{temp[7]}}, temp[7:0]};        // SXTB
                    4'b0101: result = {{48{temp[15]}}, temp[15:0]};      // SXTH
                    4'b0110: result = {{32{temp[31]}}, temp[31:0]};      // SXTW
                    4'b0111: result = temp;                              // SXTX
                    4'b1011: result = temp << 3;                         // LSL #3
                    default: result = temp;
                endcase
                
                // Set flags
                N = result[63];
                Z = (result == 0);
                C = add_result[64];
                V = (~op_a[63] & ~op_b[63] & result[63]) | (op_a[63] & op_b[63] & ~result[63]);
            end
            
            SUBS_EXT: begin
                temp = sub_result[63:0];
                
                // Apply extension operation based on EReg value
                case (EReg)
                    4'b0000: result = {56'd0, temp[7:0]};                // UXTB
                    4'b0001: result = {48'd0, temp[15:0]};               // UXTH
                    4'b0010: result = {32'd0, temp[31:0]};               // UXTW
                    4'b0011: result = temp;                              // UXTX
                    4'b0100: result = {{56{temp[7]}}, temp[7:0]};        // SXTB
                    4'b0101: result = {{48{temp[15]}}, temp[15:0]};      // SXTH
                    4'b0110: result = {{32{temp[31]}}, temp[31:0]};      // SXTW
                    4'b0111: result = temp;                              // SXTX
                    4'b1011: result = temp << 3;                         // LSL #3
                    default: result = temp;
                endcase
                
                // Set flags
                N = result[63];
                Z = (result == 0);
                C = ~sub_result[64]; // ARM uses inverted borrow for Carry flag
                V = (~op_a[63] & op_b[63] & result[63]) | (op_a[63] & ~op_b[63] & ~result[63]);
            end
            
            ADD_SFT: begin
                // ADD with shift option
                temp = add_result[63:0];
                
                case (SReg)
                    2'b00: result = temp << 6;
                    2'b01: result = temp >> 6;
                    2'b10: result = $signed(temp) >>> 6; // Arithmetic shift right
                    default: result = temp;
                endcase
            end
            
            ADDS_SFT: begin
                // ADD with shift option and set flags
                temp = add_result[63:0];
                
                case (SReg)
                    2'b00: result = temp << 6;
                    2'b01: result = temp >> 6;
                    2'b10: result = $signed(temp) >>> 6; // Arithmetic shift right
                    default: result = temp;
                endcase
                
                // Set flags
                N = result[63];
                Z = (result == 0);
                C = add_result[64];
                V = (~op_a[63] & ~op_b[63] & result[63]) | (op_a[63] & op_b[63] & ~result[63]);
            end
            
            SUB_SFT: begin
                // SUB with shift option
                temp = sub_result[63:0];
                
                case (SReg)
                    2'b00: result = temp << 6;
                    2'b01: result = temp >> 6;
                    2'b10: result = $signed(temp) >>> 6; // Arithmetic shift right
                    default: result = temp;
                endcase
            end
            
            SUBS_SFT: begin
                // SUB with shift option and set flags
                temp = sub_result[63:0];
                
                case (SReg)
                    2'b00: result = temp << 6;
                    2'b01: result = temp >> 6;
                    2'b10: result = $signed(temp) >>> 6; // Arithmetic shift right
                    default: result = temp;
                endcase
                
                // Set flags
                N = result[63];
                Z = (result == 0);
                C = ~sub_result[64]; // ARM uses inverted borrow for Carry flag
                V = (~op_a[63] & op_b[63] & result[63]) | (op_a[63] & ~op_b[63] & ~result[63]);
            end
            
            AND_SFT: begin
                // AND with shift option
                temp = op_a & op_b;
                
                case (SReg)
                    2'b00: result = temp << 6;
                    2'b01: result = temp >> 6;
                    2'b10: result = $signed(temp) >>> 6; // Arithmetic shift right
                    2'b11: result = (temp >> 6) | (temp << (64-6)); // Rotate right
                    default: result = temp;
                endcase
            end
            
            ORR_SFT: begin
                // ORR with shift option
                temp = op_a | op_b;
                
                case (SReg)
                    2'b00: result = temp << 6;
                    2'b01: result = temp >> 6;
                    2'b10: result = $signed(temp) >>> 6; // Arithmetic shift right
                    2'b11: result = (temp >> 6) | (temp << (64-6)); // Rotate right
                    default: result = temp;
                endcase
            end
            
            EOR_SFT: begin
                // EOR with shift option
                temp = op_a ^ op_b;
                
                case (SReg)
                    2'b00: result = temp << 6;
                    2'b01: result = temp >> 6;
                    2'b10: result = $signed(temp) >>> 6; // Arithmetic shift right
                    2'b11: result = (temp >> 6) | (temp << (64-6)); // Rotate right
                    default: result = temp;
                endcase
            end
            
            EON_SFT: begin
                // EON with shift option
                temp = op_a ^ ~op_b;
                
                case (SReg)
                    2'b00: result = temp << 6;
                    2'b01: result = temp >> 6;
                    2'b10: result = $signed(temp) >>> 6; // Arithmetic shift right
                    2'b11: result = (temp >> 6) | (temp << (64-6)); // Rotate right
                    default: result = temp;
                endcase
            end
            
            ANDS_SFT: begin
                // ANDS with shift option
                temp = op_a & op_b;
                
                case (SReg)
                    2'b00: result = temp << 6;
                    2'b01: result = temp >> 6;
                    2'b10: result = $signed(temp) >>> 6; // Arithmetic shift right
                    2'b11: result = (temp >> 6) | (temp << (64-6)); // Rotate right
                    default: result = temp;
                endcase
                
                // Set flags
                N = result[63];
                Z = (result == 0);
                C = 0; // Cleared by logical operations
                V = 0; // Cleared by logical operations
            end
            
            MADD: begin
                // Multiply-Add: op_a * op_b + op_c
                result = (op_a * op_b) + op_c;
            end
            
            MUL: begin
                // Multiply: op_a * op_b
                result = mul_result[63:0];
            end
            
            MSUB: begin
                // Multiply-Subtract: op_c - (op_a * op_b)
                result = op_c - (op_a * op_b);
            end
            
            MNEG: begin
                // Multiply-Negate: -(op_a * op_b)
                result = -(op_a * op_b);
            end
            
            REV: begin
                // Byte Reverse (endianness swap)
                result = {op_a[7:0], op_a[15:8], op_a[23:16], op_a[31:24], 
                          op_a[39:32], op_a[47:40], op_a[55:48], op_a[63:56]};
            end
            
            REV16: begin
                // Byte Reverse in halfwords
                result = {op_a[7:0], op_a[15:8], op_a[23:16], op_a[31:24], 
                          op_a[39:32], op_a[47:40], op_a[55:48], op_a[63:56]};
            end
            
            REV32: begin
                // Byte Reverse in words
                result = {op_a[7:0], op_a[15:8], op_a[23:16], op_a[31:24], 
                          op_a[39:32], op_a[47:40], op_a[55:48], op_a[63:56]};
            end
            
            SDIV: begin
                // Signed Division
                result = sdiv_result;
            end
            
            UDIV: begin
                // Unsigned Division
                result = div_result;
            end
            
            CCMN: begin
                // Conditional Compare Negative
                // Simplified implementation - in real hardware would check condition flags
                result = add_result[63:0];
                
                // Set flags
                N = result[63];
                Z = (result == 0);
                C = add_result[64];
                V = (~op_a[63] & ~op_b[63] & result[63]) | (op_a[63] & op_b[63] & ~result[63]);
            end
            
            CCMP: begin
                // Conditional Compare
                // Simplified implementation - in real hardware would check condition flags
                result = sub_result[63:0];
                
                // Set flags
                N = result[63];
                Z = (result == 0);
                C = ~sub_result[64]; // ARM uses inverted borrow for Carry flag
                V = (~op_a[63] & op_b[63] & result[63]) | (op_a[63] & ~op_b[63] & ~result[63]);
            end
            
            CMP_EXT: begin
                // Compare with extension option
                temp = sub_result[63:0]; // Result is discarded but flags are set
                
                // Apply extension operation based on EReg value
                case (EReg)
                    4'b0000: result = {56'd0, temp[7:0]};                // UXTB
                    4'b0001: result = {48'd0, temp[15:0]};               // UXTH
                    4'b0010: result = {32'd0, temp[31:0]};               // UXTW
                    4'b0011: result = temp;                              // UXTX
                    4'b0100: result = {{56{temp[7]}}, temp[7:0]};        // SXTB
                    4'b0101: result = {{48{temp[15]}}, temp[15:0]};      // SXTH
                    4'b0110: result = {{32{temp[31]}}, temp[31:0]};      // SXTW
                    4'b0111: result = temp;                              // SXTX
                    4'b1011: result = temp << 3;                         // LSL #
                    4'b0110: result = {{32{temp[31]}}, temp[31:0]};      // SXTW
                    4'b0111: result = temp;                              // SXTX
                    4'b1011: result = temp << 3;                         // LSL #3
                    default: result = temp;
                endcase
                
                // Set flags
                N = result[63];
                Z = (result == 0);
                C = ~sub_result[64]; // ARM uses inverted borrow for Carry flag
                V = (~op_a[63] & op_b[63] & result[63]) | (op_a[63] & ~op_b[63] & ~result[63]);
            end
            
            CMN_EXT: begin
                // Compare negative with extension option
                temp = add_result[63:0]; // Result is discarded but flags are set
                
                // Apply extension operation based on EReg value
                case (EReg)
                    4'b0000: result = {56'd0, temp[7:0]};                // UXTB
                    4'b0001: result = {48'd0, temp[15:0]};               // UXTH
                    4'b0010: result = {32'd0, temp[31:0]};               // UXTW
                    4'b0011: result = temp;                              // UXTX
                    4'b0100: result = {{56{temp[7]}}, temp[7:0]};        // SXTB
                    4'b0101: result = {{48{temp[15]}}, temp[15:0]};      // SXTH
                    4'b0110: result = {{32{temp[31]}}, temp[31:0]};      // SXTW
                    4'b0111: result = temp;                              // SXTX
                    4'b1011: result = temp << 3;                         // LSL #3
                    default: result = temp;
                endcase
                
                // Set flags
                N = result[63];
                Z = (result == 0);
                C = add_result[64];
                V = (~op_a[63] & ~op_b[63] & result[63]) | (op_a[63] & op_b[63] & ~result[63]);
            end
            
            default: begin
                result = '0;
            end
        endcase
    end
    
endmodule