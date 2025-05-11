module control_unit(
    input  logic        clk,
    input  logic        rst,
    input  logic [31:0] opcode,  // Current instruction
    
    // Control signals for datapath
    output logic        Reg2Loc,
    output logic        Uncondbranch,
    output logic        Branch,
    output logic        MemRead,
    output logic        MemtoReg,
    output logic [4:0]  ALUOp,
    output logic        MemWrite,
    output logic        ALUSrc,
    output logic        RegWrite,
    output logic [3:0]  ExtendReg,
    output logic [1:0]  ShiftReg,
    output logic        Sh,
    output logic [2:0]  imm3,
    output logic [5:0]  imm6,
    output logic [11:0] imm12,
    output logic [6:0]  imms,
    output logic [6:0]  immr
);

    // Internal signals for instruction decoding
    logic [1:0]  op0;          // Major opcode field [31:30]
    logic [6:0]  op1;          // Opcode field [29:23]
    logic [3:0]  op2;          // Opcode field [22:19]
    logic [7:0]  op3;          // Opcode field [18:11]
    logic [2:0]  cond;         // Condition field [4:2]
    logic [4:0]  Rm, Rn, Rd;   // Register fields
    
    // Instruction class determination
    logic is_data_processing_imm;
    logic is_data_processing_reg;
    logic is_data_processing_simd;
    logic is_branches;
    logic is_loads_stores;
    
    // Extract major instruction fields
    assign op0 = opcode[31:30];
    assign op1 = opcode[29:23];
    assign op2 = opcode[22:19];
    assign op3 = opcode[18:11];
    assign Rm = opcode[20:16];
    assign Rn = opcode[9:5];
    assign Rd = opcode[4:0];
    
    // Determine instruction class
    always_comb begin
        // Default values - clear all instruction class flags
        is_data_processing_imm = 1'b0;
        is_data_processing_reg = 1'b0;
        is_data_processing_simd = 1'b0;
        is_branches = 1'b0;
        is_loads_stores = 1'b0;
        
        // Instruction class determination based on major opcode fields
        case (op0)
            2'b00: begin
                if (opcode[28] == 1'b0 && opcode[26] == 1'b0) begin
                    is_data_processing_imm = 1'b1; // PC-rel addressing, Add/Sub immediate
                end else if (opcode[28:27] == 2'b10) begin
                    is_loads_stores = 1'b1; // Load/Store instructions
                end else if (opcode[28:26] == 3'b110) begin
                    is_branches = 1'b1; // Conditional branches
                end else begin
                    is_data_processing_imm = 1'b1; // Other immediate data processing
                end
            end
            
            2'b01: begin
                if (opcode[28:26] == 3'b000) begin
                    is_data_processing_imm = 1'b1; // PC-rel addressing, Add/Sub immediate with tags
                end else if (opcode[28:26] == 3'b010) begin
                    is_loads_stores = 1'b1; // Load/Store register pair
                end else if (opcode[28:26] == 3'b100) begin
                    is_data_processing_simd = 1'b1; // AdvSIMD and FP instructions
                end else if (opcode[28:26] == 3'b110) begin
                    is_branches = 1'b1; // Unconditional branch (immediate)
                end else begin
                    is_data_processing_imm = 1'b1; // Logical immediate, Move immediate
                end
            end
            
            2'b10: begin
                if (opcode[28:24] == 5'b10101) begin
                    is_branches = 1'b1; // Compare & branch
                end else if (opcode[28:24] == 5'b11010) begin
                    is_branches = 1'b1; // Conditional branch (immediate)
                end else if (opcode[28:25] == 4'b1101) begin
                    is_branches = 1'b1; // Unconditional branch (register)
                end else if (opcode[28:25] == 4'b1000) begin
                    is_data_processing_reg = 1'b1; // Data processing (register)
                end else begin
                    is_loads_stores = 1'b1; // Load/Store variations
                end
            end
            
            2'b11: begin
                if (opcode[28:24] == 5'b00000) begin
                    is_loads_stores = 1'b1; // Advanced SIMD load/store
                end else if (opcode[28:24] == 5'b01000) begin
                    is_data_processing_reg = 1'b1; // Data processing (scalar FP and SIMD)
                end else begin
                    is_data_processing_reg = 1'b1; // Data processing (register)
                end
            end
        endcase
    end
    
    // Main control signal generation
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            Reg2Loc <= 1'b0;
            Uncondbranch <= 1'b0;
            Branch <= 1'b0;
            MemRead <= 1'b0;
            MemtoReg <= 1'b0;
            ALUOp <= 5'b00000;
            MemWrite <= 1'b0;
            ALUSrc <= 1'b0;
            RegWrite <= 1'b0;
            ExtendReg <= 4'b0000;
            ShiftReg <= 2'b00;
            Sh <= 1'b0;
            imm3 <= 3'b000;
            imm6 <= 6'b000000;
            imm12 <= 12'b000000000000;
            imms <= 7'b0000000;
            immr <= 7'b0000000;
        end else begin
            // Default values
            Reg2Loc <= 1'b0;
            Uncondbranch <= 1'b0;
            Branch <= 1'b0;
            MemRead <= 1'b0;
            MemtoReg <= 1'b0;
            ALUOp <= 5'b00000;
            MemWrite <= 1'b0;
            ALUSrc <= 1'b0;
            RegWrite <= 1'b0;
            ExtendReg <= 4'b0000;
            ShiftReg <= 2'b00;
            Sh <= 1'b0;
            imm3 <= opcode[18:16];  // Common immediate field extraction
            imm6 <= opcode[21:16];  // Common immediate field extraction
            imm12 <= opcode[21:10]; // Common immediate field extraction
            imms <= opcode[15:10];  // Common shift/mask fields
            immr <= opcode[21:16];  // Common rotate/shift fields
            
            // Process based on instruction class and specific opcode
            
            //----------------------------------------------------------------------
            // Branches processing
            //----------------------------------------------------------------------
            if (is_branches) begin
                // Check branch type
                if (opcode[31:26] == 6'b000101) begin 
                    // B - Unconditional branch
                    Uncondbranch <= 1'b1;
                    RegWrite <= 1'b0;
                end
                else if (opcode[31:24] == 8'b01010100) begin 
                    // B.cond - Conditional branch
                    Branch <= 1'b1;
                    RegWrite <= 1'b0;
                end
                else if (opcode[31:26] == 6'b100101) begin 
                    // BL - Branch with link
                    Uncondbranch <= 1'b1;
                    RegWrite <= 1'b1; // Write return address to X30
                    Rd = 5'd30; // X30 is the link register
                end
                else if (opcode[31:10] == 22'b1101011000011111000000) begin 
                    // BR - Branch to register
                    ALUOp <= 5'b10010; // Use MOV to pass Rm to ALU
                    RegWrite <= 1'b0;
                    ALUSrc <= 1'b0; // Use register value
                    Branch <= 1'b1;
                end
                else if (opcode[31:10] == 22'b1101011000111111000000) begin 
                    // BLR - Branch with link to register
                    ALUOp <= 5'b10010; // Use MOV to pass Rm to ALU
                    RegWrite <= 1'b1; // Write return address to X30
                    ALUSrc <= 1'b0; // Use register value
                    Branch <= 1'b1;
                    Rd = 5'd30; // X30 is the link register
                end
                else if (opcode[31:25] == 7'b0110100) begin 
                    // CBZ/CBNZ - Compare and Branch if (Not) Zero
                    ALUOp <= 5'b00111; // Compare operation
                    ALUSrc <= 1'b0; // Use register value
                    Branch <= 1'b1;
                    RegWrite <= 1'b0;
                end
            end
            
            //----------------------------------------------------------------------
            // Load/Store processing
            //----------------------------------------------------------------------
            else if (is_loads_stores) begin
                if (opcode[31:22] == 10'b1111100010 || // LDR (immediate)
                    opcode[31:22] == 10'b0111100010) begin // LDRB/LDRH (immediate)
                    MemRead <= 1'b1;
                    MemtoReg <= 1'b1;
                    RegWrite <= 1'b1;
                    ALUSrc <= 1'b1; // Use immediate offset
                    ALUOp <= 5'b00000; // ADD for address calculation
                    
                    // Set appropriate extend register mode based on access size
                    case (opcode[31:30])
                        2'b00: ExtendReg <= 4'b0000; // Byte access
                        2'b01: ExtendReg <= 4'b0001; // Halfword access
                        2'b10: ExtendReg <= 4'b0010; // Word access
                        2'b11: ExtendReg <= 4'b0011; // Doubleword access
                    endcase
                end
                else if (opcode[31:22] == 10'b1111100000 || // STR (immediate)
                         opcode[31:22] == 10'b0111100000) begin // STRB/STRH (immediate)
                    MemWrite <= 1'b1;
                    RegWrite <= 1'b0;
                    ALUSrc <= 1'b1; // Use immediate offset
                    ALUOp <= 5'b00000; // ADD for address calculation
                    
                    // Set appropriate extend register mode based on access size
                    case (opcode[31:30])
                        2'b00: ExtendReg <= 4'b0000; // Byte access
                        2'b01: ExtendReg <= 4'b0001; // Halfword access
                        2'b10: ExtendReg <= 4'b0010; // Word access
                        2'b11: ExtendReg <= 4'b0011; // Doubleword access
                    endcase
                end
                else if (opcode[31:22] == 10'b1111100011 || // LDR (register)
                         opcode[31:22] == 10'b0111100011) begin // LDRB/LDRH (register)
                    MemRead <= 1'b1;
                    MemtoReg <= 1'b1;
                    RegWrite <= 1'b1;
                    ALUSrc <= 1'b0; // Use register offset
                    ALUOp <= 5'b00000; // ADD for address calculation
                end
                else if (opcode[31:22] == 10'b1111100001 || // STR (register)
                         opcode[31:22] == 10'b0111100001) { // STRB/STRH (register)
                    MemWrite <= 1'b1;
                    RegWrite <= 1'b0;
                    ALUSrc <= 1'b0; // Use register offset
                    ALUOp <= 5'b00000; // ADD for address calculation
                }
                else if (opcode[31:22] == 10'b1011100010) begin // LDRSW (immediate)
                    MemRead <= 1'b1;
                    MemtoReg <= 1'b1;
                    RegWrite <= 1'b1;
                    ALUSrc <= 1'b1; // Use immediate offset
                    ALUOp <= 5'b00000; // ADD for address calculation
                    ExtendReg <= 4'b0110; // Sign extend word
                end
            end
            
            //----------------------------------------------------------------------
            // Data Processing Immediate
            //----------------------------------------------------------------------
            else if (is_data_processing_imm) begin
                // Common settings for most immediate instructions
                ALUSrc <= 1'b1; // Use immediate value
                RegWrite <= 1'b1; // Write result to register
                
                // ADD/ADDS immediate
                if (opcode[31:23] == 9'b100100010) begin // ADD immediate
                    ALUOp <= 5'b00000; // ADD operation
                    Sh <= opcode[22]; // Shift amount
                    ExtendReg <= 4'b1000; // 12-bit immediate
                end
                else if (opcode[31:23] == 9'b101100010) begin // ADDS immediate
                    ALUOp <= 5'b00001; // ADDS operation (sets flags)
                    Sh <= opcode[22]; // Shift amount
                    ExtendReg <= 4'b1000; // 12-bit immediate
                end
                
                // SUB/SUBS immediate
                else if (opcode[31:23] == 9'b110100010) begin // SUB immediate
                    ALUOp <= 5'b00010; // SUB operation
                    Sh <= opcode[22]; // Shift amount
                    ExtendReg <= 4'b1000; // 12-bit immediate
                end
                else if (opcode[31:23] == 9'b111100010) begin // SUBS immediate
                    ALUOp <= 5'b00011; // SUBS operation (sets flags)
                    Sh <= opcode[22]; // Shift amount
                    ExtendReg <= 4'b1000; // 12-bit immediate
                end
                
                // Logical operations immediate
                else if (opcode[31:23] == 9'b100100100) begin // AND immediate
                    ALUOp <= 5'b01001; // AND operation
                    ExtendReg <= 4'b1010; // Logical immediate encoding
                end
                else if (opcode[31:23] == 9'b101100100) begin // ORR immediate
                    ALUOp <= 5'b01010; // ORR operation
                    ExtendReg <= 4'b1010; // Logical immediate encoding
                end
                else if (opcode[31:23] == 9'b110100100) begin // EOR immediate
                    ALUOp <= 5'b01011; // EOR operation
                    ExtendReg <= 4'b1010; // Logical immediate encoding
                end
                else if (opcode[31:23] == 9'b111100100) begin // ANDS immediate
                    ALUOp <= 5'b10001; // ANDS operation (sets flags)
                    ExtendReg <= 4'b1010; // Logical immediate encoding
                end
                
                // MOVZ/MOVN/MOVK
                else if (opcode[31:23] == 9'b100100101) begin // MOV(Z/K/N) wide immediate
                    if (opcode[22:21] == 2'b00) begin
                        ALUOp <= 5'b10010; // MOV operation
                    end else if (opcode[22:21] == 2'b01) begin
                        ALUOp <= 5'b10011; // MVN operation
                    end
                    ShiftReg <= opcode[22:21]; // Shift amount (0, 16, 32, or 48)
                    ExtendReg <= 4'b1000; // 16-bit immediate
                end
                
                // CMP/CMN immediate
                else if (opcode[31:23] == 9'b111100010 && opcode[4:0] == 5'b11111) begin // CMP immediate
                    ALUOp <= 5'b00111; // CMP operation
                    RegWrite <= 1'b0; // Don't write result, just set flags
                    Sh <= opcode[22]; // Shift amount
                    ExtendReg <= 4'b1000; // 12-bit immediate
                end
                else if (opcode[31:23] == 9'b101100010 && opcode[4:0] == 5'b11111) begin // CMN immediate
                    ALUOp <= 5'b10000; // CMN operation
                    RegWrite <= 1'b0; // Don't write result, just set flags
                    Sh <= opcode[22]; // Shift amount
                    ExtendReg <= 4'b1000; // 12-bit immediate
                end
            end
            
            //----------------------------------------------------------------------
            // Data Processing Register
            //----------------------------------------------------------------------
            else if (is_data_processing_reg) begin
                // Common settings for most register instructions
                ALUSrc <= 1'b0; // Use register value
                RegWrite <= 1'b1; // Write result to register
                
                // ADD/ADDS register
                if (opcode[31:21] == 11'b00001011001) begin // ADD register
                    ALUOp <= 5'b00000; // ADD operation
                    ShiftReg <= opcode[23:22]; // Shift type
                    ExtendReg <= 4'b0011; // No extension
                end
                else if (opcode[31:21] == 11'b00101011001) begin // ADDS register
                    ALUOp <= 5'b00001; // ADDS operation (sets flags)
                    ShiftReg <= opcode[23:22]; // Shift type
                    ExtendReg <= 4'b0011; // No extension
                end
                
                // SUB/SUBS register
                else if (opcode[31:21] == 11'b10001011001) begin // SUB register
                    ALUOp <= 5'b00010; // SUB operation
                    ShiftReg <= opcode[23:22]; // Shift type
                    ExtendReg <= 4'b0011; // No extension
                end
                else if (opcode[31:21] == 11'b10101011001) begin // SUBS register
                    ALUOp <= 5'b00011; // SUBS operation (sets flags)
                    ShiftReg <= opcode[23:22]; // Shift type
                    ExtendReg <= 4'b0011; // No extension
                end
                
                // ADC/SBC register
                else if (opcode[31:21] == 11'b00011010000) begin // ADC register
                    ALUOp <= 5'b00100; // ADC operation
                end
                else if (opcode[31:21] == 11'b00111010000) begin // ADCS register
                    ALUOp <= 5'b00101; // ADCS operation (sets flags)
                end
                else if (opcode[31:21] == 11'b10011010000) begin // SBC register
                    ALUOp <= 5'b00110; // SBC operation
                end
                else if (opcode[31:21] == 11'b10111010000) begin // SBCS register
                    ALUOp <= 5'b00111; // SBCS operation (sets flags)
                end
                
                // Logical operations register
                else if (opcode[31:21] == 11'b00001010000) begin // AND register
                    ALUOp <= 5'b01001; // AND operation
                    ShiftReg <= opcode[23:22]; // Shift type
                end
                else if (opcode[31:21] == 11'b00101010000) begin // ORR register
                    ALUOp <= 5'b01010; // ORR operation
                    ShiftReg <= opcode[23:22]; // Shift type
                end
                else if (opcode[31:21] == 11'b10001010000) begin // EOR register
                    ALUOp <= 5'b01011; // EOR operation
                    ShiftReg <= opcode[23:22]; // Shift type
                end
                else if (opcode[31:21] == 11'b10101010000) begin // ANDS register
                    ALUOp <= 5'b10001; // ANDS operation (sets flags)
                    ShiftReg <= opcode[23:22]; // Shift type
                end
                
                // Shift operations
                else if (opcode[31:21] == 11'b00011010110) begin // LSL register
                    ALUOp <= 5'b01100; // LSL operation
                end
                else if (opcode[31:21] == 11'b00011010110) begin // LSR register
                    ALUOp <= 5'b01101; // LSR operation
                end
                else if (opcode[31:21] == 11'b00011010110) begin // ASR register
                    ALUOp <= 5'b10110; // ASR operation
                end
                else if (opcode[31:21] == 11'b00011010110) begin // ROR register
                    ALUOp <= 5'b10111; // ROR operation
                end
                
                // Multiplication operations
                else if (opcode[31:21] == 11'b00011011000) begin // MADD
                    if (opcode[15:10] == 6'b011111) begin
                        ALUOp <= 5'b10101; // MUL operation
                    end else begin
                        ALUOp <= 5'b10100; // MADD operation
                    end
                end
                else if (opcode[31:21] == 11'b00011011000) begin // MSUB
                    if (opcode[15:10] == 6'b011111) begin
                        ALUOp <= 5'b10111; // MNEG operation
                    end else begin
                        ALUOp <= 5'b10110; // MSUB operation
                    end
                end
                
                // Division operations
                else if (opcode[31:21] == 11'b00011010110 && opcode[15:10] == 6'b000011) begin // SDIV
                    ALUOp <= 5'b11001; // SDIV operation
                end
                else if (opcode[31:21] == 11'b00011010110 && opcode[15:10] == 6'b000010) begin // UDIV
                    ALUOp <= 5'b11010; // UDIV operation
                end
                
                // CMP/CMN register
                else if (opcode[31:21] == 11'b11001011001 && opcode[4:0] == 5'b11111) begin // CMP register
                    ALUOp <= 5'b00111; // CMP operation
                    RegWrite <= 1'b0; // Don't write result, just set flags
                    ShiftReg <= opcode[23:22]; // Shift type
                end
                else if (opcode[31:21] == 11'b00101011001 && opcode[4:0] == 5'b11111) begin // CMN register
                    ALUOp <= 5'b10000; // CMN operation
                    RegWrite <= 1'b0; // Don't write result, just set flags
                    ShiftReg <= opcode[23:22]; // Shift type
                end
                
                // Move operations
                else if (opcode[31:21] == 11'b01001010000 && opcode[9:5] == 5'b11111) begin // MOV register
                    ALUOp <= 5'b10010; // MOV operation
                end
                else if (opcode[31:21] == 11'b01001010001 && opcode[9:5] == 5'b11111) begin // MVN register
                    ALUOp <= 5'b10011; // MVN operation
                end
                
                // Extension operations
                else if (opcode[31:24] == 8'b00110100) begin // UXTB
                    ALUOp <= 5'b11000; // UXTB operation
                end
                else if (opcode[31:24] == 8'b00110100) begin // UXTH
                    ALUOp <= 5'b11001; // UXTH operation
                end
                else if (opcode[31:24] == 8'b00110100) begin // UXTW
                    ALUOp <= 5'b11010; // UXTW operation
                end
                else if (opcode[31:24] == 8'b10110100) begin // SXTB
                    ALUOp <= 5'b11100; // SXTB operation
                end
                else if (opcode[31:24] == 8'b10110100) begin // SXTH
                    ALUOp <= 5'b11101; // SXTH operation
                end
                else if (opcode[31:24] == 8'b10110100) begin // SXTW
                    ALUOp <= 5'b11110; // SXTW operation
                end
                
                // REV operations
                else if (opcode[31:10] == 22'b1101011010110000000010) begin // REV
                    ALUOp <= 5'b11000; // REV operation
                end
                else if (opcode[31:10] == 22'b1101011010110000000001) begin // REV16
                    ALUOp <= 5'b11001; // REV16 operation
                end
                else if (opcode[31:10] == 22'b1101011010110000000010) begin // REV32
                    ALUOp <= 5'b11010; // REV32 operation
                end
            end
            
            // Handle other instructions as needed
        end
    end
endmodule