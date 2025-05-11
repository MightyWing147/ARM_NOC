module arm_core #(
    parameter CORE_ID = 0
)(
    input  logic        clk,
    input  logic        reset_n,
    
    // Memory interface
    output logic [47:0] mem_addr,
    output logic        mem_read,
    output logic        mem_write,
    output logic [63:0] mem_wdata,
    output logic [7:0]  mem_byte_en,
    input  logic [63:0] mem_rdata,
    input  logic        mem_ready,
    
    // Interrupt interface
    input  logic [3:0]  interrupt,
    
    // Debug interface
    input  logic        debug_halt,
    input  logic        debug_run,
    input  logic        debug_step,
    input  logic [47:0] debug_pc,
    input  logic        debug_pc_wr_en,
    
    // Status interface
    output logic [3:0]  core_state,
    output logic        halted
);

    // Processor state
    typedef enum logic [3:0] {
        STATE_RESET,
        STATE_FETCH,
        STATE_DECODE,
        STATE_EXECUTE,
        STATE_MEMORY,
        STATE_WRITEBACK,
        STATE_HALTED,
        STATE_EXCEPTION
    } core_state_t;
    
    core_state_t current_state, next_state;
    
    // Program Counter
    logic [63:0] pc_current, pc_next;
    
    // Instruction Register
    logic [31:0] ir;
    
    // Register File
    logic [63:0] reg_file [0:30];  // General-purpose registers (X0-X30)
    logic [63:0] sp;               // Stack Pointer (X31)
    
    // Flags Register (NZCV)
    logic N, Z, C, V;
    
    // ALU signals
    logic [63:0] alu_op_a, alu_op_b, alu_op_c;
    logic [63:0] alu_result;
    logic [5:0]  alu_opcode;
    logic        alu_carry_in;
    logic        alu_N, alu_Z, alu_C, alu_V;
    logic [3:0]  alu_EReg;
    logic [1:0]  alu_SReg;
    logic        alu_Sh;
    
    // Control signals
    logic        reg2loc;
    logic        uncond_branch;
    logic        branch;
    logic        mem_read_ctrl;
    logic        mem_to_reg;
    logic [4:0]  alu_op;
    logic        mem_write_ctrl;
    logic        alu_src;
    logic        reg_write;
    logic [3:0]  extend_reg;
    logic [1:0]  shift_reg;
    logic        sh;
    logic [2:0]  imm3;
    logic [5:0]  imm6;
    logic [11:0] imm12;
    logic [6:0]  imms;
    
    // Instruction decode signals
    logic [4:0]  rd, rn, rm, ra;
    logic [63:0] imm_extended;
    
    // Memory access signals
    logic [63:0] mem_addr_calculated;
    logic [63:0] mem_data_to_write;
    
    // Writeback signals
    logic [63:0] write_data;
    logic [4:0]  write_reg;
    logic        write_enable;
    
    // Exception handling
    logic [63:0] exception_vector;
    logic        take_exception;
    logic [3:0]  exception_type;
    
    // Debug signals
    logic        debug_mode;
    
    //-------------------------------------------------------------------------
    // Program Counter logic
    //-------------------------------------------------------------------------
    always_ff @(posedge clk or negedge reset_n) begin
        if (~reset_n) begin
            pc_current <= 64'h0;
        end else if (debug_pc_wr_en) begin
            pc_current <= debug_pc;
        end else if (take_exception) begin
            pc_current <= exception_vector;
        end else if (current_state == STATE_WRITEBACK && !halted) begin
            pc_current <= pc_next;
        end
    end
    
    always_comb begin
        // Default next PC calculation (PC + 4)
        pc_next = pc_current + 4;
        
        // Branching logic
        if (uncond_branch) begin
            // Calculate target address for unconditional branch
            pc_next = pc_current + {{38{ir[25]}}, ir[25:0], 2'b00};
        end else if (branch) begin
            // Calculate target address for conditional branch
            pc_next = pc_current + {{45{ir[23]}}, ir[23:5], 2'b00};
            
            // Check condition
            case (ir[31:28])
                4'b0000: begin // EQ - equal (Z=1)
                    if (!Z) pc_next = pc_current + 4;
                end
                4'b0001: begin // NE - not equal (Z=0)
                    if (Z) pc_next = pc_current + 4;
                end
                4'b0010: begin // CS/HS - carry set/higher or same (C=1)
                    if (!C) pc_next = pc_current + 4;
                end
                4'b0011: begin // CC/LO - carry clear/lower (C=0)
                    if (C) pc_next = pc_current + 4;
                end
                4'b0100: begin // MI - minus/negative (N=1)
                    if (!N) pc_next = pc_current + 4;
                end
                4'b0101: begin // PL - plus/positive or zero (N=0)
                    if (N) pc_next = pc_current + 4;
                end
                4'b0110: begin // VS - overflow set (V=1)
                    if (!V) pc_next = pc_current + 4;
                end
                4'b0111: begin // VC - overflow clear (V=0)
                    if (V) pc_next = pc_current + 4;
                end
                4'b1000: begin // HI - higher (C=1 & Z=0)
                    if (!C || Z) pc_next = pc_current + 4;
                end
                4'b1001: begin // LS - lower or same (C=0 | Z=1)
                    if (C && !Z) pc_next = pc_current + 4;
                end
                4'b1010: begin // GE - greater than or equal (N=V)
                    if (N != V) pc_next = pc_current + 4;
                end
                4'b1011: begin // LT - less than (N!=V)
                    if (N == V) pc_next = pc_current + 4;
                end
                4'b1100: begin // GT - greater than (Z=0 & N=V)
                    if (Z || (N != V)) pc_next = pc_current + 4;
                end
                4'b1101: begin // LE - less than or equal (Z=1 | N!=V)
                    if (!Z && (N == V)) pc_next = pc_current + 4;
                end
                4'b1110: begin // AL - always
                    // Always take the branch
                end
                default: begin
                    // Default to not taking the branch
                    pc_next = pc_current + 4;
                end
            endcase
        end
    end
    
    //-------------------------------------------------------------------------
    // Instruction Fetch
    //-------------------------------------------------------------------------
    always_ff @(posedge clk or negedge reset_n) begin
        if (~reset_n) begin
            ir <= 32'h0;
        end else if (current_state == STATE_FETCH && mem_ready) begin
            ir <= mem_rdata[31:0];
        end
    end
    
    //-------------------------------------------------------------------------
    // Register File Logic
    //-------------------------------------------------------------------------
    // Extract register addresses from instruction
    always_comb begin
        rd = ir[4:0];    // Destination register
        rn = ir[9:5];    // First source register
        rm = ir[20:16];  // Second source register
        ra = ir[14:10];  // Third source register (used in some instructions)
    end
    
    // Register file read
    always_comb begin
        // First operand is always from Rn
        alu_op_a = (rn == 5'd31) ? sp : reg_file[rn];
        
        // Second operand can be from Rm or an immediate value
        if (alu_src) begin
            alu_op_b = imm_extended;
        end else begin
            alu_op_b = (rm == 5'd31) ? sp : reg_file[rm];
        end
        
        // Third operand for instructions like MADD
        alu_op_c = (ra == 5'd31) ? sp : reg_file[ra];
    end
    
    // Register file write
    always_ff @(posedge clk or negedge reset_n) begin
        if (~reset_n) begin
            for (int i = 0; i < 31; i++) begin
                reg_file[i] <= 64'h0;
            end
            sp <= 64'h0;
        end else if (current_state == STATE_WRITEBACK && reg_write && write_reg != 5'd31) begin
            if (write_reg != 5'd31 && write_reg != 5'd0) begin
                reg_file[write_reg] <= write_data;
            end
        end else if (current_state == STATE_WRITEBACK && reg_write && write_reg == 5'd31) begin
            sp <= write_data;
        end
    end
    
    // Immediate value extension logic
    always_comb begin
        imm_extended = 64'h0;
        
        case (extend_reg)
            4'b0000: begin // UXTB - Unsigned extend byte
                imm_extended = {56'h0, alu_op_a[7:0]};
            end
            4'b0001: begin // UXTH - Unsigned extend halfword
                imm_extended = {48'h0, alu_op_a[15:0]};
            end
            4'b0010: begin // UXTW - Unsigned extend word
                imm_extended = {32'h0, alu_op_a[31:0]};
            end
            4'b0011: begin // UXTX - Unsigned extend doubleword (identity)
                imm_extended = alu_op_a;
            end
            4'b0100: begin // SXTB - Signed extend byte
                imm_extended = {{56{alu_op_a[7]}}, alu_op_a[7:0]};
            end
            4'b0101: begin // SXTH - Signed extend halfword
                imm_extended = {{48{alu_op_a[15]}}, alu_op_a[15:0]};
            end
            4'b0110: begin // SXTW - Signed extend word
                imm_extended = {{32{alu_op_a[31]}}, alu_op_a[31:0]};
            end
            4'b0111: begin // SXTX - Signed extend doubleword (identity with sign extension)
                imm_extended = alu_op_a;
            end
            4'b1000: begin // 12-bit immediate with possible shift
                if (sh) begin
                    imm_extended = {52'h0, imm12} << 12;
                end else begin
                    imm_extended = {52'h0, imm12};
                end
            end
            4'b1001: begin // Shifted immediate for ADD, SUB family
                if (shift_reg == 2'b00) begin
                    imm_extended = {58'h0, imm6} << 0;
                end else if (shift_reg == 2'b01) begin
                    imm_extended = {58'h0, imm6} << 12;
                end
            end
            4'b1010: begin // Shifted immediate for logical operations
                if (shift_reg == 2'b00) begin
                    imm_extended = {61'h0, imm3} << 0;
                end else if (shift_reg == 2'b01) begin
                    imm_extended = {61'h0, imm3} << 12;
                end
            end
            4'b1011: begin // Scaled address offset
                imm_extended = {57'h0, imm6, 1'b0};
            end
            default: begin
                imm_extended = 64'h0;
            end
        endcase
    end
    
    //-------------------------------------------------------------------------
    // Control Unit Instantiation
    //-------------------------------------------------------------------------
    control_unit control_inst (
        .clk(clk),
        .rst(~reset_n),
        .opcode(ir),
        .Reg2Loc(reg2loc),
        .Uncondbranch(uncond_branch),
        .Branch(branch),
        .MemRead(mem_read_ctrl),
        .MemtoReg(mem_to_reg),
        .ALUOp(alu_op),
        .MemWrite(mem_write_ctrl),
        .ALUSrc(alu_src),
        .RegWrite(reg_write),
        .ExtendReg(extend_reg),
        .ShiftReg(shift_reg),
        .Sh(sh),
        .imm3(imm3),
        .imm6(imm6),
        .imm12(imm12),
        .imms(imms)
    );
    
    //-------------------------------------------------------------------------
    // ALU Instantiation
    //-------------------------------------------------------------------------
    assign alu_EReg = extend_reg[3:0];
    assign alu_SReg = shift_reg;
    assign alu_Sh = sh;
    assign alu_opcode = {1'b0, alu_op};
    assign alu_carry_in = C;
    
    alu alu_inst (
        .EReg(alu_EReg),
        .SReg(alu_SReg),
        .Sh(alu_Sh),
        .op_a(alu_op_a),
        .op_b(alu_op_b),
        .op_c(alu_op_c),
        .opcode(alu_opcode),
        .carry_in(alu_carry_in),
        .result(alu_result),
        .Z(alu_Z),
        .C(alu_C),
        .V(alu_V),
        .N(alu_N)
    );
    
    //-------------------------------------------------------------------------
    // Memory Access Logic
    //-------------------------------------------------------------------------
    always_comb begin
        mem_addr_calculated = alu_result;
        mem_data_to_write = (rm == 5'd31) ? sp : reg_file[rm];
        
        // Configure memory access signals based on control signals
        mem_addr = mem_addr_calculated;
        mem_read = mem_read_ctrl && (current_state == STATE_MEMORY);
        mem_write = mem_write_ctrl && (current_state == STATE_MEMORY);
        mem_wdata = mem_data_to_write;
        
        // Byte enables based on memory operation size
        // Default to 64-bit access (all 8 bytes enabled)
        mem_byte_en = 8'hFF;
        
        // Adjust byte enables based on memory access type in instruction
        case (ir[31:30])
            2'b00: begin // Byte access
                case (mem_addr_calculated[2:0])
                    3'b000: mem_byte_en = 8'h01;
                    3'b001: mem_byte_en = 8'h02;
                    3'b010: mem_byte_en = 8'h04;
                    3'b011: mem_byte_en = 8'h08;
                    3'b100: mem_byte_en = 8'h10;
                    3'b101: mem_byte_en = 8'h20;
                    3'b110: mem_byte_en = 8'h40;
                    3'b111: mem_byte_en = 8'h80;
                endcase
            end
            2'b01: begin // Halfword access
                case (mem_addr_calculated[2:1])
                    2'b00: mem_byte_en = 8'h03;
                    2'b01: mem_byte_en = 8'h0C;
                    2'b10: mem_byte_en = 8'h30;
                    2'b11: mem_byte_en = 8'hC0;
                endcase
            end
            2'b10: begin // Word access
                if (mem_addr_calculated[2])
                    mem_byte_en = 8'hF0;
                else
                    mem_byte_en = 8'h0F;
            end
            2'b11: begin // Doubleword access (default)
                mem_byte_en = 8'hFF;
            end
        endcase
    end
    
    //-------------------------------------------------------------------------
    // Writeback Logic
    //-------------------------------------------------------------------------
    always_comb begin
        // Determine the data to write back to registers
        if (mem_to_reg && mem_read_ctrl) begin
            write_data = mem_rdata;
        end else begin
            write_data = alu_result;
        end
        
        // Determine which register to write to
        write_reg = rd;
        
        // Write enable signal
        write_enable = reg_write && (current_state == STATE_WRITEBACK);
    end
    
    //-------------------------------------------------------------------------
    // State Machine Logic
    //-------------------------------------------------------------------------
    always_ff @(posedge clk or negedge reset_n) begin
        if (~reset_n) begin
            current_state <= STATE_RESET;
            N <= 1'b0;
            Z <= 1'b0;
            C <= 1'b0;
            V <= 1'b0;
            halted <= 1'b0;
        end else begin
            current_state <= next_state;
            
            // Update flags if needed (based on instruction type)
            if (current_state == STATE_EXECUTE && 
                (alu_op == 5'b00001 || alu_op == 5'b00011 || 
                 alu_op == 5'b00101 || alu_op == 5'b00111 || 
                 alu_op == 5'b01110 || alu_op == 5'b01111 || 
                 alu_op == 5'b10001 || alu_op == 5'b10010)) begin
                N <= alu_N;
                Z <= alu_Z;
                C <= alu_C;
                V <= alu_V;
            end
            
            // Set halted flag if entering halted state
            if (next_state == STATE_HALTED) begin
                halted <= 1'b1;
            end else if (next_state == STATE_FETCH && current_state == STATE_HALTED) begin
                halted <= 1'b0;
            end
        end
    end
    
    always_comb begin
        // Default next state
        next_state = current_state;
        
        case (current_state)
            STATE_RESET: begin
                next_state = STATE_FETCH;
            end
            
            STATE_FETCH: begin
                if (debug_halt || halted) begin
                    next_state = STATE_HALTED;
                end else if (mem_ready) begin
                    next_state = STATE_DECODE;
                end
            end
            
            STATE_DECODE: begin
                next_state = STATE_EXECUTE;
            end
            
            STATE_EXECUTE: begin
                if (mem_read_ctrl || mem_write_ctrl) begin
                    next_state = STATE_MEMORY;
                end else begin
                    next_state = STATE_WRITEBACK;
                end
            end
            
            STATE_MEMORY: begin
                if (mem_ready) begin
                    next_state = STATE_WRITEBACK;
                end
            end
            
            STATE_WRITEBACK: begin
                next_state = STATE_FETCH;
            end
            
            STATE_HALTED: begin
                if (debug_run) begin
                    next_state = STATE_FETCH;
                end
            end
            
            STATE_EXCEPTION: begin
                next_state = STATE_FETCH;
            end
            
            default: begin
                next_state = STATE_FETCH;
            end
        endcase
        
        // Handle exceptions
        if (take_exception && current_state != STATE_EXCEPTION) begin
            next_state = STATE_EXCEPTION;
        end
    end
    
    // Output the current state for debugging
    assign core_state = current_state;

endmodule