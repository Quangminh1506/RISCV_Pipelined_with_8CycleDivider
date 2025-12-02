
// opcodes
localparam [`OPCODE_SIZE:0] OPC_LOAD = 7'b00_000_11;
localparam [`OPCODE_SIZE:0] OPC_STORE = 7'b01_000_11;
localparam [`OPCODE_SIZE:0] OPC_BRANCH = 7'b11_000_11;
localparam [`OPCODE_SIZE:0] OPC_JALR = 7'b11_001_11;
localparam [`OPCODE_SIZE:0] OPC_JAL = 7'b11_011_11;
localparam [`OPCODE_SIZE:0] OPC_OP_IMM = 7'b00_100_11;
localparam [`OPCODE_SIZE:0] OPC_OP_REG = 7'b01_100_11;
localparam [`OPCODE_SIZE:0] OPC_ENVIRON = 7'b11_100_11;
localparam [`OPCODE_SIZE:0] OPC_AUIPC = 7'b00_101_11;
localparam [`OPCODE_SIZE:0] OPC_LUI = 7'b01_101_11;

// ALU operations
localparam ALU_ADD = 5'b00000;
localparam ALU_SUB = 5'b00001;
localparam ALU_AND = 5'b00010;
localparam ALU_OR = 5'b00011;
localparam ALU_XOR = 5'b00100;
localparam ALU_SLT = 5'b00101; 
localparam ALU_SLTU = 5'b00110; 
localparam ALU_SLL = 5'b00111;
localparam ALU_SRL = 5'b01000;
localparam ALU_SRA = 5'b01001;
localparam ALU_MUL = 5'b01010;
localparam ALU_MULH = 5'b01011;
localparam ALU_MULHSU = 5'b01100;
localparam ALU_MULHU  = 5'b01101;

// Funct3 definitions
localparam F3_ADD_SUB = 3'b000;
localparam F3_SLL = 3'b001;
localparam F3_SLT = 3'b010;
localparam F3_SLTU = 3'b011;
localparam F3_XOR = 3'b100;
localparam F3_SRL_SRA = 3'b101;
localparam F3_OR = 3'b110;
localparam F3_AND = 3'b111;
localparam F3_MUL = 3'b000;
localparam F3_MULH = 3'b001;
localparam F3_MULHSU = 3'b010;
localparam F3_MULHU = 3'b011;
localparam F3_DIV = 3'b100;
localparam F3_DIVU = 3'b101;
localparam F3_REM = 3'b110;
localparam F3_REMU = 3'b111;

localparam F3_BEQ     = 3'b000;
localparam F3_BNE     = 3'b001;
localparam F3_BLT     = 3'b100;
localparam F3_BGE     = 3'b101;
localparam F3_BLTU    = 3'b110;
localparam F3_BGEU    = 3'b111;

localparam F7_NORMAL = 7'b0000000;
localparam F7_SUB_SRA = 7'b0100000;
localparam F7_M_EXT = 7'b0000001;
