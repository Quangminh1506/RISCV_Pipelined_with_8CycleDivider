`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 12/02/2025 06:05:30 PM
// Design Name: 
// Module Name: StageDecode
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
`define REG_SIZE 31
`define INST_SIZE 31
`define OPCODE_SIZE 6

module StageDecode (
    input clk,
    input rst,

    input [`REG_SIZE:0] f_pc,
    input [`INST_SIZE:0] f_inst,
   
    input branch_taken, 
    input x_load,       
    input [4:0] x_rd_addr,   
   
    input w_reg_we,
    input [4:0] w_rd_addr,
    input [`REG_SIZE:0] wb_data,
    input [6:0] div_busy_0, div_busy_1, div_busy_2, div_busy_3, div_busy_4, div_busy_5, div_busy_6, div_busy_7,

    output reg [`REG_SIZE:0] x_pc, x_rs1_data, x_rs2_data, x_imm, x_inst,
    output reg [4:0] x_rs1_addr, x_rs2_addr, x_rd_addr_out,
    output reg [4:0] x_alu_op,
    output reg x_reg_we, x_mem_we, x_branch, x_jump, x_jal, x_jalr, x_load_out, x_auipc, x_lui,
    output reg x_is_div_op_out, x_div_signed, x_div_get_rem,
    output reg [2:0] x_funct3,
    output reg [1:0] x_op1_sel, x_op2_sel,
    output reg halt,
    output stall_req
);
    `include "params.vh"
    
    reg [`REG_SIZE:0] d_pc;
    reg [`INST_SIZE:0] d_inst;
    
    always @(posedge clk) begin
        if (rst || branch_taken) begin
            d_pc <= 0;
            d_inst <= 0; // NOP
        end else if (!stall_req) begin
            d_pc <= f_pc;
            d_inst <= f_inst;
        end
    end

    wire [6:0] opcode = d_inst[6:0];
    wire [4:0] rd     = d_inst[11:7];
    wire [2:0] funct3 = d_inst[14:12];
    wire [4:0] rs1    = d_inst[19:15];
    wire [4:0] rs2    = d_inst[24:20];
    wire [6:0] funct7 = d_inst[31:25];

    // immediate 
    reg [`REG_SIZE:0] imm_gen;
    wire [`REG_SIZE:0] imm_i = {{20{d_inst[31]}}, d_inst[31:20]};
    wire [`REG_SIZE:0] imm_s = {{20{d_inst[31]}}, d_inst[31:25], d_inst[11:7]};
    wire [`REG_SIZE:0] imm_b = {{19{d_inst[31]}}, d_inst[31], d_inst[7], d_inst[30:25], d_inst[11:8], 1'b0};
    wire [`REG_SIZE:0] imm_j = {{11{d_inst[31]}}, d_inst[31], d_inst[19:12], d_inst[20], d_inst[30:21], 1'b0};
    wire [`REG_SIZE:0] imm_u = {d_inst[31:12], 12'b0};
    
    // Hazard detection
    // Divider denpendency
    reg conflict_div;
    integer i;
    always @(*) begin
        conflict_div = 0;
        if ((div_busy_0[6] && div_busy_0[4:0] != 0 && (div_busy_0[4:0] == rs1 || div_busy_0[4:0] == rs2)) ||
            (div_busy_1[6] && div_busy_1[4:0] != 0 && (div_busy_1[4:0] == rs1 || div_busy_1[4:0] == rs2)) ||
            (div_busy_2[6] && div_busy_2[4:0] != 0 && (div_busy_2[4:0] == rs1 || div_busy_2[4:0] == rs2)) ||
            (div_busy_3[6] && div_busy_3[4:0] != 0 && (div_busy_3[4:0] == rs1 || div_busy_3[4:0] == rs2)) ||
            (div_busy_4[6] && div_busy_4[4:0] != 0 && (div_busy_4[4:0] == rs1 || div_busy_4[4:0] == rs2)) ||
            (div_busy_5[6] && div_busy_5[4:0] != 0 && (div_busy_5[4:0] == rs1 || div_busy_5[4:0] == rs2)) ||
            (div_busy_6[6] && div_busy_6[4:0] != 0 && (div_busy_6[4:0] == rs1 || div_busy_6[4:0] == rs2)) ||
            (div_busy_7[6] && div_busy_7[4:0] != 0 && (div_busy_7[4:0] == rs1 || div_busy_7[4:0] == rs2)))
            conflict_div = 1;
                
        if (x_is_div_op_out && (x_rd_addr != 0)) begin
            if (x_rd_addr == rs1 || x_rd_addr == rs2) begin
                conflict_div = 1;
            end
        end    
    end
    // Load hazard
    wire stall_load_use;
    assign stall_load_use = (x_load && x_rd_addr != 0 && (x_rd_addr == rs1 || x_rd_addr == rs2));

    //writeback collision (becuz div and ALU ops run in parallel)
    wire collision_at_mem;

    assign collision_at_mem = (div_busy_5[6] == 1'b1) && (ctrl_reg_we == 1'b1) && (ctrl_is_div_op == 1'b0);

    //stall system
    assign stall_req = conflict_div || stall_load_use || collision_at_mem;

    wire [`REG_SIZE:0] rf_rs1_data_raw, rf_rs2_data_raw;
    reg [`REG_SIZE:0] rf_rs1_data_fwd, rf_rs2_data_fwd;

    RegFile rf_inst (
        .clk(clk), 
        .rst(rst), 
        .we(w_reg_we), 
        .rd(w_rd_addr), 
        .rd_data(wb_data),
        .rs1(rs1), 
        .rs2(rs2), 
        .rs1_data(rf_rs1_data_raw), 
        .rs2_data(rf_rs2_data_raw)
    );

    // WD bypass
    always @(*) begin
        if (w_reg_we && (w_rd_addr != 0) && (w_rd_addr == rs1)) rf_rs1_data_fwd = wb_data;
        else rf_rs1_data_fwd = rf_rs1_data_raw;

        if (w_reg_we && (w_rd_addr != 0) && (w_rd_addr == rs2)) rf_rs2_data_fwd = wb_data;
        else rf_rs2_data_fwd = rf_rs2_data_raw;
    end
    
    // control logic
    reg [4:0] ctrl_alu_op;
    reg ctrl_reg_we, ctrl_mem_we, ctrl_branch, ctrl_jal, ctrl_jalr, ctrl_load, ctrl_auipc, ctrl_lui;
    reg ctrl_is_div_op, ctrl_div_signed, ctrl_div_get_rem;
    reg [1:0] ctrl_op1_sel, ctrl_op2_sel;
    reg [3:0] ctrl_mem_mask;

    always @(*) begin
        ctrl_alu_op = ALU_ADD; 
        ctrl_reg_we = 0; 
        ctrl_mem_we = 0; 
        ctrl_branch = 0;
        ctrl_jal = 0; 
        ctrl_jalr = 0; 
        ctrl_load = 0; 
        ctrl_auipc = 0; 
        ctrl_lui = 0;
        ctrl_is_div_op = 0; 
        ctrl_div_signed = 0; 
        ctrl_div_get_rem = 0;
        ctrl_op1_sel = 0; 
        ctrl_op2_sel = 0;
        imm_gen = imm_i; 

        case (opcode)
            OPC_OP_REG: begin
                ctrl_reg_we = 1;
                if (funct7 == F7_M_EXT) begin
                    case (funct3)
                        F3_MUL:    ctrl_alu_op = ALU_MUL;
                        F3_MULH:   ctrl_alu_op = ALU_MULH;
                        F3_MULHSU: ctrl_alu_op = ALU_MULHSU;
                        F3_MULHU:  ctrl_alu_op = ALU_MULHU;
                        F3_DIV:  begin ctrl_is_div_op=1; ctrl_div_signed=1; ctrl_div_get_rem=0; end
                        F3_DIVU: begin ctrl_is_div_op=1; ctrl_div_signed=0; ctrl_div_get_rem=0; end
                        F3_REM:  begin ctrl_is_div_op=1; ctrl_div_signed=1; ctrl_div_get_rem=1; end
                        F3_REMU: begin ctrl_is_div_op=1; ctrl_div_signed=0; ctrl_div_get_rem=1; end
                    endcase
                end else if (funct7 == F7_SUB_SRA) begin
                    if (funct3 == F3_ADD_SUB) ctrl_alu_op = ALU_SUB;
                    else ctrl_alu_op = ALU_SRA;
                end else begin
                    case(funct3)
                        F3_ADD_SUB: ctrl_alu_op = ALU_ADD;
                        F3_SLL:     ctrl_alu_op = ALU_SLL;
                        F3_SLT:     ctrl_alu_op = ALU_SLT;
                        F3_SLTU:    ctrl_alu_op = ALU_SLTU;
                        F3_XOR:     ctrl_alu_op = ALU_XOR;
                        F3_SRL_SRA: ctrl_alu_op = ALU_SRL;
                        F3_OR:      ctrl_alu_op = ALU_OR;
                        F3_AND:     ctrl_alu_op = ALU_AND;
                    endcase
                end
            end
            OPC_OP_IMM: begin
                ctrl_reg_we = 1; ctrl_op2_sel = 1;
                imm_gen = imm_i;
                case(funct3)
                    F3_ADD_SUB: ctrl_alu_op = ALU_ADD;
                    F3_SLT:     ctrl_alu_op = ALU_SLT;
                    F3_SLTU:    ctrl_alu_op = ALU_SLTU;
                    F3_XOR:     ctrl_alu_op = ALU_XOR;
                    F3_OR:      ctrl_alu_op = ALU_OR;
                    F3_AND:     ctrl_alu_op = ALU_AND;
                    F3_SLL:     ctrl_alu_op = ALU_SLL;
                    F3_SRL_SRA: ctrl_alu_op = (funct7[5]) ? ALU_SRA : ALU_SRL;
                endcase
            end
            OPC_LOAD: begin
                ctrl_reg_we = 1; ctrl_load = 1; ctrl_op2_sel = 1; ctrl_alu_op = ALU_ADD; imm_gen = imm_i;
            end
            OPC_STORE: begin
                ctrl_mem_we = 1; ctrl_op2_sel = 2; ctrl_alu_op = ALU_ADD; imm_gen = imm_s;
            end
            OPC_BRANCH: begin
                ctrl_branch = 1; ctrl_alu_op = ALU_ADD; imm_gen = imm_b;
            end
            OPC_JAL: begin
                ctrl_jal = 1; ctrl_reg_we = 1; imm_gen = imm_j;
            end
            OPC_JALR: begin
                ctrl_jalr = 1; ctrl_reg_we = 1; ctrl_op2_sel = 1; imm_gen = imm_i;
            end
            OPC_LUI: begin
                ctrl_lui = 1; ctrl_reg_we = 1; imm_gen = imm_u;
            end
            OPC_AUIPC: begin
                ctrl_auipc = 1; ctrl_reg_we = 1; imm_gen = imm_u;
            end
            OPC_ENVIRON: if (funct3 == 0) halt = 1;
        endcase
    end

    always @(posedge clk) begin
        if (rst || branch_taken || stall_req) begin
            x_reg_we <= 0;
            x_mem_we <= 0;
            x_branch <= 0;
            x_jal <= 0; 
            x_jalr <= 0;
            x_load_out    <= 0;
            x_auipc <= 0; 
            x_lui <= 0;
            x_is_div_op_out   <= 0; 
            x_div_signed <= 0; 
            x_div_get_rem <= 0;
            x_inst <= 0;
            x_alu_op <= ALU_ADD;
        end else begin
            x_pc <= d_pc;
            x_rs1_data <= rf_rs1_data_fwd;
            x_rs2_data <= rf_rs2_data_fwd;
            x_imm <= imm_gen;
            x_rs1_addr <= rs1;
            x_rs2_addr <= rs2;
            x_rd_addr_out <= rd;
            
            x_alu_op <= ctrl_alu_op;
            x_reg_we <= ctrl_reg_we;
            x_mem_we <= ctrl_mem_we;
            x_branch <= ctrl_branch;
            x_jal <= ctrl_jal;
            x_jalr <= ctrl_jalr;
            x_load_out <= ctrl_load;
            x_auipc <= ctrl_auipc;
            x_lui <= ctrl_lui;
            x_is_div_op_out <= ctrl_is_div_op;
            x_div_signed  <= ctrl_div_signed;
            x_div_get_rem <= ctrl_div_get_rem;
            
            x_funct3 <= funct3;
            x_op1_sel <= ctrl_op1_sel;
            x_op2_sel <= ctrl_op2_sel;
            x_inst <= d_inst;
        end
    end

endmodule
