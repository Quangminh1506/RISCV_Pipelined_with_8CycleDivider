`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 12/02/2025 07:39:24 PM
// Design Name: 
// Module Name: StageWriteback
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

module StageWriteback (
    input      clk,
    input      rst,
    input      [`REG_SIZE:0] w_pc,
    input      [`REG_SIZE:0] w_alu_result,
    input      [`REG_SIZE:0] w_mem_data, 
    input      [4:0]         w_rd_addr,
    input                    w_reg_we,
    input                    w_load,
    input      [2:0]         w_funct3,
    input      [1:0]         w_byte_offset, 
    input      [`INST_SIZE:0] w_inst,

    output                   final_rf_we,
    output     [4:0]         final_rf_dst,
    output     [`REG_SIZE:0] final_rf_data,

    output reg    [`REG_SIZE:0] trace_pc,
    output reg    [`INST_SIZE:0] trace_inst
);
    `include "params.vh"
    reg [`REG_SIZE:0] processed_load_data;
    reg [7:0] lb_byte;
    reg [15:0] lh_half;
    
    always @(*) begin
        // byte offset
        case (w_byte_offset)
            2'b00: lb_byte = w_mem_data[7:0];
            2'b01: lb_byte = w_mem_data[15:8];
            2'b10: lb_byte = w_mem_data[23:16];
            2'b11: lb_byte = w_mem_data[31:24];
        endcase
        
        case (w_byte_offset[1])
            1'b0: lh_half = w_mem_data[15:0];
            1'b1: lh_half = w_mem_data[31:16];
        endcase
        
        // sign extend
        case (w_funct3)
            3'b000: processed_load_data = {{24{lb_byte[7]}}, lb_byte};   // LB
            3'b001: processed_load_data = {{16{lh_half[15]}}, lh_half}; // LH
            3'b100: processed_load_data = {24'b0, lb_byte};             // LBU
            3'b101: processed_load_data = {16'b0, lh_half};             // LHU
            default: processed_load_data = w_mem_data;
        endcase
    end

    wire [`REG_SIZE:0] wb_data_main;
    assign wb_data_main = (w_load) ? processed_load_data : w_alu_result;

    assign final_rf_we   = w_reg_we;
    assign final_rf_dst  = w_rd_addr;
    assign final_rf_data = wb_data_main;

    always @(posedge clk) begin
        if (rst) begin
            trace_pc <= 0;
            trace_inst <= 0;
        end else begin
            trace_pc <= (w_inst == 0) ? 0 : w_pc;
            trace_inst <= w_inst;
        end
    end

endmodule
