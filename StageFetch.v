`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 12/02/2025 06:03:50 PM
// Design Name: 
// Module Name: StageFetch
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

module StageFetch (
    input clk,
    input rst,
    input stall_pipeline, 
    input branch_taken,   
    input [`REG_SIZE:0] branch_target,  

    output [`REG_SIZE:0] pc_to_imem,

    output [`REG_SIZE:0] f_pc_current
);
    `include "params.vh"
    reg [`REG_SIZE:0] pc_reg;
    wire [`REG_SIZE:0] pc_next;

    assign pc_next = (branch_taken) ? branch_target : (pc_reg + 4);

    always @(posedge clk) begin
        if (rst) begin
            pc_reg <= 32'd0;
        end else if (!stall_pipeline) begin
            pc_reg <= pc_next;
        end
    end
    // send PC to imem
    assign pc_to_imem = pc_reg;
    assign f_pc_current = pc_reg;

endmodule
