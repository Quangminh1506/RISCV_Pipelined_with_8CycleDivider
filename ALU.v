`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/25/2025 02:30:50 PM
// Design Name: 
// Module Name: ALU
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

module ALU(
    input [`REG_SIZE:0] A,
    input [`REG_SIZE:0] B,
    input [3:0] ALUOp, 
    output reg [`REG_SIZE:0] result,
    output Zero     
);
    // --- ALUOp Codes ---
    localparam [3:0] ADD = 4'b0000;
    localparam [3:0] SUB = 4'b0001;
    localparam [3:0] AND = 4'b0010;
    localparam [3:0] OR  = 4'b0011;
    localparam [3:0] XOR = 4'b0100;
    localparam [3:0] SLT = 4'b0101; 
    localparam [3:0] SLTU = 4'b0110; 
    localparam [3:0] SLL = 4'b0111;
    localparam [3:0] SRL = 4'b1000;
    localparam [3:0] SRA = 4'b1001;
    // M Extension 
    localparam [3:0] MUL    = 4'b1010;
    localparam [3:0] MULH   = 4'b1011;
    localparam [3:0] MULHSU = 4'b1100;
    localparam [3:0] MULHU  = 4'b1101;

    // --- CLA Adder Logic ---
    wire [`REG_SIZE:0] B_inv;
    wire cla_cin;
    wire [`REG_SIZE:0] cla_Result;
    
    assign cla_cin = (ALUOp == SUB) || (ALUOp == SLT) || (ALUOp == SLTU); 
    assign B_inv = (cla_cin) ? ~B : B;

    cla cla_adder (
        .a    (A),
        .b    (B_inv),
        .cin  (cla_cin),
        .sum  (cla_Result)
    );

    wire [63:0] mul_signed_signed = $signed(A) * $signed(B);
    wire [63:0] mul_signed_unsigned = $signed(A) * B;
    wire [63:0] mul_unsigned_unsigned = A * B;
    
    always @(*) begin
        result = 0;
        case (ALUOp)
            ADD, SUB: result = cla_Result;
            AND:      result = A & B;
            OR:       result = A | B;
            XOR:      result = A ^ B;
            
            SLT:      result = ($signed(A) < $signed(B)) ? 32'd1 : 32'd0;
            SLTU:     result = (A < B) ? 32'd1 : 32'd0;
            
            SLL:      result = A << B[4:0];
            SRL:      result = A >> B[4:0];
            SRA:      result = $signed(A) >>> B[4:0];
            
            MUL:      result = mul_unsigned_unsigned[31:0];
            MULH:     result = mul_signed_signed[63:32];
            MULHSU:   result = mul_signed_unsigned[63:32];
            MULHU:    result = mul_unsigned_unsigned[63:32];

            default:  result = 0;
        endcase
    end

    assign Zero = (result == 32'd0);
endmodule
