`define REG_SIZE 31
`define INST_SIZE 31
`define OPCODE_SIZE 6

module StageExecute (
    input                    clk,
    input                    rst,
    
    input      [`REG_SIZE:0] x_pc, x_rs1_data, x_rs2_data, x_imm, 
    input      [4:0]         x_rs1_addr, x_rs2_addr, x_rd_addr,
    input      [4:0]         x_alu_op,
    input                    x_reg_we_in, x_mem_we_in, x_branch, x_jal, x_jalr, x_load_in, x_auipc, x_lui,
    input                    x_is_div_op, x_div_signed, x_div_get_rem,
    input      [2:0]         x_funct3,
    input      [1:0]         x_op1_sel, x_op2_sel,
    input      [`INST_SIZE:0] x_inst_in,

    input                    m_reg_we_fwd, w_reg_we,
    input      [4:0]         m_rd_addr, w_rd_addr,
    input      [`REG_SIZE:0] m_alu_result_fwd, // MX bypass Value
    input      [`REG_SIZE:0] wb_data_fwd,      // WX bypass Value


    output reg [`REG_SIZE:0] m_pc, m_alu_result, m_store_data,
    output reg [4:0]         m_rd_addr_out,
    output reg [4:0]         m_rs2_addr_out, 
    output reg               m_reg_we_out, 
    output reg               m_mem_we, m_load,
    output reg [2:0]         m_funct3,
    output reg [`INST_SIZE:0] m_inst,

    output                   branch_taken,
    output     [`REG_SIZE:0] branch_target,
    
    // Hazard Info
    output reg [6:0]         div_busy_0, div_busy_1, div_busy_2, div_busy_3,
    output reg [6:0]         div_busy_4, div_busy_5, div_busy_6, div_busy_7,
    output     [`REG_SIZE:0] div_quotient, div_remainder,
    //Trace div inst
    output reg [`REG_SIZE:0] div_pc_done,
    output reg [`INST_SIZE:0] div_inst_done    
    
);
    `include "params.vh"
    // forward unit
    reg [`REG_SIZE:0] alu_op1_fwd, alu_op2_fwd;

    // Forwarding RS1
    always @(*) begin
        // MX bypass
        if (m_reg_we_fwd && m_rd_addr != 0 && m_rd_addr == x_rs1_addr) 
            alu_op1_fwd = m_alu_result_fwd;
        // WX bypass
        else if (w_reg_we && w_rd_addr != 0 && w_rd_addr == x_rs1_addr) 
            alu_op1_fwd = wb_data_fwd;      
        // default from ID/EX
        else alu_op1_fwd = x_rs1_data;
    end

    // Forwarding RS2
    always @(*) begin
        if (m_reg_we_fwd && m_rd_addr != 0 && m_rd_addr == x_rs2_addr) 
            alu_op2_fwd = m_alu_result_fwd;
        else if (w_reg_we && w_rd_addr != 0 && w_rd_addr == x_rs2_addr) 
            alu_op2_fwd = wb_data_fwd;      
        else 
            alu_op2_fwd = x_rs2_data;
    end

    // ALU and branch ---
    wire [`REG_SIZE:0] alu_in1 = (x_auipc) ? x_pc : ((x_lui) ? 0 : alu_op1_fwd);
    wire [`REG_SIZE:0] alu_in2 = (x_jal || x_auipc || x_lui || x_op2_sel == 1 || x_op2_sel == 2) ? x_imm : alu_op2_fwd;

    wire [`REG_SIZE:0] alu_result_wire;
    
    ALU alu_module (
        .A(alu_in1), .B(alu_in2), .ALUOp(x_alu_op), .result(alu_result_wire), .Zero()
    );

    wire branch_cond = (x_branch && (
        (x_funct3 == F3_BEQ  && alu_op1_fwd == alu_op2_fwd) || 
        (x_funct3 == F3_BNE  && alu_op1_fwd != alu_op2_fwd) || 
        (x_funct3 == F3_BLT  && $signed(alu_op1_fwd) < $signed(alu_op2_fwd)) || 
        (x_funct3 == F3_BGE  && $signed(alu_op1_fwd) >= $signed(alu_op2_fwd))|| 
        (x_funct3 == F3_BLTU && alu_op1_fwd < alu_op2_fwd)  || 
        (x_funct3 == F3_BGEU && alu_op1_fwd >= alu_op2_fwd)    
    ));
    
    assign branch_taken = branch_cond || x_jal || x_jalr;
    assign branch_target = (x_jalr) ? (alu_op1_fwd + x_imm) : (x_pc + x_imm);

    wire [`REG_SIZE:0] x_final_result = (x_jal || x_jalr) ? (x_pc + 4) : alu_result_wire;

    //Divider
    DividerPipelined div_inst (
        .clk(clk), .rst(rst), .stall(1'b0), .is_signed(x_div_signed),
        .i_dividend(alu_op1_fwd), .i_divisor(alu_op2_fwd),
        .o_remainder(div_remainder), .o_quotient(div_quotient)
    );

    //track div inst
    reg [`REG_SIZE:0] div_pc_shift [0:7];
    reg [`INST_SIZE:0] div_inst_shift [0:7];
    
    

    reg [6:0] div_busy [0:7];
    integer j;
    
    always @(posedge clk) begin
        if (rst) begin
            for(j=0; j<8; j=j+1) begin
                div_busy[j] <= 0;
                div_pc_shift[j] <= 0;
                div_inst_shift[j] <= 0;
            end
        end else begin
            for(j=1; j<8; j=j+1) begin
                div_busy[j] <= div_busy[j-1];
                div_pc_shift[j] <= div_pc_shift[j-1];
                div_inst_shift[j] <= div_inst_shift[j-1];
            end
            div_busy[0] <= {x_is_div_op, x_div_get_rem, x_rd_addr};
            div_pc_shift[0] <= x_pc;   // save current pc
            div_inst_shift[0] <= x_inst_in; // save current inst
        end
    end

    always @(*) begin
        div_busy_0 = div_busy[0]; 
        div_busy_1 = div_busy[1];
        div_busy_2 = div_busy[2]; 
        div_busy_3 = div_busy[3];
        div_busy_4 = div_busy[4]; 
        div_busy_5 = div_busy[5];
        div_busy_6 = div_busy[6]; 
        div_busy_7 = div_busy[7];
        
        //output trace
        div_pc_done = div_pc_shift[7];   
        div_inst_done = div_inst_shift[7];
    end

    always @(posedge clk) begin
        if (rst) begin
            m_reg_we_out   <= 0; 
            m_mem_we       <= 0; 
            m_load         <= 0;
            m_inst         <= 0;
            m_pc           <= 0;
            m_alu_result   <= 0;
            m_store_data   <= 0;
            m_rd_addr_out  <= 0;
            m_rs2_addr_out <= 0;
            m_funct3       <= 0;
        end else begin
            m_reg_we_out   <= (x_is_div_op) ? 0 : x_reg_we_in; 
            m_mem_we       <= x_mem_we_in;
            
            m_pc           <= x_pc;
            m_alu_result   <= x_final_result;
            m_store_data   <= alu_op2_fwd; 
            m_rd_addr_out  <= x_rd_addr;  
            m_rs2_addr_out <= x_rs2_addr; 
            
            m_funct3       <= x_funct3; 
            m_load         <= x_load_in;
            m_inst         <= x_inst_in;
        end
    end

endmodule
