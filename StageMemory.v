`define REG_SIZE 31
`define INST_SIZE 31
`define OPCODE_SIZE 6

module StageMemory (
    input                    clk,
    input                    rst,

    input      [`REG_SIZE:0] m_pc, 
    input      [`REG_SIZE:0] m_alu_result_in, 
    input      [`REG_SIZE:0] m_store_data_in,
    input      [4:0]         m_rd_addr, 
    input      [4:0]         m_rs2_addr, // used to compare WM Bypass
    input                    m_reg_we_in, 
    input                    m_mem_we_in, 
    input                    m_load_in,
    input      [2:0]         m_funct3_in,
    input      [`INST_SIZE:0] m_inst_in,

    input                    w_reg_we_fwd,  
    input      [4:0]         w_rd_addr_fwd, 
    input      [`REG_SIZE:0] wb_data_fwd, 

    output     [`REG_SIZE:0] addr_to_dmem,
    output reg [`REG_SIZE:0] store_data_to_dmem,
    output reg [3:0]         store_we_to_dmem,
    input      [`REG_SIZE:0] load_data_from_dmem,

    output reg [`REG_SIZE:0] w_pc, 
    output reg [`REG_SIZE:0] w_alu_result, 
    output reg [`REG_SIZE:0] w_mem_data,
    output reg [4:0]         w_rd_addr_out,
    output reg               w_reg_we,     
    output reg               w_load,
    output reg [2:0]         w_funct3,
    output reg [1:0]         w_byte_offset,
    output reg [`INST_SIZE:0] w_inst,
    // to execute forwarding
    output     [`REG_SIZE:0] m_result_fwd
);
    `include "params.vh"
    // WM bypass
    reg [`REG_SIZE:0] final_store_data;
    
    always @(*) begin
        if (w_reg_we_fwd && w_rd_addr_fwd != 0 && w_rd_addr_fwd == m_rs2_addr) 
            final_store_data = wb_data_fwd; // form WB bypass 
        else 
            final_store_data = m_store_data_in; // from default execute 
    end

    assign addr_to_dmem = m_alu_result_in;
    
    // make we mask
    always @(*) begin
        if (m_mem_we_in) begin
            case (m_funct3_in)
               3'b000: begin // SB
                   if (m_alu_result_in[1:0] == 0) store_we_to_dmem = 4'b0001;
                   else if (m_alu_result_in[1:0] == 1) store_we_to_dmem = 4'b0010;
                   else if (m_alu_result_in[1:0] == 2) store_we_to_dmem = 4'b0100;
                   else store_we_to_dmem = 4'b1000;
               end
               3'b001: begin // SH
                   if (m_alu_result_in[1] == 0) store_we_to_dmem = 4'b0011;
                   else store_we_to_dmem = 4'b1100;
               end
               default: store_we_to_dmem = 4'b1111; // SW
            endcase
        end else begin
            store_we_to_dmem = 4'b0000;
        end
    end

    // store shift
    always @(*) begin
        if (m_funct3_in == 3'b000) store_data_to_dmem = {4{final_store_data[7:0]}};
        else if (m_funct3_in == 3'b001) store_data_to_dmem = {2{final_store_data[15:0]}};
        else store_data_to_dmem = final_store_data;
    end

    // MX bypass
    assign m_result_fwd = m_alu_result_in;

    // MEM/WB 
    always @(posedge clk) begin
        if (rst) begin
            w_reg_we <= 0; 
            w_inst <= 0;
            w_pc <= 0; 
            w_alu_result <= 0; 
            w_mem_data <= 0;
            w_rd_addr_out <= 0; 
            w_load <= 0; 
            w_funct3 <= 0; 
            w_byte_offset <= 0;
        end else begin
            w_pc          <= m_pc;
            w_alu_result  <= m_alu_result_in;
            w_mem_data    <= load_data_from_dmem;
            w_rd_addr_out <= m_rd_addr;
            w_reg_we      <= m_reg_we_in;
            w_load        <= m_load_in;
            w_funct3      <= m_funct3_in;
            w_byte_offset <= m_alu_result_in[1:0];
            w_inst        <= m_inst_in;
        end
    end

endmodule