`timescale 1ns / 1ns

// registers are 32 bits in RV32
`define REG_SIZE 31

// inst. are 32 bits in RV32IM
`define INST_SIZE 31

// RV opcodes are 7 bits
`define OPCODE_SIZE 6

`define DIVIDER_STAGES 8

// Don't forget your old codes
//`include "cla.v"
//`include "DividerUnsignedPipelined.v"

module RegFile (
  input      [        4:0] rd,
  input      [`REG_SIZE:0] rd_data,
  input      [        4:0] rs1,
  output reg [`REG_SIZE:0] rs1_data,
  input      [        4:0] rs2,
  output reg [`REG_SIZE:0] rs2_data,
  input                    clk,
  input                    we,
  input                    rst
);
  localparam NumRegs = 32;
  reg [`REG_SIZE:0] regs[0:NumRegs-1];
  integer i;

  // TODO: your code here
  always @(posedge clk) begin
    if (rst) begin
      for (i = 0; i < NumRegs; i = i + 1) begin
        regs[i] <= 0;
      end
    end else begin
      if (we && (rd != 0)) begin
        regs[rd] <= rd_data;
      end
    end
  end

  always @(*) begin
    rs1_data = (rs1 == 5'd0) ? 0 : regs[rs1];
    rs2_data = (rs2 == 5'd0) ? 0 : regs[rs2];
  end
  
endmodule

module DatapathPipelined (
  input                     clk,
  input                     rst,
  output     [ `REG_SIZE:0] pc_to_imem,
  input      [`INST_SIZE:0] inst_from_imem,
  // dmem is read/write
  output  [ `REG_SIZE:0] addr_to_dmem,
  input   [ `REG_SIZE:0] load_data_from_dmem,
  output  [ `REG_SIZE:0] store_data_to_dmem,
  output  [         3:0] store_we_to_dmem,
  output                    halt,
  // The PC of the inst currently in Writeback. 0 if not a valid inst.
  output  [ `REG_SIZE:0] trace_writeback_pc,
  // The bits of the inst currently in Writeback. 0 if not a valid inst.
  output  [`INST_SIZE:0] trace_writeback_inst
);

    // system stall
    wire stall_pipeline; 

    //Fetch -> Decode 
    wire [`REG_SIZE:0] if_pc;

    //Decode -> Execute 
    wire [`REG_SIZE:0] id_pc, id_rs1_data, id_rs2_data, id_imm;
    wire [`INST_SIZE:0] id_inst;
    wire [4:0] id_rs1_addr, id_rs2_addr, id_rd_addr;
    wire [4:0] id_alu_op;
    wire       id_reg_we, id_mem_we, id_branch, id_jal, id_jalr, id_load, id_auipc, id_lui;
    wire       id_is_div_op, id_div_signed, id_div_get_rem;
    wire [2:0] id_funct3;
    wire [1:0] id_op1_sel, id_op2_sel;

    // Execute -> Memory 
    wire [`REG_SIZE:0] ex_pc, ex_alu_result, ex_store_data;
    wire [`INST_SIZE:0] ex_inst;
    wire [4:0] ex_rd_addr, ex_rs2_addr;
    wire       ex_reg_we, ex_mem_we, ex_load;
    wire [2:0] ex_funct3;
    
    // Execute -> Fetch/Decode
    wire               branch_taken;
    wire [`REG_SIZE:0] branch_target;
    // Divider busy signal 
    wire [6:0]         div_busy_0, div_busy_1, div_busy_2, div_busy_3;
    wire [6:0]         div_busy_4, div_busy_5, div_busy_6, div_busy_7;
    // Divider result (Execute -> Writeback)
    wire [`REG_SIZE:0] div_quotient, div_remainder;

    // Memory -> Writeback
    wire [`REG_SIZE:0] mem_pc, mem_alu_result, mem_mem_data;
    wire [`INST_SIZE:0] mem_inst;
    wire [4:0]         mem_rd_addr;
    wire               mem_reg_we, mem_load;
    wire [2:0]         mem_funct3;
    wire [1:0]         mem_byte_offset;
    
    // forwarding signal (Memory -> Execute)
    wire [`REG_SIZE:0] m_result_fwd;

    // Writeback -> Decode/execute 
    wire               wb_final_we;
    wire [4:0]         wb_final_dst;
    wire [`REG_SIZE:0] wb_final_data;


  /****************/
  /* FETCH STAGE */
  /****************/
    StageFetch FETCH (
        .clk(clk), 
        .rst(rst),
        .stall_pipeline(stall_pipeline),
        .branch_taken(branch_taken),    
        .branch_target(branch_target),  
        .pc_to_imem(pc_to_imem),
        .f_pc_current(if_pc)
    );

  /****************/
  /* DECODE STAGE */
  /****************/
    StageDecode DECODE (
        .clk(clk), 
        .rst(rst),

        .f_pc(if_pc), 
        .f_inst(inst_from_imem),
        
        // Hazard & Control Inputs
        .branch_taken(branch_taken),
        .x_load(id_load),         // load-use hazard 
        .x_rd_addr(id_rd_addr),   // Check Hazard (dùng output của chính ID/EX)        
        // Writeback Inputs 
        .w_reg_we(wb_final_we),
        .w_rd_addr(wb_final_dst),
        .wb_data(wb_final_data),

        // Divider busy 
        .div_busy_0(div_busy_0), 
        .div_busy_1(div_busy_1), 
        .div_busy_2(div_busy_2), 
        .div_busy_3(div_busy_3),
        .div_busy_4(div_busy_4), 
        .div_busy_5(div_busy_5), 
        .div_busy_6(div_busy_6), 
        .div_busy_7(div_busy_7),

        // ID/EX Registers 
        .x_pc(id_pc), 
        .x_rs1_data(id_rs1_data), 
        .x_rs2_data(id_rs2_data), 
        .x_imm(id_imm), 
        .x_inst(id_inst),
        .x_rs1_addr(id_rs1_addr), 
        .x_rs2_addr(id_rs2_addr), 
        .x_rd_addr_out(id_rd_addr),
        .x_alu_op(id_alu_op),
        .x_reg_we(id_reg_we), 
        .x_mem_we(id_mem_we), 
        .x_branch(id_branch), 
        .x_jump(), .x_jal(id_jal), 
        .x_jalr(id_jalr), 
        .x_load_out(id_load), 
        .x_auipc(id_auipc), 
        .x_lui(id_lui),
        .x_is_div_op_out(id_is_div_op), 
        .x_div_signed(id_div_signed), 
        .x_div_get_rem(id_div_get_rem),
        .x_funct3(id_funct3), 
        .x_op1_sel(id_op1_sel), 
        .x_op2_sel(id_op2_sel),

        .halt (halt),
        .stall_req(stall_pipeline)
    );

  /****************/
  /* EXECUTE STAGE */
  /****************/
    StageExecute EXECUTE (
        .clk(clk), .rst(rst),
        // Inputs from ID/EX 
        .x_pc(id_pc), 
        .x_rs1_data(id_rs1_data), 
        .x_rs2_data(id_rs2_data), 
        .x_imm(id_imm),
        .x_rs1_addr(id_rs1_addr), 
        .x_rs2_addr(id_rs2_addr), 
        .x_rd_addr(id_rd_addr),
        .x_alu_op(id_alu_op),
        .x_reg_we_in(id_reg_we), 
        .x_mem_we_in(id_mem_we), 
        .x_branch(id_branch), 
        .x_jal(id_jal), 
        .x_jalr(id_jalr), 
        .x_load_in(id_load), 
        .x_auipc(id_auipc), 
        .x_lui(id_lui),
        .x_is_div_op(id_is_div_op), 
        .x_div_signed(id_div_signed), 
        .x_div_get_rem(id_div_get_rem),
        .x_funct3(id_funct3), 
        .x_op1_sel(id_op1_sel), 
        .x_op2_sel(id_op2_sel),
        .x_inst_in(id_inst),

        // Forwarding (MX and WX Bypass)
        .m_reg_we_fwd(ex_reg_we), 
        .m_rd_addr(ex_rd_addr), 
        .m_alu_result_fwd(ex_alu_result), 
        
        //  writeback input (2 cycles ahead)
        .w_reg_we(wb_final_we), 
        .w_rd_addr(wb_final_dst), 
        .wb_data_fwd(wb_final_data),

        // outputs: EX/MEM  
        .m_pc(ex_pc), 
        .m_alu_result(ex_alu_result), 
        .m_store_data(ex_store_data),
        .m_rd_addr_out(ex_rd_addr), 
        .m_rs2_addr_out(ex_rs2_addr),
        .m_reg_we_out(ex_reg_we), 
        .m_mem_we(ex_mem_we), 
        .m_load(ex_load),
        .m_funct3(ex_funct3), 
        .m_inst(ex_inst),

        // branch and hazard Outputs
        .branch_taken(branch_taken), 
        .branch_target(branch_target),
        .div_busy_0(div_busy_0), 
        .div_busy_1(div_busy_1), 
        .div_busy_2(div_busy_2), 
        .div_busy_3(div_busy_3),
        .div_busy_4(div_busy_4), 
        .div_busy_5(div_busy_5), 
        .div_busy_6(div_busy_6), 
        .div_busy_7(div_busy_7),
        .div_quotient(div_quotient), 
        .div_remainder(div_remainder)
    );

 /****************/
  /* MEMORY STAGE */
  /****************/
    StageMemory MEMORY (
        .clk(clk), .rst(rst),
        // inputs from EX/MEM 
        .m_pc(ex_pc), 
        .m_alu_result_in(ex_alu_result), 
        .m_store_data_in(ex_store_data),
        .m_rd_addr(ex_rd_addr), 
        .m_rs2_addr(ex_rs2_addr),
        .m_reg_we_in(ex_reg_we), 
        .m_mem_we_in(ex_mem_we), 
        .m_load_in(ex_load),
        .m_funct3_in(ex_funct3), 
        .m_inst_in(ex_inst),
        
        // forwarding  (WM Bypass - store data)
        .w_reg_we_fwd(wb_final_we), 
        .w_rd_addr_fwd(wb_final_dst), 
        .wb_data_fwd(wb_final_data),

        // memory Interface
        .addr_to_dmem(addr_to_dmem),
        .store_data_to_dmem(store_data_to_dmem),
        .store_we_to_dmem(store_we_to_dmem),
        .load_data_from_dmem(load_data_from_dmem),

        // otuputs: MEM/WB 
        .w_pc(mem_pc), 
        .w_alu_result(mem_alu_result), 
        .w_mem_data(mem_mem_data),
        .w_rd_addr_out(mem_rd_addr), 
        .w_reg_we(mem_reg_we), 
        .w_load(mem_load),
        .w_funct3(mem_funct3), 
        .w_byte_offset(mem_byte_offset), 
        .w_inst(mem_inst),
        .m_result_fwd(m_result_fwd)
    );

 /****************/
  /* WRITEBACK STAGE */
  /****************/
    StageWriteback WRITEBACK (
        // inputs from MEM/WB 
        .w_pc(mem_pc), 
        .w_alu_result(mem_alu_result), 
        .w_mem_data(mem_mem_data),
        .w_rd_addr(mem_rd_addr), 
        .w_reg_we(mem_reg_we), 
        .w_load(mem_load),
        .w_funct3(mem_funct3), 
        .w_byte_offset(mem_byte_offset), 
        .w_inst(mem_inst),

        // Divider result input (divider unit output)
        .div_valid(div_busy_7[6]),      // Valid bit of stage 7
        .div_get_rem(div_busy_7[5]),    // remainder flag
        .div_dst(div_busy_7[4:0]),      // destination register
        .div_quotient(div_quotient), 
        .div_remainder(div_remainder),

        // outputs
        .final_rf_we(wb_final_we),
        .final_rf_dst(wb_final_dst),
        .final_rf_data(wb_final_data),
        .trace_pc(trace_writeback_pc),
        .trace_inst(trace_writeback_inst)
    );

endmodule

module MemorySingleCycle #(
    parameter NUM_WORDS = 512
) (
    input                    rst,                 // rst for both imem and dmem
    input                    clk,                 // clock for both imem and dmem
	                                              // The memory reads/writes on @(negedge clk)
    input      [`REG_SIZE:0] pc_to_imem,          // must always be aligned to a 4B boundary
    output reg [`REG_SIZE:0] inst_from_imem,      // the value at memory location pc_to_imem
    input      [`REG_SIZE:0] addr_to_dmem,        // must always be aligned to a 4B boundary
    output     [`REG_SIZE:0] load_data_from_dmem, // the value at memory location addr_to_dmem
    input      [`REG_SIZE:0] store_data_to_dmem,  // the value to be written to addr_to_dmem, controlled by store_we_to_dmem
    // Each bit determines whether to write the corresponding byte of store_data_to_dmem to memory location addr_to_dmem.
    // E.g., 4'b1111 will write 4 bytes. 4'b0001 will write only the least-significant byte.
    input      [        3:0] store_we_to_dmem
);

  // memory is arranged as an array of 4B words
  reg [`REG_SIZE:0] mem_array[0:NUM_WORDS-1];

  // preload instructions to mem_array
  initial begin
    $readmemh("J:/Vivado project/SoC_Assignment/test.hex", mem_array);
  end



  localparam AddrMsb = $clog2(NUM_WORDS) + 1;
  localparam AddrLsb = 2;

  always @(negedge clk) begin
    inst_from_imem <= mem_array[{pc_to_imem[AddrMsb:AddrLsb]}];
  end
    
    assign load_data_from_dmem = mem_array[{addr_to_dmem[AddrMsb:AddrLsb]}];

  always @(negedge clk) begin
    if (store_we_to_dmem[0]) begin
      mem_array[addr_to_dmem[AddrMsb:AddrLsb]][7:0] <= store_data_to_dmem[7:0];
    end
    if (store_we_to_dmem[1]) begin
      mem_array[addr_to_dmem[AddrMsb:AddrLsb]][15:8] <= store_data_to_dmem[15:8];
    end
    if (store_we_to_dmem[2]) begin
      mem_array[addr_to_dmem[AddrMsb:AddrLsb]][23:16] <= store_data_to_dmem[23:16];
    end
    if (store_we_to_dmem[3]) begin
      mem_array[addr_to_dmem[AddrMsb:AddrLsb]][31:24] <= store_data_to_dmem[31:24];
    end
    // dmem is "read-first": read returns value before the write
  end
endmodule

/* This design has just one clock for both processor and memory. */
module Processor (
    input                 clk,
    input                 rst,
    output                halt,
    output [ `REG_SIZE:0] trace_writeback_pc,
    output [`INST_SIZE:0] trace_writeback_inst
);

  wire [`INST_SIZE:0] inst_from_imem;
  wire [ `REG_SIZE:0] pc_to_imem, mem_data_addr, mem_data_loaded_value, mem_data_to_write;
  wire [         3:0] mem_data_we;

  // This wire is set by cocotb to the name of the currently-running test, to make it easier
  // to see what is going on in the waveforms.
  wire [(8*32)-1:0] test_case;

  MemorySingleCycle #(
      .NUM_WORDS(512)
  ) memory (
    .rst                 (rst),
    .clk                 (clk),
    // imem is read-only
    .pc_to_imem          (pc_to_imem),
    .inst_from_imem      (inst_from_imem),
    // dmem is read-write
    .addr_to_dmem        (mem_data_addr),
    .load_data_from_dmem (mem_data_loaded_value),
    .store_data_to_dmem  (mem_data_to_write),
    .store_we_to_dmem    (mem_data_we)
  );

  DatapathPipelined datapath (
    .clk                  (clk),
    .rst                  (rst),
    .pc_to_imem           (pc_to_imem),
    .inst_from_imem       (inst_from_imem),
    .addr_to_dmem         (mem_data_addr),
    .store_data_to_dmem   (mem_data_to_write),
    .store_we_to_dmem     (mem_data_we),
    .load_data_from_dmem  (mem_data_loaded_value),
    .halt                 (halt),
    .trace_writeback_pc   (trace_writeback_pc),
    .trace_writeback_inst (trace_writeback_inst)
  );

endmodule