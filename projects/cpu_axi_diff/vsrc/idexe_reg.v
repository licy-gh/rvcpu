
`include "defines.v"

module idexe_reg(
    input   wire    cpu_clk_50M,
    input   wire    cpu_rst_n,
    
    //来自译码阶段的信息
    input   wire [`ALUTYPE_BUS] id_alutype,
    input   wire [`ALUOP_BUS]   id_aluop,
    input   wire [`REG_BUS]     id_src1,
    input   wire [`REG_BUS]     id_src2,
    input   wire [`REG_ADDR_BUS]id_wa,
    input   wire                id_wreg,
    input   wire                id_mreg,
    input   wire[`REG_BUS]      id_din,
	input	wire[`INST_ADDR_BUS]id_pc,
	input	wire[`INST_BUS]		id_inst,
    
    //送至执行阶段的信息
    output  reg [`ALUTYPE_BUS]  exe_alutype,
    output  reg [`ALUOP_BUS]    exe_aluop,
    output  reg [`REG_BUS]      exe_src1,
    output  reg [`REG_BUS]      exe_src2,
    output  reg [`REG_ADDR_BUS] exe_wa,
    output  reg                 exe_wreg,
    output  reg                 exe_mreg,
    output  reg [`REG_BUS]      exe_din,
	output	reg [`INST_ADDR_BUS]exe_pc,
	output	reg [`INST_BUS]		exe_inst,

    //流水线暂停信号
    input   wire                id_stall,
    input   wire                data_read_stall,

    //跳转指令添加
    input   wire [`REG_BUS  ]   id_ret_addr,
    output  reg  [`REG_BUS  ]   exe_ret_addr,

    //csr指令添加
    input   wire [`REG_ADDR_BUS]id_csr_waddr,
    output  reg [`REG_ADDR_BUS] exe_csr_waddr,
	
	//异常处理flush冲刷信号
    input	wire 					excep_flush,

    //skip指令相关
    input   wire                    id2diff_skip,
    output  reg                     exe2diff_skip
    
);

    always@(posedge cpu_clk_50M) begin
        //复位的时候将送至执行阶段的信息清0
        if(cpu_rst_n == `RST_ENABLE) begin
            exe_alutype <=      `NOP;
            exe_aluop   <=      `RISCV64_SLL;
            exe_src1    <=      `ZERO_WORD;
            exe_src2    <=      `ZERO_WORD;
            exe_wa      <=      `REG_NOP;
            exe_wreg    <=      `WRITE_DISABLE;
            exe_mreg    <=      `WRITE_DISABLE;
            exe_din     <=      `ZERO_WORD;
            exe_ret_addr<=      `ZERO_WORD;
			exe_pc		<=		`PC_INIT;
			exe_inst	<=		`ZERO_INST;  
            exe_csr_waddr<=     3'b0;
            exe2diff_skip<=     `FALSE_V;
        end
        else  if(excep_flush == `TRUE_V)    begin
            exe_alutype <=      `NOP;
            exe_aluop   <=      `RISCV64_SLL;
            exe_src1    <=      `ZERO_WORD;
            exe_src2    <=      `ZERO_WORD;
            exe_wa      <=      `REG_NOP;
            exe_wreg    <=      `WRITE_DISABLE;
            exe_mreg    <=      `WRITE_DISABLE;
            exe_din     <=      `ZERO_WORD;
            exe_ret_addr<=      `ZERO_WORD;
			exe_pc		<=		`PC_INIT;
			exe_inst	<=		`ZERO_INST;  
            exe_csr_waddr<=     3'b0;
            exe2diff_skip<=     `FALSE_V;
        end
        else  if(id_stall == `STOP && data_read_stall == `NOSTOP)    begin
            exe_alutype <=      `NOP;
            exe_aluop   <=      `RISCV64_SLL;
            exe_src1    <=      `ZERO_WORD;
            exe_src2    <=      `ZERO_WORD;
            exe_wa      <=      `REG_NOP;
            exe_wreg    <=      `WRITE_DISABLE;
            exe_mreg    <=      `WRITE_DISABLE;
            exe_din     <=      `ZERO_WORD;
            exe_ret_addr<=      `ZERO_WORD;
			exe_pc		<=		`PC_INIT;
			exe_inst	<=		`ZERO_INST;  
            exe_csr_waddr<=     3'b0;
            exe2diff_skip<=     `FALSE_V;
        end
        else if(data_read_stall == `NOSTOP)  begin
            exe_alutype <=      id_alutype;
            exe_aluop   <=      id_aluop;
            exe_src1    <=      id_src1;
            exe_src2    <=      id_src2;
            exe_wa      <=      id_wa;
            exe_wreg    <=      id_wreg;
            exe_mreg    <=      id_mreg;
            exe_din     <=      id_din;
            exe_ret_addr<=      id_ret_addr;
			exe_pc		<=		id_pc;
			exe_inst	<=		id_inst;
            exe_csr_waddr<=     id_csr_waddr;
            exe2diff_skip<=     id2diff_skip;
        end
    end
endmodule
    