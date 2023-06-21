
`include "defines.v"

module	memwb_reg(
	input		wire				cpu_clk_50M,
	input		wire				cpu_rst_n,
	
	//来自访存阶段的信息
	input 		wire[`REG_ADDR_BUS]	mem_wa,
	input		wire				mem_wreg,
	input		wire[`REG_BUS]		mem_dreg,
	input		wire 				mem_mreg,
	input		wire[`BSEL_BUS]		mem_dre,
	input		wire 				mem_sign,
	input		wire[`WORD_BUS]		mem_dm,
	input		wire[`INST_ADDR_BUS]mem_pc,
	input		wire[`INST_BUS]		mem_inst,
	
	//送至写回阶段的信息
	output		reg[`REG_ADDR_BUS]	wb_wa,
	output		reg					wb_wreg,
	output		reg[`REG_BUS]		wb_dreg,
	output		reg 				wb_mreg,
	output		reg[`BSEL_BUS]		wb_dre,
	output		reg 				wb_sign,
	output		reg[`WORD_BUS]		wb_dm,
	output		reg[`INST_ADDR_BUS]	wb_pc,
	output		reg[`INST_BUS]		wb_inst,

	//访存产生的暂停信号
	input	wire 					data_read_stall,

	//csr指令添加
	input	wire 					mem_csr_we,
	input	wire[`CSR_ADDR_BUS]		mem_csr_waddr,
	input	wire[`REG_BUS	 ]		mem_csr_wdata,

	output	reg 					wb_csr_we,
	output	reg[`CSR_ADDR_BUS]		wb_csr_waddr,
	output	reg[`REG_BUS	 ]		wb_csr_wdata,

	// excep_flush
	input	wire					excep_flush,

	//to diff 异常指令编号
	input	wire[`EXCEP_CODE_BUS]	mem_excep_code,
	output	reg[`EXCEP_CODE_BUS]	wb_excep_code,

	//skip信号相关
	input	wire 					mem2diff_skip,
	output	reg 					wb2diff_skip
	
);
	
	always@(posedge cpu_clk_50M) begin
		//复位的时候将送至写回阶段的信息清0
		if(cpu_rst_n == `RST_ENABLE) begin
			wb_wa		<=			`REG_NOP;
			wb_wreg		<=			`WRITE_DISABLE;
			wb_dreg		<=			`ZERO_WORD;
			wb_mreg		<=			`WRITE_DISABLE;
			wb_dre		<=			`ZERO_WORD;
			wb_sign		<=			`FALSE_V;
			wb_dm		<=			`ZERO_WORD;
			wb_pc		<=			`PC_INIT;
			wb_inst		<=			`ZERO_INST;
			wb_csr_we	<=			`FALSE_V;
			wb_csr_waddr<=			3'b0;
			wb_csr_wdata<=			`ZERO_WORD;
			wb_excep_code<=			3'b0;
		end
		else if(excep_flush == `TRUE_V)	begin
			wb_wa		<=			`REG_NOP;
			wb_wreg		<=			`WRITE_DISABLE;
			wb_dreg		<=			`ZERO_WORD;
			wb_mreg		<=			`WRITE_DISABLE;
			wb_dre		<=			`ZERO_WORD;
			wb_sign		<=			`FALSE_V;
			wb_dm		<=			`ZERO_WORD;
			wb_pc		<=			mem_pc;
			wb_inst		<=			mem_inst;
			wb_csr_we	<=			mem_csr_we;
			wb_csr_waddr<=			mem_csr_waddr;
			wb_csr_wdata<=			mem_csr_wdata;
			wb_excep_code<=			mem_excep_code;
		end
		else if(data_read_stall == `STOP) begin
			wb_wa		<=			`REG_NOP;
			wb_wreg		<=			`WRITE_DISABLE;
			wb_dreg		<=			`ZERO_WORD;
			wb_mreg		<=			`WRITE_DISABLE;
			wb_dre		<=			`ZERO_WORD;
			wb_sign		<=			`FALSE_V;
			wb_dm		<=			`ZERO_WORD;
			wb_pc		<=			`PC_INIT;
			wb_inst		<=			`ZERO_INST;
			wb_csr_we	<=			`FALSE_V;
			wb_csr_waddr<=			3'b0;
			wb_csr_wdata<=			`ZERO_WORD;
			wb_excep_code<=			3'b0;
		end
		//将来自访存阶段的信息寄存并送至写回阶段
		else if(data_read_stall == `NOSTOP) begin
			wb_wa		<=			mem_wa;
			wb_wreg		<=			mem_wreg;
			wb_dreg		<=			mem_dreg;
			wb_mreg		<=			mem_mreg;
			wb_dre		<=			mem_dre;
			wb_sign		<=			mem_sign;
			wb_dm		<=			mem_dm;
			wb_pc		<=			mem_pc;
			wb_inst		<=			mem_inst;
			wb_csr_we	<=			mem_csr_we;
			wb_csr_waddr<=			mem_csr_waddr;
			wb_csr_wdata<=			mem_csr_wdata;
			wb_excep_code<=			mem_excep_code;
			wb2diff_skip <=			mem2diff_skip;
		end
	end
	
endmodule