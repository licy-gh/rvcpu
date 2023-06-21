
`include	"defines.v"

module exemem_reg(
	input	wire					cpu_clk_50M,
	input	wire					cpu_rst_n,
	
	//来自执行阶段的信息
	input	wire[`ALUOP_BUS]		exe_aluop,
	input	wire[`REG_ADDR_BUS]		exe_wa,
	input	wire					exe_wreg,
	input	wire[`REG_BUS]			exe_wd,
	input	wire 					exe_mreg,
	input	wire[`REG_BUS]			exe_din,
	input	wire[`INST_ADDR_BUS]	exe_pc,
	input	wire[`INST_BUS]			exe_inst,
	
	
	//送到访存阶段的信息
	output	reg[`ALUOP_BUS]			mem_aluop,
	output	reg[`REG_ADDR_BUS]		mem_wa,
	output	reg						mem_wreg, 
	output	reg[`REG_BUS]			mem_wd,
	output	reg 					mem_mreg,
	output	reg[`REG_BUS]			mem_din,
	output	reg[`INST_ADDR_BUS]		mem_pc,
	output	reg[`INST_BUS]			mem_inst,

	//访存产生的暂停信号
	input	wire					data_read_stall,

	//csr指令添加信号
	input	wire 					exe_csr_we,
	input	wire[`CSR_ADDR_BUS]		exe_csr_waddr,
	input	wire[`REG_BUS	  ]		exe_csr_wdata,

	output	reg 					mem_csr_we,
	output	reg[`CSR_ADDR_BUS]		mem_csr_waddr,
	output	reg[`REG_BUS	 ]		mem_csr_wdata,
	
	//异常处理flush冲刷信号
	input	wire[`EXCEP_CODE_BUS]	exe_excep_code,
	output	reg[`EXCEP_CODE_BUS]	mem_excep_code,
    input	wire 					excep_flush,

	input	wire 					exe_excep_flush,
	input	wire[`INST_ADDR_BUS]	exe_excep_flush_pc,

	output	wire					mem_excep_flush,
	output	wire[`INST_ADDR_BUS]	mem_excep_flush_pc,

	//skip信号相关
	input	wire					exe2diff_skip,
	output	reg 					mem2diff_skip
	
);
	
    always@(posedge cpu_clk_50M) begin
		if(cpu_rst_n == `RST_ENABLE) begin
		    mem_aluop   <=      `RISCV64_SLL;
			mem_wa		<=		`REG_NOP;
			mem_wreg	<=		`WRITE_DISABLE;
			mem_wd		<=		`ZERO_WORD;
			mem_mreg	<=		`WRITE_DISABLE;
			mem_din		<=		`ZERO_WORD;
			mem_pc		<=		`PC_INIT;
			mem_inst	<=		`ZERO_INST;
			mem_csr_we	<=		`FALSE_V;
			mem_csr_waddr	<=	3'b0;
			mem_csr_wdata	<=	`ZERO_WORD;
			mem_excep_code	<=	3'b0;
			mem2diff_skip	<=	`FALSE_V;
			mem_excep_flush	<=	`FALSE_V;
			mem_excep_flush_pc<=`ZERO_WORD;
		end
		if(excep_flush == `TRUE_V) begin
		    mem_aluop   <=      `RISCV64_SLL;
			mem_wa		<=		`REG_NOP;
			mem_wreg	<=		`WRITE_DISABLE;
			mem_wd		<=		`ZERO_WORD;
			mem_mreg	<=		`WRITE_DISABLE;
			mem_din		<=		`ZERO_WORD;
			mem_pc		<=		`PC_INIT;
			mem_inst	<=		`ZERO_INST;
			mem_csr_we	<=		`FALSE_V;
			mem_csr_waddr	<=	3'b0;
			mem_csr_wdata	<=	`ZERO_WORD;
			mem_excep_code	<=	3'b0;
			mem2diff_skip	<=	`FALSE_V;
			mem_excep_flush	<=	`FALSE_V;
			mem_excep_flush_pc<=`ZERO_WORD;
		end
		else if(data_read_stall == `NOSTOP) begin
			mem_aluop	<=		exe_aluop;
			mem_wa		<=		exe_wa;
			mem_wreg	<=		exe_wreg;
			mem_wd		<=		exe_wd;
			mem_mreg	<=		exe_mreg;
			mem_din		<=		exe_din;
			mem_pc		<=		exe_pc;
			mem_inst	<=		exe_inst;
			mem_csr_we	<=		exe_csr_we;
			mem_csr_waddr<=		exe_csr_waddr;
			mem_csr_wdata<=		exe_csr_wdata;
			mem_excep_code	<=	exe_excep_code;
			mem2diff_skip	<=	exe2diff_skip;
			mem_excep_flush	<=	exe_excep_flush;
			mem_excep_flush_pc<=exe_excep_flush_pc;
		end
	end

endmodule