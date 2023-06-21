
`include "defines.v"

module mem_stage(

	
	//	从执行阶段获得的信息
	input	wire[`ALUOP_BUS]	mem_aluop_i,
	input	wire[`REG_ADDR_BUS]	mem_wa_i,
	input	wire				mem_wreg_i,
	input	wire[`WORD_BUS]		mem_wd_i,
	input	wire 				mem_mreg_i,
	input	wire[`WORD_BUS]		mem_din_i,
	input	wire[`INST_ADDR_BUS]mem_pc_i,
	input	wire[`INST_BUS]		mem_inst_i,
	
	//	送至写回阶段的信息
	output	wire[`REG_ADDR_BUS]	mem_wa_o,
	output	wire				mem_wreg_o,
	output	wire[`REG_BUS]		mem_dreg_o,
	output	wire				mem_mreg_o,
	output	wire[`BSEL_BUS]		dre,
	output	wire[`INST_ADDR_BUS]mem_pc_o,
	output	wire[`INST_BUS]		mem_inst_o,
	output	wire[`WORD_BUS]		mem_dm_o,

	//	送至数据存储器的读信号
	output	wire 				rvalid,
	input						r_ready,
	input	wire[`WORD_BUS]		data_read,
	output	wire[`WORD_BUS]		raddr,
	output	wire[ 1: 0]			rsize,
	input	wire[ 1: 0]			resp,
	
	//	送至数据存储器的写信号
	output	wire[`WORD_BUS]		waddr,
	output	wire[`WORD_BUS]		wdata,
	output	wire[ 7: 0]			wmask,
	output	wire				wvalid,
	input	wire 				axi_w_isbusy,
	
	//	访问数据存储器引起的流水线暂停信号
	output	wire 				d_stall,

	output	wire 				sign,

	//	excep
	input	wire 				mem_excep_flush_i,
	input	wire[`INST_ADDR_BUS]mem_excep_flush_pc_i,

	output	wire 				mem_excep_flush_o,
	output	wire[`INST_ADDR_BUS]mem_excep_flush_pc_o,

	//	csr指令添加信号
	input	wire 				mem_csr_we_i,
	input	wire[`CSR_ADDR_BUS]	mem_csr_waddr_i,
	input	wire[`REG_BUS	  ]	mem_csr_wdata_i,
	
	output	wire 				mem_csr_we_o,
	output	wire[`CSR_ADDR_BUS]	mem_csr_waddr_o,
	output	wire[`REG_BUS	  ]	mem_csr_wdata_o,

	input	wire[`EXCEP_CODE_BUS]	mem_excep_code_i,
	output	wire[`EXCEP_CODE_BUS]	mem_excep_code_o,

	//	skip信号相关
	input	wire 				mem2diff_skip_i,
	output	reg 				mem2diff_skip_o

);


    //	如果当前指令不是访存指令，则只需将从执行阶段获得的信息直接输出
	assign mem_wa_o    	=   mem_wa_i;
	assign mem_wreg_o  	=   mem_wreg_i;
	assign mem_dreg_o  	=   mem_wd_i;
	assign mem_mreg_o	=	mem_mreg_i;
	assign mem_pc_o		=	mem_pc_i;
	assign mem_inst_o	=   mem_inst_i;

	//skip信号相关
	assign mem2diff_skip_o  = ((d_rce == `TRUE_V || d_wce == `TRUE_V) && (mem_wd_i == `MTIME_ADDR || mem_wd_i == `MTIMECMP_ADDR))	?		`TRUE_V	:	mem2diff_skip_i;

	//	ecall指令时向mepc寄存器写入当前异常pc，否则将执行阶段获得的信息直接输出
	assign mem_csr_we_o   = mem_csr_we_i;		
	assign mem_csr_waddr_o= mem_csr_waddr_i;
	assign mem_csr_wdata_o=	mem_csr_wdata_i;

	assign mem_excep_code_o =mem_excep_code_i;
	assign mem_excep_flush_o=mem_excep_flush_i;
	assign mem_excep_flush_pc_o	= mem_excep_flush_pc_i;
	
	
		
	// 确定当前的访存指令
    wire inst_lb = (mem_aluop_i == 8'h74);
    wire inst_lw = (mem_aluop_i == 8'h71);
    wire inst_sb = (mem_aluop_i == 8'h70);
    wire inst_sw = (mem_aluop_i == 8'h76);
    wire inst_lh = (mem_aluop_i == 8'h72);
    wire inst_lbu= (mem_aluop_i == 8'h75);
    wire inst_lhu= (mem_aluop_i == 8'h73);
    wire inst_sh = (mem_aluop_i == 8'h77);
    wire inst_ld = (mem_aluop_i == 8'h90);  //RISCV64I ONLY
	wire inst_lwu= (mem_aluop_i == 8'h91);	//RISCV64I ONLY
	wire inst_sd = (mem_aluop_i == 8'h92);	//RISCV64I ONLY

	//	有无符号加载存储器
	assign	sign	=	inst_lb | inst_lw | inst_lh;


	/******************************************axi读数据存储器信号***************************************/
	wire	read_handshake_done;
	assign	read_handshake_done = rvalid & r_ready;

	//	获得数据存储器读使能信号
	wire 	d_rce	=	inst_lb | inst_lbu | inst_lw | inst_lwu | inst_lh | inst_lhu | inst_ld;

	assign 	rvalid	=	(mem_excep_flush_o == `TRUE_V)	?	`FALSE_V	:
						(axi_w_isbusy == `TRUE_V)	?	`FALSE_V	:	d_rce;
	assign	wvalid	=	(mem_excep_flush_o == `TRUE_V)	?	`FALSE_V	:
						(axi_w_isbusy == `TRUE_V)	?	`FALSE_V	:	d_wce;

	//	获得数据存储器的访问地址
	assign raddr	=	mem_wd_i;
	assign waddr	=	mem_wd_i;

	//	获得数据存储器的访问类型
	assign rsize	=	(inst_ld)	?	`SIZE_D	:
						(inst_lw | inst_lwu)	?	`SIZE_W	:
						(inst_lh | inst_lhu)	?	`SIZE_H	:
						(inst_lb | inst_lbu)	?	`SIZE_B	:	`SIZE_D	;

	//	获得数据存储器的访问数据
	assign mem_dm_o =	(read_handshake_done == `TRUE_V)	? data_read	:	`ZERO_WORD;

	//	读数据存储器产生的流水线暂停信号
	assign	d_stall	=	((d_wce == `TRUE_V && axi_w_isbusy == `TRUE_V && waddr[27:16] != `CLINT_ADDR) || (d_rce == `TRUE_V && read_handshake_done == `FALSE_V))	?	`STOP	:	`NOSTOP;

	/******************************************axi读数据存储器信号***************************************/

	//	获得数据存储器读字节选择信号
    assign dre[0]	= inst_lb | inst_lbu;
	assign dre[1]	= inst_lh | inst_lhu;
	assign dre[2]	= inst_lw | inst_lwu;
	assign dre[3]	= inst_ld;

    // 	获得数据存储器写使能信号
    wire d_wce   = (inst_sb | inst_sw | inst_sh | inst_sd);

    // 获得数据存储器写字节使能信号
	assign wmask[0] = ((inst_sb & (mem_wd_i[2 : 0] == 3'b000)) | (inst_sw & (mem_wd_i[2 : 0] == 3'b000)) | (inst_sh & (mem_wd_i[2 : 0] == 3'b000)) | inst_sd)	?	1'b1	:	1'b0;
	assign wmask[1] = ((inst_sb & (mem_wd_i[2 : 0] == 3'b001)) | (inst_sw & (mem_wd_i[2 : 0] == 3'b000)) | (inst_sh & (mem_wd_i[2 : 0] == 3'b000)) | inst_sd)	?	1'b1	:	1'b0;
	assign wmask[2] = ((inst_sb & (mem_wd_i[2 : 0] == 3'b010)) | (inst_sw & (mem_wd_i[2 : 0] == 3'b000)) | (inst_sh & (mem_wd_i[2 : 0] == 3'b010)) | inst_sd)	?	1'b1	:	1'b0;
	assign wmask[3] = ((inst_sb & (mem_wd_i[2 : 0] == 3'b011)) | (inst_sw & (mem_wd_i[2 : 0] == 3'b000)) | (inst_sh & (mem_wd_i[2 : 0] == 3'b010)) | inst_sd)	?	1'b1	:	1'b0;
    assign wmask[4] = ((inst_sb & (mem_wd_i[2 : 0] == 3'b100)) | (inst_sw & (mem_wd_i[2 : 0] == 3'b100)) | (inst_sh & (mem_wd_i[2 : 0] == 3'b100)) | inst_sd)	?	1'b1	:	1'b0;
    assign wmask[5] = ((inst_sb & (mem_wd_i[2 : 0] == 3'b101)) | (inst_sw & (mem_wd_i[2 : 0] == 3'b100)) | (inst_sh & (mem_wd_i[2 : 0] == 3'b100)) | inst_sd)	?	1'b1	:	1'b0;
    assign wmask[6] = ((inst_sb & (mem_wd_i[2 : 0] == 3'b110)) | (inst_sw & (mem_wd_i[2 : 0] == 3'b100)) | (inst_sh & (mem_wd_i[2 : 0] == 3'b110)) | inst_sd)	?	1'b1	:	1'b0;
    assign wmask[7] = ((inst_sb & (mem_wd_i[2 : 0] == 3'b111)) | (inst_sw & (mem_wd_i[2 : 0] == 3'b100)) | (inst_sh & (mem_wd_i[2 : 0] == 3'b110)) | inst_sd)	?	1'b1	:	1'b0;
                   
    // 确定待写入数据存储器的数据
    assign  wdata   = (d_wce == `FALSE_V)			?	`ZERO_WORD	:
					  (wmask == 8'b1111_1111      ) ? mem_din_i  :
                      (wmask == 8'b1111_0000       ) ? {mem_din_i[31:0],{32{1'b0}}} :
					  (wmask == 8'b0000_1111       ) ? {{32{1'b0}},mem_din_i[31:0]} :
                      (wmask == 8'b1000_0000       ) ? {mem_din_i[7:0],56'b0} : 
                      (wmask == 8'b0100_0000       ) ? {{8'b0},mem_din_i[7:0],{48'b0}} :
                      (wmask == 8'b0010_0000       ) ? {{16'b0},mem_din_i[7:0],{40'b0}} :
                      (wmask == 8'b0001_0000       ) ? {{24'b0},mem_din_i[7:0],{32'b0}} : 
					  (wmask == 8'b0000_1000       ) ? {{32'b0},mem_din_i[7:0],{24'b0}} : 
                      (wmask == 8'b0000_0100       ) ? {{40'b0},mem_din_i[7:0],{16'b0}} :
                      (wmask == 8'b0000_0010       ) ? {{48'b0},mem_din_i[7:0],{8'b0}} :
                      (wmask == 8'b0000_0001       ) ? {{56'b0},mem_din_i[7:0]} : 
                      (wmask == 8'b1100_0000       ) ? {mem_din_i[15:0], {48{1'b0}}} :
                      (wmask == 8'b0011_0000      ) ? {{16{1'b0}}, mem_din_i[15:0],{32{1'b0}}}  :
					  (wmask == 8'b0000_1100      ) ? {{32{1'b0}}, mem_din_i[15:0],{16{1'b0}}}  :
					  (wmask == 8'b0000_0011       ) ? {{48{1'b0}}, mem_din_i[15:0]}    : `ZERO_WORD;

endmodule