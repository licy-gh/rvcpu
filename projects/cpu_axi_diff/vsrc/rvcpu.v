
`include "defines.v"
 
 module rvcpu(
	input		wire					cpu_clk_50M,
	input 		wire					cpu_rst_n,
	

    //传入axi的取指信号
    output      wire                    if_burst_valid,
    output      wire[`INST_ADDR_BUS]    if_burst_addr,
    output      wire[ 1: 0]             if_burst_len,
    output      wire[ 1: 0]             if_burst_size,

    input       wire                    if_burst_ready,
    input       wire[`WORD_BUS]         if_burst_data,

    input       wire                    if_resp,

    //传入axi的数据读信号
    output      wire                    mem_burst_valid,
    output      wire[`WORD_BUS]         mem_burst_addr,
    output      wire[ 1: 0]             mem_burst_len,
    output      wire[ 1: 0]             mem_burst_size,
    
    input       wire                    mem_burst_ready,
    input       wire[`WORD_BUS]         mem_burst_data,

    input  [ 1: 0]                      d_resp,

    //传入axi的数据写信号
    output      wire                    d_wvalid,
    output      wire[ 7: 0]             d_wmask,
    output      wire[`WORD_BUS]         d_waddr,
    output      wire[`WORD_BUS]         d_wdata,
    input       wire                    axi_w_isdone,

    //clint模块传出的时钟中断信号
    input       wire                    clock_interr,
    output      wire                    interr_isdone,

    //传给difftest的寄存器写信号
    output      wire                    wb_wreg_o,
    output      wire[`REG_ADDR_BUS]     wb_wa_o,
    output      wire[`WORD_BUS]         wb_wd_o,

    //传给difftes的当前pc值与指令值
    output      wire[`INST_ADDR_BUS]    wb_pc_o,
    output      wire[`INST_BUS]         wb_inst_o,
    output      wire[`EXCEP_CODE_BUS]   wb_excep_code_o,
    output      wire                    wb2diff_skip_o,

    output      wire[`REG_BUS]  regs_o[0:`REG_NUM-1],
    output      wire[`REG_BUS]  csrs_o[0:`CSR_NUM],

    //串口输出信号
    output      wire                    uart_ena,
    output      wire[ 7: 0]             uart_data_out
	

);

    //连接取指阶段到取指译码寄存器阶段的信号
    wire[`INST_ADDR_BUS]        pc;
    wire[`INST_BUS]             if_inst_o;

    wire                        handshake_done; //取指完成信号
    //连接取指阶段与ram模块的取指信号
    //wire                        ice;
    //wire[`INST_ADDR_BUS]        iaddr;
    //wire[`INST_BUS]             inst;

    //连接取指译码寄存器到译码阶段的信号
    wire[`INST_ADDR_BUS]        id_pc_i;
    wire[`INST_BUS]             id_inst_i;

    //连接译码阶段到寄存器堆的信号
    wire[`REG_ADDR_BUS]         ra1;
    wire[`REG_ADDR_BUS]         ra2;
    wire[`REG_BUS]              rd1;
    wire[`REG_BUS]              rd2;

    //转移指令添加
    wire [`INST_ADDR_BUS ] 	if_pc_plus_4;
    wire [`INST_ADDR_BUS ] 	id_pc_plus_4;
    wire [`REG_BUS       ] 	id_ret_addr;
    wire [`REG_BUS       ] 	exe_ret_addr;

    //连接译码阶段到译码执行寄存器的信号
    wire[`ALUTYPE_BUS]          id_alutype_o;
    wire[`ALUOP_BUS]            id_aluop_o;
    wire[`REG_ADDR_BUS]         id_wa_o;
    wire                        id_wreg_o;
    wire[`REG_BUS]              id_src1_o;
    wire[`REG_BUS]              id_src2_o;
    wire                        id_mreg_o;
    wire[`REG_BUS]              id_din_o;
    wire[`INST_ADDR_BUS]        id_pc_o;
    wire[`INST_BUS]             id_inst_o;

    //连接译码执行寄存器到执行阶段的信号
    wire[`ALUTYPE_BUS]          exe_alutype_i;
    wire[`ALUOP_BUS]            exe_aluop_i;
    wire[`REG_ADDR_BUS]         exe_wa_i;
    wire                        exe_wreg_i;
    wire[`REG_BUS]              exe_src1_i;
    wire[`REG_BUS]              exe_src2_i;
    wire                        exe_mreg_i;
    wire[`REG_BUS]              exe_din_i;
    wire[`INST_ADDR_BUS]        exe_pc_i;
    wire[`INST_BUS]             exe_inst_i;

    //连接执行阶段到执行访存寄存器的信号
    wire[`ALUOP_BUS]            exe_aluop_o; 
    wire[`REG_ADDR_BUS]         exe_wa_o;
    wire                        exe_wreg_o;
    wire[`REG_BUS]              exe_wd_o;
    wire                        exe_mreg_o;
    wire[`REG_BUS]              exe_din_o;
    wire[`INST_ADDR_BUS]        exe_pc_o;
    wire[`INST_BUS]             exe_inst_o;

    //连接执行访存寄存器到访存阶段的信号
    wire[`ALUOP_BUS]            mem_aluop_i;
    wire[`REG_ADDR_BUS]         mem_wa_i;
    wire                        mem_wreg_i;
    wire[`REG_BUS]              mem_wd_i;
    wire                        mem_mreg_i;
    wire[`REG_BUS]              mem_din_i;
    wire[`INST_ADDR_BUS]        mem_pc_i;
    wire[`INST_BUS]             mem_inst_i;

    //连接访存阶段到访存写会寄存器的信号
    wire[`REG_ADDR_BUS]         mem_wa_o;
    wire                        mem_wreg_o;
    wire[`REG_BUS]              mem_dreg_o;
    wire                        mem_mreg_o;
    wire[`BSEL_BUS]             mem_dre_o;
    wire                        mem_sign_o;
    wire[`WORD_BUS]             mem_dm_o;
    wire[`INST_ADDR_BUS]        mem_pc_o;
    wire[`INST_BUS]             mem_inst_o;


    //连接访存写会寄存器与写会阶段的信号
    wire[`REG_ADDR_BUS]         wb_wa_i;
    wire                        wb_wreg_i;
    wire[`REG_BUS]              wb_dreg_i;
    wire                        wb_mreg_i;
    wire[`BSEL_BUS]             wb_dre_i;
    wire                        wb_sign_i;
    wire[`WORD_BUS]             wb_dm_i;
    wire[`INST_ADDR_BUS]        wb_pc_i;
    wire[`INST_BUS]             wb_inst_i;
    wire[`EXCEP_CODE_BUS]       wb_excep_code_i;
    
    /************************流水线暂停信号**********************/

    wire                        id_stall;
    wire                        data_read_stall;

    /************************流水线暂停信号**********************/

    /************************分支预测**********************/
	//btb到取指阶段的信号
	wire                   btb_jump_ena;
	wire[`INST_ADDR_BUS]   btb_prepc_o;
	
	//取指到取指/译码寄存器的信号
	wire                   if_jump_ena;
	wire[`INST_ADDR_BUS]   if_jump_pc;
	
	//取指/译码寄存器到译码阶段的信号
	wire                   id_jump_ena;
	wire[`INST_ADDR_BUS]   id_jump_pc;
	
	//译码阶段到btb的更新信号
	wire                   btb_update_req;
	wire                   btb_update_state;
	wire[`PC_ADDR_BTB  ]   btb_update_pc;
	wire[`INST_ADDR_BUS]   btb_update_prepc;
	
	//译码阶段传出的流水线冲刷信号
	wire                   flush;
	wire[`INST_ADDR_BUS]   flush_addr;

	/************************分支预测*********************/


    /************************csr指令添加*********************/

    wire[`CSR_ADDR_BUS]     id_csr_waddr_o;
    wire[`CSR_ADDR_BUS]     exe_csr_waddr_i;

    wire                    exe_csr_we_o;
    wire[`CSR_ADDR_BUS]     exe_csr_waddr_o;
    wire[`REG_BUS     ]     exe_csr_wdata_o;

    wire                    mem_csr_we_i;
    wire[`CSR_ADDR_BUS]     mem_csr_waddr_i;
    wire[`REG_BUS     ]     mem_csr_wdata_i;

    wire 				    mem_csr_we_o;
	wire[`CSR_ADDR_BUS]	    mem_csr_waddr_o;
	wire[`REG_BUS	  ]	    mem_csr_wdata_o;

    wire                    wb_csr_we_i;
    wire[`CSR_ADDR_BUS]     wb_csr_waddr_i;
    wire[`REG_BUS     ]     wb_csr_wdata_i;

    wire[`CSR_ADDR_BUS]     csr_raddr;
    wire[`REG_BUS     ]     csr_rdata;
    wire                    csr_we;
    wire[`CSR_ADDR_BUS]     csr_waddr;
    wire[`REG_BUS     ]     csr_wdata;


    /************************csr指令添加*********************/

    /************************异常指令添加*********************/

    wire                    global_trap_ena;
    wire                    time_trap_ena;

    wire[`REG_BUS     ]     ecall_mtvec_read;
    wire[`REG_BUS     ]     mret_mepc_read;

    wire                    excep_flush;
    wire[`INST_ADDR_BUS]    excep_flush_pc;
    wire[`EXCEP_CODE_BUS]   excep_code;

    wire                    exe_excep_flush_o;
    wire[`INST_ADDR_BUS]    exe_excep_flush_pc_o;
    wire[`EXCEP_CODE_BUS]   exe_excep_code_o;

    wire                    mem_excep_flush_i;
    wire[`INST_ADDR_BUS]    mem_excep_flush_pc_i;

    wire[`EXCEP_CODE_BUS]   mem_excep_code_i;
    wire[`EXCEP_CODE_BUS]   mem_excep_code_o;

    /************************异常指令添加*********************/

    /************************icache添加*********************/

    wire                    icache_isused;

    wire                    if2cache_ena;
    wire[`INST_ADDR_BUS]    if2cache_addr;

    wire                    if_read_isdone;
    wire[`INST_BUS]         if_data_read;


    /************************icache添加*********************/

    /************************dcache添加*********************/

    wire                    dcache_isused;

    wire                    mem2cache_ena;
    wire[`INST_ADDR_BUS]    mem2cache_addr;
    wire[ 1: 0]             mem2cache_rsize;

    wire                    mem_read_isdone;
    wire[`WORD_BUS]         mem_data_read;

    wire                    cpu_write_ena;
    wire[`WORD_BUS]         cpu_write_addr;
    wire[`WORD_BUS]         cpu_write_data;
    wire[ 7: 0]             cpu_write_mask;

    wire                    axi_w_isbusy;


    /************************dcache添加*********************/

    //difftest的skip信号相关
    wire                    id2diff_skip_o;
    wire                    exe2diff_skip_i;
    wire                    exe2diff_skip_o;
    wire                    mem2diff_skip_i;
    wire                    mem2diff_skip_o;
    wire                    wb2diff_skip_i;

    if_stage    if_stage0(
        .cpu_clk_50M(cpu_clk_50M),.cpu_rst_n(cpu_rst_n),
        .pc_plus_4(if_pc_plus_4),.if_pc_o(pc),
        .if_inst_o(if_inst_o),
        //连接axi的取指信号
        .if_valid(if2cache_ena),.if_addr(if2cache_addr),
        .axi_isused(icache_isused),
        //取指完成信号
        .if_read_isdone(if_read_isdone),.if_data_read(if_data_read),
        .handshake_done(handshake_done),
        //流水线暂停信号
        .data_stall(data_read_stall),.id_stall(id_stall),
        //分支预测添加
		.if_pre_jumpena(btb_jump_ena),.if_pre_pc(btb_prepc_o),
		.jump_ena_o(if_jump_ena),.jump_pc_o(if_jump_pc),
		//预测错误回传信号
		.flush(flush),.flush_addr(flush_addr),
        //异常处理冲刷信号添加
        .excep_flush(excep_flush),.excep_flush_pc(excep_flush_pc)
    );

    ifid_reg    ifid_reg0(
        .cpu_clk_50M(cpu_clk_50M),.cpu_rst_n(cpu_rst_n),
        .if_pc(pc),.if_inst(if_inst_o),.if_pc_plus_4(if_pc_plus_4),
        .id_pc(id_pc_i),.id_inst(id_inst_i),.id_pc_plus_4(id_pc_plus_4),
        //取指完成信号
        .handshake_done(handshake_done),
        //流水线暂停信号
        .id_stall(id_stall),.data_read_stall(data_read_stall),
        //分支预测添加
		.flush(flush),
		.if_jump_ena(if_jump_ena),.if_jump_pc(if_jump_pc),
		.id_jump_ena(id_jump_ena),.id_jump_pc(id_jump_pc),
        //异常处理flush冲刷信号添加
        .excep_flush(excep_flush)
    );

    id_stage    id_stage0(
        .cpu_rst_n(cpu_rst_n),
        .id_pc_i(id_pc_i),.id_inst_i(id_inst_i),
        .pc_plus_4(id_pc_plus_4),
        .rd1(rd1),.rd2(rd2),
        .ra1(ra1),.ra2(ra2),
        .id_alutype_o(id_alutype_o),.id_aluop_o(id_aluop_o),
        .id_wreg_o(id_wreg_o),.id_wa_o(id_wa_o),
        .id_pc_o(id_pc_o),.id_inst_o(id_inst_o),
        .id_src1_o(id_src1_o),.id_src2_o(id_src2_o),
        .id_mreg_o(id_mreg_o),.id_din_o(id_din_o),
        //数据定向回传
        .exe2id_wreg(exe_wreg_o),.exe2id_wa(exe_wa_o),.exe2id_wd(exe_wd_o),
        .mem2id_wreg(mem_wreg_o),.mem2id_wa(mem_wa_o),.mem2id_wd(mem_dreg_o),
        //加载相关信号
        .exe2id_mreg(exe_mreg_o),.mem2id_mreg(mem_mreg_o),.id_stall(id_stall),
		//分支预测及错误回传流水线冲刷
		.id_jump_ena_i(id_jump_ena),.id_jump_pc_i(id_jump_pc),
		.btb_update_req(btb_update_req),.btb_update_state(btb_update_state),
		.btb_update_pc(btb_update_pc),.btb_update_prepc(btb_update_prepc),
        .ret_addr(id_ret_addr),
		.flush(flush),.flush_addr(flush_addr),
        //串口输出信号
        .uart_ena(uart_ena),.uart_data_out(uart_data_out),
        .id2diff_skip_o(id2diff_skip_o),
        //csr指令添加
        .csr_raddr(csr_raddr),.csr_rdata(csr_rdata),.id_csr_waddr(id_csr_waddr_o),
        .exe2id_csr_we(exe_csr_we_o),.exe2id_csr_waddr(exe_csr_waddr_o),.exe2id_csr_wdata(exe_csr_wdata_o),
        .mem2id_csr_we(mem_csr_we_o),.mem2id_csr_waddr(mem_csr_waddr_o),.mem2id_csr_wdata(mem_csr_wdata_o)
    );

    idexe_reg   idexe_reg0(
        .cpu_clk_50M(cpu_clk_50M),.cpu_rst_n(cpu_rst_n),
        .id_alutype(id_alutype_o),.id_aluop(id_aluop_o),
        .id_wa(id_wa_o),.id_wreg(id_wreg_o),
        .id_src1(id_src1_o),.id_src2(id_src2_o),
        .id_mreg(id_mreg_o),.id_din(id_din_o),
        .id_pc(id_pc_o),.id_inst(id_inst_o),
        .exe_alutype(exe_alutype_i),.exe_aluop(exe_aluop_i),
        .exe_wa(exe_wa_i),.exe_wreg(exe_wreg_i),
        .exe_src1(exe_src1_i),.exe_src2(exe_src2_i),
        .exe_mreg(exe_mreg_i),.exe_din(exe_din_i),
		.id_ret_addr(id_ret_addr),.exe_ret_addr(exe_ret_addr),
        .exe_pc(exe_pc_i),.exe_inst(exe_inst_i),
        //流水线暂停信号
        .id_stall(id_stall),.data_read_stall(data_read_stall),
        //csr指令添加
        .id_csr_waddr(id_csr_waddr_o),.exe_csr_waddr(exe_csr_waddr_i),
        //异常处理flush冲刷信号
        .excep_flush(excep_flush),
        //skip信号相关
        .id2diff_skip(id2diff_skip_o),.exe2diff_skip(exe2diff_skip_i)
    );

    exe_stage   exe_stage0(
        .exe_alutype_i(exe_alutype_i),.exe_aluop_i(exe_aluop_i),
        .exe_wa_i(exe_wa_i),.exe_wreg_i(exe_wreg_i),
        .exe_src1_i(exe_src1_i),.exe_src2_i(exe_src2_i),
        .exe_mreg_i(exe_mreg_i),.exe_din_i(exe_din_i),
        .ret_addr(exe_ret_addr),
        .exe_pc_i(exe_pc_i),.exe_inst_i(exe_inst_i),
        .exe_aluop_o(exe_aluop_o),
        .exe_wa_o(exe_wa_o),.exe_wreg_o(exe_wreg_o),.exe_wd_o(exe_wd_o),
        .exe_mreg_o(exe_mreg_o),.exe_din_o(exe_din_o),
        .exe_pc_o(exe_pc_o),.exe_inst_o(exe_inst_o),
        //csr指令添加
        .exe_csr_waddr_i(exe_csr_waddr_i),
        .exe_csr_waddr(exe_csr_waddr_o),.exe_csr_wdata(exe_csr_wdata_o),.exe_csr_we(exe_csr_we_o),
        //异常指令添加
        .global_trap_ena(global_trap_ena),.time_trap_ena(time_trap_ena),
        .mtvec_read(ecall_mtvec_read),.mepc_read(mret_mepc_read),
        .excep_flush(exe_excep_flush_o),.excep_flush_pc(exe_excep_flush_pc_o),.excep_code(exe_excep_code_o),
        //clint模块传出的时钟中断信号
        .clock_interr(clock_interr),.interr_isdone(interr_isdone),
        //skip信号相关
        .exe2diff_skip_i(exe2diff_skip_i),.exe2diff_skip_o(exe2diff_skip_o)        
    );

    exemem_reg  exemem_reg0(
        .cpu_clk_50M(cpu_clk_50M),.cpu_rst_n(cpu_rst_n),
        .exe_aluop(exe_aluop_o),
        .exe_wa(exe_wa_o),.exe_wreg(exe_wreg_o),.exe_wd(exe_wd_o),
        .exe_mreg(exe_mreg_o),.exe_din(exe_din_o),
        .exe_pc(exe_pc_o),.exe_inst(exe_inst_o),
        .mem_aluop(mem_aluop_i),
        .mem_wa(mem_wa_i),.mem_wreg(mem_wreg_i),.mem_wd(mem_wd_i),
        .mem_mreg(mem_mreg_i),.mem_din(mem_din_i),
        .mem_pc(mem_pc_i),.mem_inst(mem_inst_i),
        //流水线暂停信号
        .data_read_stall(data_read_stall),
        //csr指令信号
        .exe_csr_we(exe_csr_we_o),.exe_csr_waddr(exe_csr_waddr_o),.exe_csr_wdata(exe_csr_wdata_o),
        .mem_csr_we(mem_csr_we_i),.mem_csr_waddr(mem_csr_waddr_i),.mem_csr_wdata(mem_csr_wdata_i),
        //异常处理flush冲刷信号
        .excep_flush(excep_flush),
        .exe_excep_flush(exe_excep_flush_o),.exe_excep_flush_pc(exe_excep_flush_pc_o),
        .mem_excep_flush(mem_excep_flush_i),.mem_excep_flush_pc(mem_excep_flush_pc_i),
        .exe_excep_code(exe_excep_code_o),.mem_excep_code(mem_excep_code_i),
        //skip信号相关
        .exe2diff_skip(exe2diff_skip_o),.mem2diff_skip(mem2diff_skip_i)
    );

    mem_stage   mem_stage0(
        .mem_aluop_i(mem_aluop_i),
        .mem_wa_i(mem_wa_i),.mem_wreg_i(mem_wreg_i),.mem_wd_i(mem_wd_i),
        .mem_mreg_i(mem_mreg_i),.mem_din_i(mem_din_i),
        .mem_pc_i(mem_pc_i),.mem_inst_i(mem_inst_i),
        .mem_wa_o(mem_wa_o),.mem_wreg_o(mem_wreg_o),.mem_dreg_o(mem_dreg_o),
        .mem_mreg_o(mem_mreg_o),.dre(mem_dre_o),.sign(mem_sign_o),.mem_dm_o(mem_dm_o),
        .mem_pc_o(mem_pc_o),.mem_inst_o(mem_inst_o),
        //axi-data-read
        .rvalid(mem2cache_ena),.r_ready(mem_read_isdone),.data_read(mem_data_read),
        .raddr(mem2cache_addr),.rsize(mem2cache_rsize),.resp(d_resp),
        //axi-data-write
        .wvalid(cpu_write_ena),.waddr(cpu_write_addr),.wdata(cpu_write_data),.wmask(cpu_write_mask),
        .axi_w_isbusy(axi_w_isbusy),
        //stall
        .d_stall(data_read_stall),
        //csr指令添加
        .mem_csr_we_i(mem_csr_we_i),.mem_csr_waddr_i(mem_csr_waddr_i),.mem_csr_wdata_i(mem_csr_wdata_i),
        .mem_csr_we_o(mem_csr_we_o),.mem_csr_waddr_o(mem_csr_waddr_o),.mem_csr_wdata_o(mem_csr_wdata_o),
        //excep 
        .mem_excep_flush_i(mem_excep_flush_i),.mem_excep_flush_pc_i(mem_excep_flush_pc_i),
        .mem_excep_flush_o(excep_flush),.mem_excep_flush_pc_o(excep_flush_pc),
        .mem_excep_code_i(mem_excep_code_i),.mem_excep_code_o(excep_code),
        .mem2diff_skip_i(mem2diff_skip_i),.mem2diff_skip_o(mem2diff_skip_o)
    );

    memwb_reg   memwb_reg0(
        .cpu_clk_50M(cpu_clk_50M),.cpu_rst_n(cpu_rst_n),
        .mem_wa(mem_wa_o),.mem_wreg(mem_wreg_o),.mem_dreg(mem_dreg_o),
        .mem_mreg(mem_mreg_o),.mem_dre(mem_dre_o),.mem_sign(mem_sign_o),
        .mem_dm(mem_dm_o),
        .mem_pc(mem_pc_o),.mem_inst(mem_inst_o),
        .wb_wa(wb_wa_i),.wb_wreg(wb_wreg_i),.wb_dreg(wb_dreg_i),
        .wb_mreg(wb_mreg_i),.wb_dre(wb_dre_i),.wb_sign(wb_sign_i),
        .wb_dm(wb_dm_i),
        .wb_pc(wb_pc_i),.wb_inst(wb_inst_i),
        //流水线暂停信号
        .data_read_stall(data_read_stall),
        //csr指令添加信号
        .mem_csr_we(mem_csr_we_o),.mem_csr_waddr(mem_csr_waddr_o),.mem_csr_wdata(mem_csr_wdata_o),
        .wb_csr_we(wb_csr_we_i),.wb_csr_waddr(wb_csr_waddr_i),.wb_csr_wdata(wb_csr_wdata_i),
        //to diff 异常指令编号
        .excep_flush(excep_flush),
        .mem_excep_code(excep_code),.wb_excep_code(wb_excep_code_i),
        //skip信号相关
        .mem2diff_skip(mem2diff_skip_o),.wb2diff_skip(wb2diff_skip_i)
    );

    wb_stage    wb_stage0(
        .wb_wa_i(wb_wa_i),.wb_wreg_i(wb_wreg_i),.wb_dreg_i(wb_dreg_i),
        .wb_mreg_i(wb_mreg_i),.wb_dre_i(wb_dre_i),.sign(wb_sign_i),
        .wb_pc_i(wb_pc_i),.wb_inst_i(wb_inst_i),
        .dm(wb_dm_i),
        .wb_wa_o(wb_wa_o),.wb_wreg_o(wb_wreg_o),.wb_wd_o(wb_wd_o),
        // to diff
        .wb_pc_o(wb_pc_o),.wb_inst_o(wb_inst_o),
        .wb_excep_code_i(wb_excep_code_i),.wb_excep_code_o(wb_excep_code_o),
        .wb2diff_skip_i(wb2diff_skip_i),.wb2diff_skip_o(wb2diff_skip_o),
        //csr指令添加信号
        .wb_csr_we_i(wb_csr_we_i),.wb_csr_waddr_i(wb_csr_waddr_i),.wb_csr_wdata_i(wb_csr_wdata_i),
        .csr_we(csr_we),.csr_waddr(csr_waddr),.csr_wdata(csr_wdata)
    );

    regfile     regfile0(
        .cpu_clk_50M(cpu_clk_50M),.cpu_rst_n(cpu_rst_n),
        .ra1(ra1),.ra2(ra2),
        .rd1(rd1),.rd2(rd2),
        .wa(wb_wa_o),.we(wb_wreg_o),.wd(wb_wd_o),
        // to diff
        .regs_o(regs_o)
    );

    btb btb0(
	    .cpu_rst_n(cpu_rst_n),.cpu_clk_50M(cpu_clk_50M),
	    .pc_tag(pc[11:2]),
	    .btb_jump_ena(btb_jump_ena),.btb_prepc_o(btb_prepc_o),
	    .wr_req(btb_update_req),.wr_jump_state(btb_update_state),
	    .wr_pc_tag(btb_update_pc),.wr_predicted_pc(btb_update_prepc)
	           
	);

    csrfile     csrfile0(
        .cpu_clk_50M(cpu_clk_50M),.cpu_rst_n(cpu_rst_n),
        .csr_raddr(csr_raddr),.csr_rdata(csr_rdata),
        .csr_we(csr_we),.csr_waddr(csr_waddr),.csr_wdata(csr_wdata),
        //异常处理指令添加
        .global_trap_ena(global_trap_ena),.time_trap_ena(time_trap_ena),
        .excep_mtvec(ecall_mtvec_read),.excep_mret_mepc(mret_mepc_read),
        .excep_code(excep_code),
        // to diff
        .csrs_o(csrs_o)
    );

    icache      icache0(
        .cpu_clk_50M(cpu_clk_50M),.cpu_rst_n(cpu_rst_n),
        .cpu_if_ena(if2cache_ena),.cpu_if_pc(if2cache_addr),
        .axi_isused(icache_isused),
        .read_isdone(if_read_isdone),.icache_data_read(if_data_read),
        .if_burst_valid(if_burst_valid),.if_burst_addr(if_burst_addr),.if_burst_len(if_burst_len),.if_burst_size(if_burst_size),
        .if_burst_ready(if_burst_ready),.if_burst_data(if_burst_data),

        .flush(flush)
    );

    dcache      dcache0(
        .cpu_clk_50M(cpu_clk_50M),.cpu_rst_n(cpu_rst_n),
        .cpu_dataread_ena(mem2cache_ena),.cpu_dataread_addr(mem2cache_addr),.cpu_dataread_size(mem2cache_rsize),
        .axi_isused(dcache_isused),
        .mem_read_isdone(mem_read_isdone),.dcache_data_read(mem_data_read),
        .cpu_write_ena(cpu_write_ena),.cpu_write_addr(cpu_write_addr),.cpu_write_data(cpu_write_data),.cpu_write_mask(cpu_write_mask),
        .mem_w_valid(d_wvalid),.mem_w_addr(d_waddr),.mem_w_data(d_wdata),.mem_w_mask(d_wmask),
        .axi_w_isdone(axi_w_isdone),.axi_w_isbusy(axi_w_isbusy),
        .mem_burst_valid(mem_burst_valid),.mem_burst_addr(mem_burst_addr),.mem_burst_len(mem_burst_len),.mem_burst_size(mem_burst_size),
        .mem_burst_ready(mem_burst_ready),.mem_burst_data(mem_burst_data)
    );

endmodule