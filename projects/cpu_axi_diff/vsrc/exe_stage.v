
`include "defines.v"

module exe_stage(
    
    //从译码阶段获得的信息
	input		wire[`ALUTYPE_BUS]		exe_alutype_i,
	input		wire[`ALUOP_BUS]		exe_aluop_i,
    input		wire[`REG_BUS]			exe_src1_i,
	input		wire[`REG_BUS]			exe_src2_i,
	input		wire[`REG_ADDR_BUS]		exe_wa_i,
	input		wire					exe_wreg_i,
	input		wire					exe_mreg_i,
	input		wire[`REG_BUS]			exe_din_i,
	input		wire[`INST_ADDR_BUS]	exe_pc_i,
	input		wire[`INST_BUS]			exe_inst_i,

	//转移指令添加
	input       wire[`INST_ADDR_BUS]   ret_addr,
	
	//送至执行阶段的信息
	output		wire[`ALUOP_BUS]		exe_aluop_o,
	output		wire[`REG_ADDR_BUS]		exe_wa_o,
	output		wire					exe_wreg_o,
	output		wire[`REG_BUS]			exe_wd_o,
	output		wire 					exe_mreg_o,
	output		wire[`REG_BUS]			exe_din_o,
	output		wire[`INST_ADDR_BUS]	exe_pc_o,
	output		wire[`INST_BUS]			exe_inst_o,

	//csr指令添加
	input		wire[`CSR_ADDR_BUS]		exe_csr_waddr_i,
	output		wire 					exe_csr_we,
	output		wire[`REG_ADDR_BUS]		exe_csr_waddr,
	output		wire[`REG_BUS]			exe_csr_wdata,

	input	wire[`REG_BUS	  ]	mtvec_read,
	input	wire[`REG_BUS	  ] mepc_read,

	//	异常指令添加信号
	input	wire 				global_trap_ena,
	input	wire 				time_trap_ena,

	output	wire 				  excep_flush,
	output	wire[`EXCEP_CODE_BUS] excep_code,
	output	wire[`INST_ADDR_BUS ] excep_flush_pc,

	//	时钟中断信号
	input	wire 				clock_interr,
	output	wire 				interr_isdone,

	//	skip相关信号
	input	wire 				exe2diff_skip_i,
	output	wire 				exe2diff_skip_o
	
);

    //直接传到下一阶段
	assign  exe_aluop_o =  exe_aluop_i;
	assign	exe_pc_o	=  exe_pc_i;
	assign	exe_inst_o	=  exe_inst_i;
    assign  exe_wa_o    =  exe_wa_i;
    assign  exe_wreg_o  =  exe_wreg_i;
	assign	exe_mreg_o	=  exe_mreg_i;
	assign	exe_din_o	=  exe_din_i;
	assign	exe2diff_skip_o = exe2diff_skip_i;

    wire		[`REG_BUS]				logicres;           //保存逻辑运算的结果
	wire		[`REG_BUS]				shiftres;           //保存移位运算的结果
	wire       	[`REG_BUS]              moveres;           	//保存移动操作的结果
	wire		[`REG_BUS]			    arithres;           //保存算术运算的结果
	wire       	[`REG_BUS]              mulres;             //保存乘法运算需写回的结果
	wire       	[`DOUBLE_REG_BUS]       mulres_sign;        //保存有符号乘法临时结果
	wire       	[`DOUBLE_REG_BUS]       mulres_unsign;      //保存无符号乘法临时结果
	wire       	[`DOUBLE_REG_BUS]       mulres_sign_unsign; //保存有符号与无符号数相乘的临时结果

	//根据内部操作码aluop进行逻辑运算
	assign  logicres = (exe_aluop_i == `RISCV64_ANDI)   ?   (exe_src1_i & exe_src2_i)  :
	                   (exe_aluop_i == `RISCV64_AND)    ?   (exe_src1_i & exe_src2_i)  :
	                   (exe_aluop_i == `RISCV64_ORI)    ?   (exe_src1_i | exe_src2_i)  :
	                   (exe_aluop_i == `RISCV64_OR)     ?   (exe_src1_i | exe_src2_i)  :
	                   (exe_aluop_i == `RISCV64_XORI)   ?   (exe_src1_i ^ exe_src2_i)  :
	                   (exe_aluop_i == `RISCV64_XOR)    ?   (exe_src1_i ^ exe_src2_i)  :
	                   (exe_aluop_i == `RISCV64_LUI)    ?   exe_src2_i 				   :
	                   (exe_aluop_i == `RISCV64_AUIPC)  ?   (exe_src1_i + exe_src2_i)  :  `ZERO_WORD;

	wire		[`REG_BUS]				add_test;
	wire 		[`REG_BUS]				sub_test;
	wire 		[`REG_BUS]				lshift_test;
	wire 		[`REG_BUS]				rshift_test;
	assign	add_test = exe_src1_i + exe_src2_i;
	assign	sub_test = exe_src1_i + (~exe_src2_i) + 1;
	assign	lshift_test = exe_src1_i << exe_src2_i[4:0];
	assign	rshift_test = {32'b0,exe_src1_i[31:0] >> exe_src2_i[4:0]};
	
	//根据内部操作码aluop进行算术运算
	assign	arithres = (exe_aluop_i == `RISCV64_ADDI)	?	(exe_src1_i + exe_src2_i)  															:
					   (exe_aluop_i == `RISCV64_ADD)    ?   (exe_src1_i + exe_src2_i)  															:
					   (exe_aluop_i == `RISCV64_ADDIW)	?	{{32{add_test[31]}},{add_test[31:0]}}												:
					   (exe_aluop_i == `RISCV64_ADDW)	?	{{32{add_test[31]}},{add_test[31:0]}}												:
					   (exe_aluop_i == `RISCV64_SUB)    ?   (exe_src1_i + (~exe_src2_i) + 1)   													:
					   (exe_aluop_i == `RISCV64_SUBW)	?	{{32{sub_test[31]}},{sub_test[31:0]}}												:
					   (exe_aluop_i == `RISCV64_SLTI)   ?   (($signed(exe_src1_i) < $signed(exe_src2_i))   ?   64'h00000000_00000001 : 64'b0)  	:
					   (exe_aluop_i == `RISCV64_SLT)    ?   (($signed(exe_src1_i) < $signed(exe_src2_i))   ?   64'h00000000_00000001 : 64'b0)  	:
					   (exe_aluop_i == `RISCV64_SLTU)   ?   ((exe_src1_i < exe_src2_i)   ?   64'h00000000_00000001 : 64'b0)    					:
					   (exe_aluop_i == `RISCV64_SLTIU)  ?   ((exe_src1_i < exe_src2_i)   ?   64'h00000000_00000001 : 64'b0)    					:    `ZERO_WORD;			   
	//根据内部操作码aluop进行移位运算
	assign  shiftres = (exe_aluop_i == `RISCV64_SLLI)   ?   (exe_src1_i << exe_src2_i)    																			:
					   (exe_aluop_i == `RISCV64_SLLIW)	?	{{32{lshift_test[31]}},{lshift_test[31:0]}}																:
	                   (exe_aluop_i == `RISCV64_SRLI)   ?   (exe_src1_i >> exe_src2_i)    																			:
					   (exe_aluop_i == `RISCV64_SRLIW)	?	{{32{rshift_test[31]}},{rshift_test[31:0]}}																:
	                   (exe_aluop_i == `RISCV64_SRAI)   ?   (({64{exe_src1_i[63]}} << (7'd64 - {1'b0,exe_src2_i[5:0]})) | (exe_src1_i >> exe_src2_i))    			:
					   (exe_aluop_i == `RISCV64_SRAIW)	?	(({64{exe_src1_i[31]}} << (6'd32 - {1'b0,exe_src2_i[5:0]})) | {32'b0,(exe_src1_i[31:0] >> exe_src2_i)})			:
	                   (exe_aluop_i == `RISCV64_SLL)    ?   (exe_src1_i << exe_src2_i)    																			:
					   (exe_aluop_i == `RISCV64_SLLW)	?	{{32{lshift_test[31]}},{lshift_test[31:0]}}			:					
	                   (exe_aluop_i == `RISCV64_SRL)    ?   (exe_src1_i >> exe_src2_i)    																			: 
					   (exe_aluop_i == `RISCV64_SRLW)	?	{{32{rshift_test[31]}},{rshift_test[31:0]}}																:
	                   (exe_aluop_i == `RISCV64_SRA)    ?   (({64{exe_src1_i[63]}} << (7'd64 - {1'b0,exe_src2_i[5:0]})) | (exe_src1_i >> exe_src2_i))    			:
					   (exe_aluop_i == `RISCV64_SRAW)	?	(({64{exe_src1_i[31]}} << (6'd32 - {1'b0,exe_src2_i[4:0]})) | {32'b0,(exe_src1_i[31:0] >> exe_src2_i[4:0])})	:	`ZERO_WORD;
											

	//根据内部类型码alutype确定数据存储器存取操作的地址
	assign     moveres = (exe_alutype_i == `MOVE) ? (exe_src1_i + exe_src2_i) : `ZERO_WORD;
	   
	//根据内部操作码aluop进行乘法运算，并保存至下一阶段
	assign     mulres_sign     =   ($signed(exe_src1_i) * $signed(exe_src2_i));
	assign     mulres_unsign   =   ($unsigned({1'b0,exe_src1_i}) * $unsigned({1'b0,exe_src2_i}));
	assign     mulres_sign_unsign  =   ($signed(exe_src1_i) * $unsigned({1'b0,exe_src2_i}));
	
	assign     mulres = (exe_aluop_i == `RISCV64_MUL)    ?     {mulres_sign[63:0]}     :
	                    (exe_aluop_i == `RISCV64_MULH)   ?     {mulres_sign[127:64]}    :
	                    (exe_aluop_i == `RISCV64_MULHU)  ?     {mulres_unsign[127:64]}  :
	                    (exe_aluop_i == `RISCV64_MULHSU) ?     {mulres_sign_unsign[127:64]} :   `ZERO_WORD;									

	//根据内部操作码aluop进行csr指令操作
	assign	exe_csr_we	=	(exe_aluop_i == `RISCV64_ECALL || (global_trap_ena == `TRAP_ON && time_trap_ena == `TRAP_ON && clock_interr == `TRUE_V))	?	`TRUE_V	:
							(exe_alutype_i == `CSR)			?	`TRUE_V		:	`FALSE_V;
	assign	exe_csr_waddr=	(exe_aluop_i == `RISCV64_ECALL || (global_trap_ena == `TRAP_ON && time_trap_ena == `TRAP_ON && clock_interr == `TRUE_V))	?	`MEPC			:
							(exe_alutype_i == `CSR)			?	exe_csr_waddr_i	:	3'b0;
	assign	exe_csr_wdata=	(exe_aluop_i == `RISCV64_ECALL || (global_trap_ena == `TRAP_ON && time_trap_ena == `TRAP_ON && clock_interr == `TRUE_V))	?	exe_pc_i		:
							(exe_aluop_i == `RISCV64_CSRRW)	?	exe_src1_i	:
							(exe_aluop_i == `RISCV64_CSRRS)	?	(exe_src1_i | exe_src2_i)	:
							(exe_aluop_i == `RISCV64_CSRRC)	?	((~exe_src1_i) & exe_src2_i)	:
							(exe_aluop_i == `RISCV64_CSRRWI)?	exe_src1_i	:
							(exe_aluop_i == `RISCV64_CSRRSI)?	(exe_src1_i | exe_src2_i)	:
							(exe_aluop_i == `RISCV64_CSRRCI)?	((~exe_src1_i) & exe_src2_i)	:	`ZERO_WORD;

    assign  exe_wd_o    =   (exe_alutype_i == `ARITH)   	?   arithres    :  
					  		(exe_alutype_i ==  `LOGIC)      ?   logicres    :
					  		(exe_alutype_i ==  `SHIFT)      ?   shiftres    :
					  		(exe_alutype_i ==  `MUL  )      ?   mulres      :
					  		(exe_alutype_i ==  `MOVE )      ?   moveres     :
							(exe_alutype_i ==  `JUMP)       ?   ret_addr    :
							(exe_alutype_i ==  `CSR)		?	exe_src2_i	:	`ZERO_WORD;



	/********************************异常处理模块******************************/

	assign	excep_flush	=	((exe_aluop_i == `RISCV64_ECALL) || exe_aluop_i == `RISCV64_MRET || (global_trap_ena == `TRAP_ON && time_trap_ena == `TRAP_ON && clock_interr == `TRUE_V && exe_pc_i != `PC_INIT))	?	`TRUE_V		:	`FALSE_V;
	assign 	excep_flush_pc =(global_trap_ena == `TRAP_ON && time_trap_ena == `TRAP_ON && clock_interr == `TRUE_V && exe_pc_i != `PC_INIT)	?	mtvec_read	:				//(mtvec_read[ 1: 0] == 2'b0)		?	{2'b0,mtvec_read[63:2]}	:
							(exe_aluop_i == `RISCV64_ECALL)	?	mtvec_read				:															//{2'b0,mtvec_read[63:2]}	:
							(exe_aluop_i == `RISCV64_MRET)	?	mepc_read				:	`ZERO_WORD;
	assign	excep_code	= 	(clock_interr== `TRUE_V && global_trap_ena == `TRAP_ON && time_trap_ena == `TRAP_ON && exe_pc_i != `PC_INIT)	?	`TIME_TRAP_CODE	:
							(exe_aluop_i == `RISCV64_ECALL)	?	`ECALL_CODE	:	
							(exe_aluop_i == `RISCV64_MRET)	?	`MRET_CODE	:	3'b0;

	assign	interr_isdone = clock_interr == `TRUE_V && global_trap_ena == `TRAP_ON && time_trap_ena == `TRAP_ON && exe_pc_i != `PC_INIT;

	/********************************异常处理模块******************************/


endmodule