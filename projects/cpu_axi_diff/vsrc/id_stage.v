
`include "defines.v"

module id_stage(
    
    input  wire                     cpu_rst_n,
    // 从取指阶段获得的pc值与指令
    input  wire [`INST_ADDR_BUS]    id_pc_i,
    input  wire [`INST_ADDR_BUS]    pc_plus_4,
    input  wire [`INST_BUS     ]    id_inst_i,

    // 从通用寄存器堆读出的数据 
    input  wire [`REG_BUS      ]    rd1,
    input  wire [`REG_BUS      ]    rd2,

    // 送至执行阶段的译码信息
    output wire [`ALUTYPE_BUS  ]    id_alutype_o,
    output wire [`ALUOP_BUS    ]    id_aluop_o,
    output wire [`REG_ADDR_BUS ]    id_wa_o,
    output wire                     id_wreg_o,
    output wire                     id_mreg_o,
    output wire [`REG_BUS      ]    id_din_o,
	output wire [`INST_ADDR_BUS]	id_pc_o,
	output wire [`INST_BUS	   ]	id_inst_o,

    // 送至执行阶段的源操作数1、源操作数2
    output wire [`REG_BUS      ]    id_src1_o,
    output wire [`REG_BUS      ]    id_src2_o,
      
    // 送至读通用寄存器堆端口的地址
    output wire [`REG_ADDR_BUS ]    ra1,
    output wire [`REG_ADDR_BUS ]    ra2,

    /*****************数据的定向回传*************************/
    //从执行阶段获得的回传信号
    input   wire                    exe2id_wreg,
    input   wire[`REG_ADDR_BUS]     exe2id_wa,
    input   wire[`REG_BUS]          exe2id_wd,

    //从访存阶段获得的回传信号
    input   wire                    mem2id_wreg,
    input   wire[`REG_ADDR_BUS]     mem2id_wa,
    input   wire[`REG_BUS]          mem2id_wd,
    /*****************数据的定向回传*************************/

    /*****************加载相关的处理信号*************************/
    input   wire                    exe2id_mreg,
    input   wire                    mem2id_mreg,
    //暂停使能信号
    output  wire                    id_stall,

    /*****************加载相关的处理信号*************************/

    //串口输出添加测试数据
    output  wire                    uart_ena,
    output  wire [ 7: 0]            uart_data_out,

    //传给difftest的skip信号
    output  wire                    id2diff_skip_o,
    
    
    // 从取指阶段获得的分支预测信息
    input   wire                    id_jump_ena_i,      //预测跳转方向
    input   wire [`INST_ADDR_BUS]   id_jump_pc_i,       //预测跳转目标pc值
    
    // 送回译码阶段的跳转地址
    output  wire                     flush,             //分支预测错误时回传流水线冲刷信号
    output  wire [`INST_ADDR_BUS]    flush_addr,        //分支预测错误时回传流水线冲刷地址
    output  wire [`INST_ADDR_BUS]    ret_addr,          //函数调用的返回地址
    
    // 送至btb表的更新信息
    output  wire                    btb_update_req,     //btb表写使能信号
    output  wire 		            btb_update_state,   //btb表当前写指令是否发生跳转
    output  wire [`PC_ADDR_BTB  ]   btb_update_pc,      //btb表当前写入指令pc
    output  wire [`INST_ADDR_BUS]   btb_update_prepc,    //btb表当前写入指令跳转pc

    //  csr寄存器相关信号
    output  wire [`CSR_ADDR_BUS]    csr_raddr,
    input   wire [`REG_BUS     ]    csr_rdata,
    output  wire [`CSR_ADDR_BUS]    id_csr_waddr,

    input   wire                    exe2id_csr_we,
    input   wire [`CSR_ADDR_BUS]    exe2id_csr_waddr,
    input   wire [`REG_BUS     ]    exe2id_csr_wdata,

    input   wire                    mem2id_csr_we,
    input   wire [`CSR_ADDR_BUS]    mem2id_csr_waddr,
    input   wire [`REG_BUS     ]    mem2id_csr_wdata

);

    // 根据小端模式组织指令字
    wire [`INST_BUS] id_inst;
    assign id_inst = (cpu_rst_n == `RST_ENABLE)	?	32'b0 : id_inst_i;                          //(cpu_rst_n == `RST_ENABLE)   ?   `ZERO_INST  :   {id_inst_i[7:0],id_inst_i[15:8],id_inst_i[23:16],id_inst_i[31:24]};

    assign uart_ena = (id_inst == 32'h0000007b/* || id_pc_i == 64'h80000054*/)   ?   `TRUE_V :   `FALSE_V;
    assign uart_data_out = //(id_inst == 31'h0000007b)    ?   rd1[ 7: 0] : 8'b0;
                            (fwrd1 == 2'b01)    ?   exe2id_wd[ 7: 0]   :   
                            (fwrd1 == 2'b10)    ?   mem2id_wd[ 7: 0]   :  rd1[ 7: 0];

    wire [ 6: 0] funct7  = id_inst[31 :25];
    wire [ 4: 0] rs2     = id_inst[24 :20];
    wire [ 4: 0] rs1     = id_inst[19 :15];
    wire [ 2: 0] funct3  = id_inst[14 :12];
    wire [ 4: 0] rd      = id_inst[11 : 7];
    wire [ 6: 0] op      = id_inst[6  : 0];

    // csr指令字段
    wire [11: 0] csr     = id_inst[31 :20];


    // 第一级译码逻辑产生SYSCALL、ERET、MFC0和MTC0指令的识别信号

    wire inst_cal_i = ~op[6]&~op[5]&op[4]&~op[3]&~op[2]&op[1]&op[0];    //i型运算指令op字段
    wire inst_addi  = inst_cal_i&~funct3[2]&~funct3[1]&~funct3[0];
    wire inst_slti  = inst_cal_i&~funct3[2]&funct3[1]&~funct3[0];
    wire inst_sltiu = inst_cal_i&~funct3[2]&funct3[1]&funct3[0];
    wire inst_xori  = inst_cal_i&funct3[2]&~funct3[1]&~funct3[0];
    wire inst_ori   = inst_cal_i&funct3[2]&funct3[1]&~funct3[0];
    wire inst_andi  = inst_cal_i&funct3[2]&funct3[1]&funct3[0];
    wire inst_slli  = inst_cal_i&~funct3[2]&~funct3[1]&funct3[0];
    wire inst_srli  = inst_cal_i&funct3[2]&~funct3[1]&funct3[0]&~funct7[6]&~funct7[5]&~funct7[4]&~funct7[3]&~funct7[2]&~funct7[1];
    wire inst_srai  = inst_cal_i&funct3[2]&~funct3[1]&funct3[0]&~funct7[6]&funct7[5]&~funct7[4]&~funct7[3]&~funct7[2]&~funct7[1];
	
	wire inst_cal_iw= ~op[6]&~op[5]&op[4]&op[3]&~op[2]&op[1]&op[0];  //RISCV64-I ONLY
	wire inst_addiw = inst_cal_iw&~funct3[2]&~funct3[1]&~funct3[0];
	wire inst_slliw = inst_cal_iw&~funct3[2]&~funct3[1]&funct3[0];
	wire inst_srliw = inst_cal_iw&funct3[2]&~funct3[1]&funct3[0]&~funct7[6]&~funct7[5]&~funct7[4]&~funct7[3]&~funct7[2]&~funct7[1]&~funct7[0];
	wire inst_sraiw = inst_cal_iw&funct3[2]&~funct3[1]&funct3[0]&~funct7[6]&funct7[5]&~funct7[4]&~funct7[3]&~funct7[2]&~funct7[1]&~funct7[0];
    
    wire inst_cal_r = ~op[6]&op[5]&op[4]&~op[3]&~op[2]&op[1]&op[0];    //r型运算指令op字段
    wire inst_add   = inst_cal_r&~funct3[2]&~funct3[1]&~funct3[0]&~funct7[6]&~funct7[5]&~funct7[4]&~funct7[3]&~funct7[2]&~funct7[1]&~funct7[0];
    wire inst_sub   = inst_cal_r&~funct3[2]&~funct3[1]&~funct3[0]&~funct7[6]&funct7[5]&~funct7[4]&~funct7[3]&~funct7[2]&~funct7[1]&~funct7[0];
    wire inst_sll   = inst_cal_r&~funct3[2]&~funct3[1]&funct3[0];
    wire inst_slt   = inst_cal_r&~funct3[2]&funct3[1]&~funct3[0];
    wire inst_sltu  = inst_cal_r&~funct3[2]&funct3[1]&funct3[0];
    wire inst_xor   = inst_cal_r&funct3[2]&~funct3[1]&~funct3[0];
    wire inst_srl   = inst_cal_r&funct3[2]&~funct3[1]&funct3[0]&~funct7[6]&~funct7[5]&~funct7[4]&~funct7[3]&~funct7[2]&~funct7[1]&~funct7[0];
    wire inst_sra   = inst_cal_r&funct3[2]&~funct3[1]&funct3[0]&~funct7[6]&funct7[5]&~funct7[4]&~funct7[3]&~funct7[2]&~funct7[1]&~funct7[0];
    wire inst_or    = inst_cal_r&funct3[2]&funct3[1]&~funct3[0];
    wire inst_and   = inst_cal_r&funct3[2]&funct3[1]&funct3[0];
	
	wire inst_cal_rw= ~op[6]&op[5]&op[4]&op[3]&~op[2]&op[1]&op[0];
	wire inst_addw	= inst_cal_rw&~funct3[2]&~funct3[1]&~funct3[0]&~funct7[6]&~funct7[5]&~funct7[4]&~funct7[3]&~funct7[2]&~funct7[1]&~funct7[0];
	wire inst_subw	= inst_cal_rw&~funct3[2]&~funct3[1]&~funct3[0]&~funct7[6]&funct7[5]&~funct7[4]&~funct7[3]&~funct7[2]&~funct7[1]&~funct7[0];
	wire inst_sllw	= inst_cal_rw&~funct3[2]&~funct3[1]&funct3[0];
	wire inst_srlw	= inst_cal_rw&funct3[2]&~funct3[1]&funct3[0]&~funct7[6]&~funct7[5]&~funct7[4]&~funct7[3]&~funct7[2]&~funct7[1]&~funct7[0];
	wire inst_sraw	= inst_cal_rw&funct3[2]&~funct3[1]&funct3[0]&~funct7[6]&funct7[5]&~funct7[4]&~funct7[3]&~funct7[2]&~funct7[1]&~funct7[0];
	
	wire inst_lui	= ~op[6]&op[5]&op[4]&~op[3]&op[2]&op[1]&op[0];
	wire inst_auipc	= ~op[6]&~op[5]&op[4]&~op[3]&op[2]&op[1]&op[0];
	wire inst_jal	= op[6]&op[5]&~op[4]&op[3]&op[2]&op[1]&op[0];
	wire inst_jalr	= op[6]&op[5]&~op[4]&~op[3]&op[2]&op[1]&op[0];
	
	wire inst_off	= op[6]&op[5]&~op[4]&~op[3]&~op[2]&op[1]&op[0];		//分支指令op字段
	wire inst_beq	= inst_off&~funct3[2]&~funct3[1]&~funct3[0];
	wire inst_bne	= inst_off&~funct3[2]&~funct3[1]&funct3[0];
	wire inst_blt	= inst_off&funct3[2]&~funct3[1]&~funct3[0];
	wire inst_bge	= inst_off&funct3[2]&~funct3[1]&funct3[0];
	wire inst_bltu	= inst_off&funct3[2]&funct3[1]&~funct3[0];
	wire inst_bgeu	= inst_off&funct3[2]&funct3[1]&funct3[0];
	
	wire inst_load	= ~op[6]&~op[5]&~op[4]&~op[3]&~op[2]&op[1]&op[0];	//加载指令op字段
	wire inst_lb	= inst_load&~funct3[2]&~funct3[1]&~funct3[0];
	wire inst_lh	= inst_load&~funct3[2]&~funct3[1]&funct3[0];
	wire inst_ld   	= inst_load&~funct3[2]&funct3[1]&funct3[0];
	wire inst_lw	= inst_load&~funct3[2]&funct3[1]&~funct3[0];
	wire inst_lwu	= inst_load&funct3[2]&funct3[1]&~funct3[0];
	wire inst_lbu	= inst_load&funct3[2]&~funct3[1]&~funct3[0];
	wire inst_lhu	= inst_load&funct3[2]&~funct3[1]&funct3[0];
	
	wire inst_store	= ~op[6]&op[5]&~op[4]&~op[3]&~op[2]&op[1]&op[0];	//存储指令op字段
	wire inst_sb	= inst_store&~funct3[2]&~funct3[1]&~funct3[0];
	wire inst_sh	= inst_store&~funct3[2]&~funct3[1]&funct3[0];
	wire inst_sw	= inst_store&~funct3[2]&funct3[1]&~funct3[0];
	wire inst_sd	= inst_store&~funct3[2]&funct3[1]&funct3[0];
	
	wire inst_mu   = ~op[6]&op[5]&op[4]&~op[3]&~op[2]&op[1]&op[0]&~funct7[6]&~funct7[5]&~funct7[4]&~funct7[3]&~funct7[2]&~funct7[1]&funct7[0];     //乘法指令
	wire inst_mul  = inst_mu&~funct3[2]&~funct3[1]&~funct3[0];
    wire inst_mulh = inst_mu&~funct3[2]&~funct3[1]&funct3[0];
    wire inst_mulhu= inst_mu&~funct3[2]&~funct3[1]&funct3[0];
    wire inst_mulhsu= inst_mu&~funct3[2]&funct3[1]&~funct3[0];

    //csr指令
    wire inst_csr   = op[6]&op[5]&op[4]&~op[3]&~op[2]&op[1]&op[0];
    wire inst_csrrw = inst_csr&~funct3[2]&~funct3[1]&funct3[0];
    wire inst_csrrs = inst_csr&~funct3[2]&funct3[1]&~funct3[0];
    wire inst_csrrc = inst_csr&~funct3[2]&funct3[1]&funct3[0];
    wire inst_csrrwi= inst_csr&funct3[2]&~funct3[1]&funct3[0];
    wire inst_csrrsi= inst_csr&funct3[2]&funct3[1]&~funct3[0];
    wire inst_csrrci= inst_csr&funct3[2]&funct3[1]&funct3[0];
    wire ecall      = inst_csr&~funct3[2]&~funct3[1]&~funct3[0]&(csr == 12'b000000000000);
    wire mret       = (id_inst[31: 0] == 32'h30200073);
    
    
    // 操作类型alutype
    assign id_alutype_o[2] = inst_slli | inst_srli | inst_srai | inst_slliw | inst_srliw | inst_sraiw | inst_sll | inst_srl | inst_sra | inst_sllw | inst_srlw | inst_sraw | inst_jal | inst_jalr
                              | inst_mul | inst_mulh | inst_mulhu | inst_mulhsu | inst_csrrw | inst_csrrwi | inst_csrrs | inst_csrrsi | inst_csrrc | inst_csrrci;
    assign id_alutype_o[1] = inst_andi | inst_ori | inst_xori | inst_and | inst_or | inst_xor | inst_lui | inst_auipc | inst_csrrw | inst_csrrwi | inst_csrrs | inst_csrrsi | inst_csrrc | inst_csrrci
                             | inst_mul | inst_mulh | inst_mulhu | inst_mulhsu | inst_lwu | inst_ld | inst_lw | inst_lh | inst_lhu | inst_lb | inst_lbu | inst_sw | inst_sh | inst_sb | inst_sd;
    assign id_alutype_o[0] = inst_addi | inst_slti | inst_sltiu | inst_addiw | inst_add | inst_sub | inst_slt | inst_sltu | inst_addw | inst_subw | inst_jal | inst_jalr 
							| inst_lwu | inst_ld | inst_lw | inst_lh | inst_lhu | inst_lb | inst_lbu | inst_sw | inst_sh | inst_sb | inst_sd | inst_csrrw | inst_csrrwi | inst_csrrs | inst_csrrsi | inst_csrrc | inst_csrrci;
                             
    // 内部操作码aluop
    assign id_aluop_o[7]   = inst_mul | inst_mulh | inst_mulhu | inst_mulhsu | inst_lwu | inst_ld | inst_sd | ecall | mret;
    assign id_aluop_o[6]   = inst_slli | inst_srli | inst_srai | inst_sll | inst_srl | inst_sra | inst_slliw | inst_srliw | inst_sraiw | inst_sllw | inst_srlw | inst_sraw | inst_lui | inst_auipc | inst_jal | inst_jalr 
							 | inst_beq | inst_bne | inst_blt | inst_bltu | inst_bge | inst_bgeu | inst_lw | inst_lh | inst_lhu | inst_lb | inst_lbu | inst_sw | inst_sh | inst_sb | ecall | mret;
    assign id_aluop_o[5]   = inst_andi | inst_ori | inst_xori | inst_and | inst_or | inst_xor| inst_beq | inst_bne | inst_blt | inst_bltu | inst_bge | inst_bgeu 
                             | inst_lw | inst_lh | inst_lhu | inst_lb | inst_lbu | inst_sw | inst_sh | inst_sb | ecall | mret;
    assign id_aluop_o[4]   = inst_addi | inst_slti | inst_sltiu | inst_add | inst_sub | inst_slt | inst_sltu | inst_addiw | inst_addw | inst_subw | inst_lui | inst_auipc | inst_jal | inst_jalr |
							 | inst_lwu | inst_ld | inst_sd | inst_lw | inst_lh | inst_lhu | inst_lb | inst_lbu | inst_sw | inst_sh | inst_sb | ecall | mret;
    assign id_aluop_o[3]   = inst_addiw | inst_addw | inst_subw | inst_srliw | inst_sraiw | inst_sllw | inst_srlw | inst_sraw;
    assign id_aluop_o[2]   = inst_add | inst_sub | inst_slt | inst_sltu | inst_and | inst_or | inst_xor | inst_sll | inst_srl | inst_sra | inst_slliw | inst_sraw  
							 | inst_jalr | inst_bltu | inst_bge | inst_bgeu | inst_lb | inst_lbu | inst_sw | inst_sh | inst_csrrwi | inst_csrrsi | inst_csrrci;
    assign id_aluop_o[1]   = inst_slti | inst_sltiu | inst_slt | inst_sltu | inst_subw | inst_ori | inst_xori | inst_xor | inst_srli | inst_srai | inst_sra | inst_slliw | inst_sllw | inst_srlw | inst_auipc | inst_jal 
                             | inst_bne | inst_blt | inst_bgeu | inst_lh | inst_lhu | inst_sw | inst_sh | inst_sd | inst_csrrs | inst_csrrc | inst_csrrci | mret;
    assign id_aluop_o[0]   = inst_addi | inst_sltiu | inst_sub | inst_sltu | inst_addw | inst_andi | inst_xori | inst_or | inst_slli | inst_srai | inst_srl | inst_slliw | inst_sraiw | inst_srlw | inst_lui | inst_jal 
                             | inst_beq | inst_blt | inst_bge | inst_lwu | inst_lw | inst_lhu | inst_lbu | inst_sh | inst_csrrw | inst_csrrc | inst_csrrsi | ecall;
    
    //  目的寄存器写使能信号
    assign id_wreg_o       = (inst_addi | inst_slti | inst_sltiu | inst_xori | inst_ori | inst_andi
                             | inst_slli | inst_srli | inst_srai | inst_add | inst_sub | inst_sll 
                             | inst_addiw | inst_addw | inst_subw 
                             | inst_slt | inst_sltu | inst_xor | inst_srl | inst_sra | inst_or | inst_and
                             | inst_slliw | inst_srliw | inst_sraiw | inst_sllw | inst_srlw | inst_sraw
                             | inst_lui | inst_auipc | inst_jal | inst_jalr 
                             | inst_lb | inst_lbu | inst_lh | inst_lhu | inst_lwu | inst_lw | inst_ld  
                             | inst_mul | inst_mulh | inst_mulhu | inst_mulhsu
                             | inst_csrrw | inst_csrrwi | inst_csrrs | inst_csrrsi | inst_csrrc | inst_csrrci);

    //  移位使能信号
    wire shift      =   inst_slli | inst_srli | inst_srai | inst_sll | inst_srl | inst_sra | inst_slliw | inst_srliw | inst_sraiw | inst_sllw | inst_srlw | inst_sraw;
    wire shift_r    =   inst_sll | inst_srl | inst_sra | inst_sllw | inst_srlw | inst_sraw;

    //移位位数
    wire [`INST_ADDR_BUS] shift_ra    = (shift_r == `SHIFT_ENABLE && fwrd2 == 2'b01)  ?     {58'b0,exe2id_wd[5:0]}          :
                                        (shift_r == `SHIFT_ENABLE && fwrd2 == 2'b10)  ?     {58'b0,mem2id_wd[5:0]}          :
                                        (shift_r == `SHIFT_ENABLE)  ?     {58'b0, rd2[5:0]}             :
                                        (shift   == `SHIFT_ENABLE)  ?     {58'b0,id_inst[25], rs2[4:0]} : `ZERO_WORD; 

    // 立即数使能信号
    wire immsel =   inst_addi | inst_slti | inst_sltiu | inst_addiw | inst_andi | inst_ori | inst_xori 
                    | inst_lui | inst_auipc | inst_lb | inst_lh | inst_lwu | inst_ld | inst_lw | inst_lbu | inst_lhu | inst_sw | inst_sh | inst_sb | inst_sd 
                    | inst_jal | inst_jalr;

    wire i_imm =    inst_jalr | inst_lb | inst_lh | inst_lwu | inst_ld | inst_lw | inst_lbu | inst_lhu | inst_addi | inst_slti | inst_sltiu | inst_addiw
                    | inst_xori | inst_ori | inst_andi;
    wire s_imm = inst_sb | inst_sh | inst_sw | inst_sd;
    wire b_imm =    inst_beq | inst_bne | inst_blt | inst_bge | inst_bltu | inst_bgeu;
    wire u_imm =    inst_lui | inst_auipc;
    wire j_imm =    inst_jal;
    

    //存储器到寄存器使能信号
    assign id_mreg_o = inst_lb | inst_lbu | inst_lw | inst_lwu | inst_lh | inst_lhu | inst_ld;
    //读通用寄存器堆端口1的地址为rs1字段，
    assign  ra1     = (id_inst == 32'h0000007b) ?   5'b01010    :   rs1;
    
    //读通用寄存器堆端口2的地址为rs2字段，
    assign  ra2     = rs2;
    
    //获得指令操作所需的立即数
    wire    [`REG_BUS]  imm_ext =   (i_imm) ? {{53{id_inst[31]}},id_inst[30:20]} :
                                    (s_imm) ? {{53{id_inst[31]}},id_inst[30:25],id_inst[11:7]} :
                                    (b_imm) ? {{52{id_inst[31]}},id_inst[7],id_inst[30:25],id_inst[11:8],1'b0} :
                                    (u_imm) ? {{33{id_inst[31]}},id_inst[30:12],12'b0} :   
                                    (j_imm) ? {{44{id_inst[31]}},id_inst[19:12],id_inst[20],id_inst[30:21],1'b0} : `ZERO_WORD;

    //获得待写入目的寄存器rd的地址  //无条件跳转指令默认x1寄存器
    assign id_wa_o =   rd;

    /********************csr指令添加*****************************/
    wire csr_i  =   inst_csrrwi | inst_csrrsi | inst_csrrci;
    wire csr_r  =   inst_csrrw | inst_csrrs | inst_csrrc;
    assign  csr_raddr   =   (mret)  ?   `MEPC   :
                            (ecall) ?   `MTVEC  :
                            (inst_csr == `TRUE_V && csr == 12'h300)  ?   `MSTATUS   :
                            (inst_csr == `TRUE_V && csr == 12'h305)  ?   `MTVEC     :
                            (inst_csr == `TRUE_V && csr == 12'h341)  ?   `MEPC      :
                            (inst_csr == `TRUE_V && csr == 12'h342)  ?   `MCAUSE    :
                            (inst_csr == `TRUE_V && csr == 12'h304)  ?   `MIE       :
                            (inst_csr == `TRUE_V && csr == 12'h344)  ?   `MIP       :
                            (inst_csr == `TRUE_V && csr == 12'hB00)  ?   `MCYCLE    :
                            (inst_csr == `TRUE_V && csr == 12'h340)  ?   `MSCRATCH  :       `ZERO_WORD;
    wire[`WORD_BUS] csr_imm =   (csr_i) ?   {27'b0,rs1} :   `ZERO_WORD;
    assign  id_csr_waddr    =   (inst_csr && ~mret && ~ecall)  ?   csr_raddr   :  `ZERO_WORD; 
        
    //csr定向回传添加
    wire[`WORD_BUS] csr_rdata_select = (exe2id_csr_we == `WRITE_ENABLE && exe2id_csr_waddr == csr_raddr && csr_raddr != 3'b0)   ?   exe2id_csr_wdata    :
                                       (mem2id_csr_we == `WRITE_ENABLE && mem2id_csr_waddr == csr_raddr && csr_raddr != 3'b0)   ?   mem2id_csr_wdata    :   csr_rdata;
        
        
    /********************csr指令添加*****************************/

    /********************定向回传添加*****************************/
    //源操作数1选择信号（可能来自当前pc值或寄存器读端口1或定向回传）
    wire [1:0]  fwrd1   =   (exe2id_wreg == `WRITE_ENABLE && exe2id_wa == ra1 && ra1 != 5'b0)  ?   2'b01   :
                            (mem2id_wreg == `WRITE_ENABLE && mem2id_wa == ra1 && ra1 != 5'b0)  ?   2'b10   :   2'b11;

    //源操作数2选择信号（可能来自立即数或寄存器读端口2或定向回传）
    wire [1:0]  fwrd2   =   (exe2id_wreg == `WRITE_ENABLE && exe2id_wa == ra2 && ra2 != 5'b0)  ?   2'b01   :
                            (mem2id_wreg == `WRITE_ENABLE && mem2id_wa == ra2 && ra2 != 5'b0)  ?   2'b10   :   2'b11;                      

    /********************定向回传添加*****************************/

    
    //获得源操作数1来自读端口rs1
    assign  id_src1_o   =   (csr_i)             ?   csr_imm     :
                            //(csr_r)             ?   rd1         :
                            (inst_auipc)        ?   id_pc_i     :
                            (fwrd1 == 2'b01)    ?   exe2id_wd   :   
                            (fwrd1 == 2'b10)    ?   mem2id_wd   :  rd1;

    //获得源操作数2
    assign  id_src2_o   =   (csr_i | csr_r)             ?   csr_rdata_select   :
                            (immsel == `IMM_ENABLE )    ?   imm_ext     : 
                            (shift == `SHIFT_ENABLE)    ?   shift_ra    :
                            (fwrd2 == 2'b01)            ?   exe2id_wd   :
                            (fwrd2 == 2'b10)            ?   mem2id_wd   :   rd2;

    //获得访存阶段要存入数据存储器的数据sw sb sh 指令来自rs2寄存器
    assign  id_din_o    =   (fwrd2 == 2'b01)    ?   exe2id_wd    :
                            (fwrd2 == 2'b10)    ?   mem2id_wd    :   rd2 ;

    //加载相关暂停信号
    assign  id_stall    =   ((fwrd1 == 2'b01 || fwrd2 == 2'b01) && (exe2id_mreg == `TRUE_V))    ?   `STOP   :
                            ((fwrd1 == 2'b10 || fwrd2 == 2'b10) && (mem2id_mreg == `TRUE_V))    ?   `STOP   :   `NOSTOP;

    //获得转移控制信号
    wire equ    =   (inst_beq   )   ?   (id_src1_o  ==  id_src2_o)  :
                    (inst_bne   )   ?   (id_src1_o  !=  id_src2_o)  :
                    (inst_blt   )   ?   ($signed(id_src1_o)     <   $signed(id_src2_o))   :
                    (inst_bltu  )   ?   ($unsigned(id_src1_o)   <   $unsigned(id_src2_o))   :
                    (inst_bge   )   ?   ($signed(id_src1_o)     >=  $signed(id_src2_o))     :
                    (inst_bgeu  )   ?   ($unsigned(id_src1_o)   >=  $unsigned(id_src2_o))   :   1'b0;
                    
    //生成转移地址计算所需信号
    wire [`INST_ADDR_BUS] imm_jump = (inst_jalr) ? {{52{id_inst[31]}},id_inst[31: 20]}  :
                                     (inst_jal ) ? {{44{id_inst[31]}},id_inst[19:12],id_inst[20],id_inst[30:21],1'b0}  :
                                     (b_imm)     ? {{52{id_inst[31]}},id_inst[7],id_inst[30:25],id_inst[11:8],1'b0}  :   `ZERO_WORD;

    wire    jtsel;
    //获得转移指令使能信号
    assign  jtsel     = inst_jal | inst_jalr | equ;
    
    //获得转移地址
    wire [`INST_ADDR_BUS] jump_addr;
    assign  jump_addr = (inst_jal | equ)    ?   id_pc_i + imm_jump  :
                        (inst_jalr)         ?   id_src1_o + imm_jump      :   pc_plus_4;
                        
    assign  flush   =   (id_stall == `STOP)    ?   `JUMP_DISABLE   :
                        (jtsel == `JUMP_ENABLE  && id_jump_ena_i == `JUMP_DISABLE)                              ?   1'b1    :
                        (jtsel == `JUMP_DISABLE && id_jump_ena_i == `JUMP_ENABLE )                              ?   1'b1    :
                        (jtsel == `JUMP_ENABLE  && id_jump_ena_i == `JUMP_ENABLE && jump_addr != id_jump_pc_i)  ?   1'b1    :    1'b0;   
    
    assign  flush_addr = (flush == `FALSE_V)    ?   `ZERO_WORD  :
                         (flush == `TRUE_V && jtsel == `JUMP_ENABLE)     ?   jump_addr   :
                         (flush == `TRUE_V && jtsel == `JUMP_DISABLE)    ?   pc_plus_4   :   `ZERO_WORD;
    
    assign  btb_update_req  =   (id_stall == `STOP)    ?   `FALSE_V : (inst_jal | inst_jalr | b_imm);                               
    assign  btb_update_state=   jtsel;
    assign  btb_update_pc   =   (btb_update_req == `TRUE_V)   ?   id_pc_i[11:2]   :   10'b0;
    assign  btb_update_prepc=   (btb_update_req == `TRUE_V)   ?   jump_addr 	     :   `ZERO_WORD; 
    
    //生成子程序调用返回地址
    assign ret_addr   = pc_plus_4;

	//pc值与指令向下传递//difftest needed
    assign id_pc_o   = id_pc_i;
    assign id_inst_o = id_inst_i;
    assign id2diff_skip_o = (id_inst == 32'h0000007b) || (id_inst[6:0] == 7'b1110011 && id_inst[14:12] != 3'b000 && id_inst[31:20] == 12'hB00)  ?   `TRUE_V :   `FALSE_V;

endmodule

