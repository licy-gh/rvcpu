
`include "defines.v"


module if_stage(
    input       wire        cpu_clk_50M,
    input       wire        cpu_rst_n,
    
    output 	    wire  [`INST_ADDR_BUS] 	    if_pc_o,
    output      wire [`INST_BUS     ]       if_inst_o,

    //流水线暂停信号
    input                                   id_stall,
    input                                   data_stall,

    //axi访问ram信号
    output                                  if_valid,
	output      reg  [`INST_ADDR_BUS]       if_addr,
    input       wire [`WORD_BUS]            if_data_read,
    input       wire                        if_read_isdone,

    input       wire                        axi_isused,     //excep_flush -> axi is used;

    output      wire                        handshake_done,

    //分支预测添加
    input       wire                        if_pre_jumpena,
    input       wire [`INST_ADDR_BUS]       if_pre_pc,
    output      wire                        jump_ena_o, //跳转使能，是否预测跳转
    output      wire [`INST_ADDR_BUS]       jump_pc_o, 	//预测跳转地址
    
    //预测错误回传信号
    input       wire [`INST_ADDR_BUS]       flush_addr,
    input       wire                        flush,
    output      wire [`INST_ADDR_BUS]       pc_plus_4,

    //异常处理跳转执行相关信号添加
    input       wire                        excep_flush,
    input       wire [`INST_ADDR_BUS]       excep_flush_pc
    
    );

    reg [`INST_ADDR_BUS] pc;

    assign pc_plus_4 = (cpu_rst_n == `RST_ENABLE) ? `PC_INIT   :    pc+4;

    reg ce;
    always@(posedge cpu_clk_50M) begin
        if(cpu_rst_n == `RST_ENABLE) begin
            ce <= `CHIP_DISABLE;          //复位的时候指令存储器禁用
        end else begin
            ce <= `CHIP_ENABLE;            //复位结束时，指令存储器使能
        end
    end
    
    /************************axi访问信号添加************************/

    assign  handshake_done = if_read_isdone;

    assign  if_valid    =   //(axi_isused == `TRUE_V)                         ?   `CHIP_DISABLE   :
                            (excep_flush == `TRUE_V)                        ?   ce              :
                            (cpu_rst_n == `RST_ENABLE || flush == `TRUE_V)  ?   `CHIP_DISABLE   :   ce;
    assign  if_addr     = pc;
    assign  if_inst_o   = (handshake_done == `TRUE_V) ?  if_data_read[31: 0]   :   `ZERO_INST;
    assign  if_pc_o     = (handshake_done == `TRUE_V) ?  pc                    :   `PC_INIT;
    /************************axi访问信号添加************************/

    /************************分支预测添加************************/
    wire is_jal  = (if_inst_o[6:0] == 7'b1101111)    ?   `TRUE_V    :    `FALSE_V;
    wire is_jalr = (if_inst_o[6:0] == 7'b1100111)    ?   `TRUE_V    :    `FALSE_V;
    wire is_bj   = (if_inst_o[6:0] == 7'b1100011)    ?   `TRUE_V    :    `FALSE_V;
    wire is_jump = is_jal | is_jalr | is_bj;   
     
    wire [`INST_ADDR_BUS] pc_next;

    assign  jump_ena_o  = (is_jump    == `TRUE_V && if_pre_jumpena == `JUMP_ENABLE)	  ?   	`JUMP_ENABLE	:	`JUMP_DISABLE;
    assign  jump_pc_o   = (jump_ena_o == `JUMP_ENABLE)	            ?   if_pre_pc       : 	`ZERO_WORD;
    assign  pc_next     = (excep_flush== `TRUE_V)                   ?   excep_flush_pc  :
                          (flush 	  == `TRUE_V)     		        ?   flush_addr  	:   
                          (jump_ena_o == `JUMP_ENABLE)          	?   jump_pc_o  		:   pc_plus_4;
    /***********************分支预测添加*************************/

    wire    pc_stall; 
    
    assign  pc_stall=  (excep_flush == `TRUE_V)    ?   `FALSE_V    :
                        (flush == `TRUE_V)          ?   `FALSE_V    :
                        (id_stall == `NOSTOP && data_stall == `NOSTOP && handshake_done == `TRUE_V) ?   `FALSE_V    :   `TRUE_V;


    always@(posedge cpu_clk_50M) begin
        if(ce == `CHIP_DISABLE)
            //指令存储器禁用的时候，PC保持初始值
            pc <= `PC_INIT;
        else if(pc_stall == `FALSE_V) begin
            pc <= pc_next;
        end
    end

/*
    always@(posedge cpu_clk_50M) begin
        if(ce == `CHIP_DISABLE)
            //指令存储器禁用的时候，PC保持初始值
            pc <= `PC_INIT;
        else if(flush == `TRUE_V) begin
            pc <= pc_next;
        end
        else if(stall == `NOSTOP && handshake_done == `TRUE_V) begin
            pc <= pc_next;                       //指令存储使能后，PC值每时钟周期加4
        end
    end
*/
     
endmodule