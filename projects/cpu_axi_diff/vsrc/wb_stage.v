
`include "defines.v"

module wb_stage(
	
	//从访存阶段获得的信息
	input	wire[`REG_ADDR_BUS]	wb_wa_i,
	input	wire				wb_wreg_i,
	input	wire[`REG_BUS]		wb_dreg_i,
	input	wire				wb_mreg_i,
	input	wire[`BSEL_BUS]		wb_dre_i,
	input	wire 				sign,
    // to diff
	input	wire[`INST_ADDR_BUS]wb_pc_i,
	input	wire[`INST_BUS]		wb_inst_i,
    input   wire[`EXCEP_CODE_BUS]wb_excep_code_i,
    input   wire                wb2diff_skip_i,
	
	//写回目的寄存器的数据
	output	wire[`REG_ADDR_BUS]	wb_wa_o,
	output	wire				wb_wreg_o,
	output	wire[`WORD_BUS]		wb_wd_o,
    // to diff
	output	wire[`INST_ADDR_BUS]wb_pc_o,
	output	wire[`INST_BUS]		wb_inst_o,
    output  wire[`EXCEP_CODE_BUS]wb_excep_code_o,
    output  wire                wb2diff_skip_o,

	//来自数据存储器的数据
	input	wire[`WORD_BUS]		dm,

    //csr指令添加信号
    input   wire                wb_csr_we_i,
    input   wire[`CSR_ADDR_BUS] wb_csr_waddr_i,
    input   wire[`REG_BUS     ] wb_csr_wdata_i,

    output  wire                csr_we,
    output  wire[`CSR_ADDR_BUS] csr_waddr,
    output  wire[`REG_BUS    ]  csr_wdata
	
);
	
    //传给csr寄存器堆的信息
    assign  csr_we      =   wb_csr_we_i;
    assign  csr_waddr   =   wb_csr_waddr_i;
    assign  csr_wdata   =   (wb_csr_we_i == `TRUE_V && wb_csr_waddr_i == `MSTATUS && (wb_csr_wdata_i[16:15] == 2'b11 || wb_csr_wdata_i[14:13] == 2'b11))    ?   {1'b1,wb_csr_wdata_i[62:0]} :
                            (wb_csr_we_i == `TRUE_V && wb_csr_waddr_i == `MSTATUS)  ?   {1'b0,wb_csr_wdata_i[62:0]} : wb_csr_wdata_i;

	//传至通用寄存器堆的信息
	assign	wb_wa_o		=	wb_wa_i;
	assign	wb_wreg_o	=	wb_wreg_i;

	//difftest
	assign	wb_pc_o		=	wb_pc_i;
	assign	wb_inst_o	=	wb_inst_i;
    assign  wb_excep_code_o =   wb_excep_code_i;
    assign  wb2diff_skip_o  =   wb2diff_skip_i;
	/*
    wire [`WORD_BUS] data  =    (wb_dre_i == 64'hffff_ffff_ffff_ffff        ) ? dm  :
                                (wb_dre_i == 64'hffff_ffff_0000_0000 && sign) ? {{32{dm[63]}}, dm[63:32]} :
                                (wb_dre_i == 64'h0000_0000_ffff_ffff && sign) ? {{32{dm[31]}}, dm[31: 0]} :
                                (wb_dre_i == 64'hff00_0000_0000_0000 && sign) ? {{56{dm[63]}}, dm[63:56]} :
                                (wb_dre_i == 64'h00ff_0000_0000_0000 && sign) ? {{56{dm[55]}}, dm[55:48]} :
                                (wb_dre_i == 64'h0000_ff00_0000_0000 && sign) ? {{56{dm[47]}}, dm[47:40]} :
                                (wb_dre_i == 64'h0000_00ff_0000_0000 && sign) ? {{56{dm[39]}}, dm[39:32]} :
                                (wb_dre_i == 64'h0000_0000_ff00_0000 && sign) ? {{56{dm[31]}}, dm[31:24]} :
                                (wb_dre_i == 64'h0000_0000_00ff_0000 && sign) ? {{56{dm[23]}}, dm[23:16]} :
                                (wb_dre_i == 64'h0000_0000_0000_ff00 && sign) ? {{56{dm[15]}}, dm[15:8 ]} :
                                (wb_dre_i == 64'h0000_0000_0000_00ff && sign) ? {{56{dm[ 7]}}, dm[7 :0 ]} : 
                                (wb_dre_i == 64'hffff_0000_0000_0000 && sign) ? {{48{dm[63]}}, dm[63:48]} :
                                (wb_dre_i == 64'h0000_ffff_0000_0000 && sign) ? {{48{dm[47]}}, dm[47:32]} : 
                                (wb_dre_i == 64'h0000_0000_ffff_0000 && sign) ? {{48{dm[31]}}, dm[31:16]} :
                                (wb_dre_i == 64'h0000_0000_0000_ffff && sign) ? {{48{dm[15]}}, dm[15: 0]} :
                                (wb_dre_i == 64'hffff_ffff_0000_0000 && ~sign) ? {{32{1'b0}},  dm[63:32]} :
                                (wb_dre_i == 64'h0000_0000_ffff_ffff && ~sign) ? {{32{1'b0}},  dm[31: 0]} :
                                (wb_dre_i == 64'hff00_0000_0000_0000 && ~sign) ? {{56{1'b0}},  dm[63:56]} :
                                (wb_dre_i == 64'h00ff_0000_0000_0000 && ~sign) ? {{56{1'b0}},  dm[55:48]} :
                                (wb_dre_i == 64'h0000_ff00_0000_0000 && ~sign) ? {{56{1'b0}},  dm[47:40]} : 
                                (wb_dre_i == 64'h0000_00ff_0000_0000 && ~sign) ? {{56{1'b0}},  dm[39:32]} :
                                (wb_dre_i == 64'h0000_0000_ff00_0000 && ~sign) ? {{56{1'b0}},  dm[31:24]} :
                                (wb_dre_i == 64'h0000_0000_00ff_0000 && ~sign) ? {{56{1'b0}},  dm[23:16]} :
                                (wb_dre_i == 64'h0000_0000_0000_ff00 && ~sign) ? {{56{1'b0}},  dm[15:8 ]} :
                                (wb_dre_i == 64'h0000_0000_0000_00ff && ~sign) ? {{56{1'b0}},  dm[ 7: 0]} : 
                                (wb_dre_i == 64'hffff_0000_0000_0000 && ~sign) ? {{48{1'b0}},  dm[63:48]} :
                                (wb_dre_i == 64'h0000_ffff_0000_0000 && ~sign) ? {{48{1'b0}},  dm[47:32]} : 
                                (wb_dre_i == 64'h0000_0000_ffff_0000 && ~sign) ? {{48{1'b0}},  dm[31:16]} :
                                (wb_dre_i == 64'h0000_0000_0000_ffff && ~sign) ? {{48{1'b0}},  dm[15:0]}  : `ZERO_WORD; */
    wire    [`WORD_BUS] data    =   (wb_dre_i[3] == `TRUE_V)    ?   dm  :
                                    (wb_dre_i[2] == `TRUE_V && sign)    ?   {{32{dm[31]}},dm[31: 0]}    :
                                    (wb_dre_i[2] == `TRUE_V && ~sign)   ?   {{32{1'b0}},dm[31: 0]}      :
                                    (wb_dre_i[1] == `TRUE_V && sign)    ?   {{48{dm[15]}},dm[15: 0]}    :
                                    (wb_dre_i[1] == `TRUE_V && ~sign)   ?   {{48{1'b0}},dm[15: 0]}      :
                                    (wb_dre_i[0] == `TRUE_V && sign)    ?   {{56{dm[7]}},dm[ 7: 0]}    :
                                    (wb_dre_i[0] == `TRUE_V && ~sign)   ?   {{56{1'b0}},dm[ 7: 0]}      :   `ZERO_WORD;

    assign wb_wd_o = (wb_mreg_i == `MREG_ENABLE) ?  data  :   wb_dreg_i;		
    					
endmodule
	