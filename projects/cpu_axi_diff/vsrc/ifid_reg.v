
`include "defines.v"

module ifid_reg(
    input   wire    cpu_clk_50M,
    input   wire    cpu_rst_n,
    
    //来自取指阶段的信息
    input   wire [`INST_ADDR_BUS]   if_pc,
    input   wire [`INST_BUS     ]   if_inst,
    input   wire [`INST_ADDR_BUS]   if_pc_plus_4,
    
    //送至译码阶段的信息
    output  reg [`INST_ADDR_BUS]    id_pc,
    output  reg [`INST_BUS     ]    id_inst,
    output  reg [`INST_ADDR_BUS]    id_pc_plus_4,

    //流水线暂停信号
    input   wire                    id_stall,
    input   wire                    data_read_stall,

    //axi读指令完成信号
    input   wire                    handshake_done,

    //动态预测添加
    input   wire                    if_jump_ena,
    input   wire [`INST_ADDR_BUS]   if_jump_pc,
    input   wire                    flush,
    
    output  reg                     id_jump_ena,
    output  reg [`INST_ADDR_BUS]    id_jump_pc,

    //异常处理flush冲刷信号添加
    input   wire                    excep_flush
 
);

    always@(posedge cpu_clk_50M) begin
        if(cpu_rst_n == `RST_ENABLE) begin //复位或流水线冲刷的时候将译码阶段的信息清0
            id_pc           <=  `PC_INIT;
            id_inst         <=  `ZERO_INST;
            id_pc_plus_4    <=  `PC_INIT;
            id_jump_ena     <=  `JUMP_DISABLE;
            id_jump_pc      <=  `PC_INIT;
        end
        else if(data_read_stall == `STOP || id_stall == `STOP) begin
            
        end
        else if(handshake_done == `FALSE_V || flush == `TRUE_V || excep_flush == `TRUE_V) begin       //指令未取到时暂停译码
            id_pc           <=  `PC_INIT;
            id_inst         <=  `ZERO_INST;
            id_pc_plus_4    <=  `PC_INIT;
            id_jump_ena     <=  `JUMP_DISABLE;
            id_jump_pc      <=  `PC_INIT;
        end
        else if(id_stall == `NOSTOP && data_read_stall == `NOSTOP) begin
            id_pc           <=  if_pc;                 //将来自取指阶段的信息寄存并送至译码阶段
            id_inst         <=  if_inst;
            id_pc_plus_4    <=  if_pc_plus_4;
            id_jump_ena     <=  if_jump_ena;
            id_jump_pc      <=  if_jump_pc;
        end
    end
endmodule