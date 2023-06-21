
`include "defines.v"


module btb(
        input   wire        cpu_rst_n,
        input   wire        cpu_clk_50M,
        
        input   wire[`PC_ADDR_BTB  ]     pc_tag,    			//输入PC的[11：2]位作为检索地址
        output  reg                      btb_jump_ena,          //对外输出信号，预测下条指令是否跳转
        output  reg [`INST_ADDR_BUS]     btb_prepc_o,	        //从btb 中得到的预测PC
        input   wire                     wr_req,                //写请求信号
        input   wire[`PC_ADDR_BTB  ]     wr_pc_tag,             //要写入的分支PC_tag
        input   wire[`INST_ADDR_BUS]     wr_predicted_pc,       //要写入的预测PC
        input   wire                     wr_jump_state	      	//传入当前指令跳转状态
         
    );
    
        reg                     valid     [0:`BUFFER_SIZE-1];
        reg [`BUFFER_ADDR_LEN]  pre_pc    [0:`BUFFER_SIZE-1];
        reg [1:0]               pre_state [0:`BUFFER_SIZE-1];
        
        wire [1:0]  update_jump_state   =   (valid[wr_pc_tag] 	  == `FALSE_V 	  	&& wr_jump_state == `FALSE_V)		?	`WEAK_NOTJUMP	        	:
											(valid[wr_pc_tag] 	  == `FALSE_V     	&& wr_jump_state == `TRUE_V )		?	`WEAK_JUMP		            :
											(pre_state[wr_pc_tag] == `STRONG_JUMP 	&& wr_jump_state == `TRUE_V )		?	`STRONG_JUMP	            :
											(pre_state[wr_pc_tag] != `STRONG_JUMP 	&& wr_jump_state == `TRUE_V )		?	(pre_state[wr_pc_tag] + 1)	:
											(pre_state[wr_pc_tag] == `STRONG_NOTJUMP&& wr_jump_state == `FALSE_V)		?	`STRONG_NOTJUMP	:
											(pre_state[wr_pc_tag] != `STRONG_NOTJUMP&& wr_jump_state == `FALSE_V)		?	(pre_state[wr_pc_tag] - 1)	:	`WEAK_JUMP;

        always @(posedge cpu_clk_50M)   begin
            if (cpu_rst_n == `RST_ENABLE) begin
            end
			else begin
				if (wr_req == `WRITE_ENABLE) 
				    valid       [wr_pc_tag]    <=  `TRUE_V; 
				    pre_pc      [wr_pc_tag]    <=  wr_predicted_pc;
				    pre_state   [wr_pc_tag]    <=  update_jump_state;
		    end
		end 
       
		always @(*)   begin
            if (cpu_rst_n == `RST_ENABLE)	begin
				btb_jump_ena		=	`JUMP_DISABLE;
				btb_prepc_o			=	`ZERO_WORD;
			end
			else if (valid[pc_tag] == `FALSE_V) begin
                btb_jump_ena        =   `JUMP_DISABLE;
				btb_prepc_o			=	`ZERO_WORD;
			end
			else 	begin
				btb_jump_ena	=	pre_state[pc_tag][1];
				btb_prepc_o		=	pre_pc   [pc_tag];
					
			end
		end
		
		
     
 endmodule
 