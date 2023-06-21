
`include "defines.v"

module regfile(
    input  wire                     cpu_clk_50M,
	input  wire                     cpu_rst_n,
	
	// 写端口
	input  wire  [`REG_ADDR_BUS]	wa,
	input  wire  [`REG_BUS 	   ]	wd,
	input  wire 				    we,
	
	// 读端口1
	input  wire  [`REG_ADDR_BUS]	ra1,
	output reg   [`REG_BUS 	   ]    rd1,
	
	// 读端口2 
	input  wire  [`REG_ADDR_BUS]   	ra2,
	output reg   [`REG_BUS 	   ]    rd2,

	output wire	 [`REG_BUS]	regs_o[0:`REG_NUM-1]
	

);

    //定义32个64位寄存器
    reg [`REG_BUS] 	regs[0:`REG_NUM-1];
    
	
	always @(posedge cpu_clk_50M) begin
		if (cpu_rst_n == `RST_ENABLE) begin
			regs[ 0] <= `ZERO_WORD;
			regs[ 1] <= `ZERO_WORD;
			regs[ 2] <= `ZERO_WORD;
			regs[ 3] <= `ZERO_WORD;
			regs[ 4] <= `ZERO_WORD;
			regs[ 5] <= `ZERO_WORD;
			regs[ 6] <= `ZERO_WORD;
			regs[ 7] <= `ZERO_WORD;
			regs[ 8] <= `ZERO_WORD;
			regs[ 9] <= `ZERO_WORD;
			regs[10] <= `ZERO_WORD;
			regs[11] <= `ZERO_WORD;
			regs[12] <= `ZERO_WORD;
			regs[13] <= `ZERO_WORD;
			regs[14] <= `ZERO_WORD;
			regs[15] <= `ZERO_WORD;
			regs[16] <= `ZERO_WORD;
			regs[17] <= `ZERO_WORD;
			regs[18] <= `ZERO_WORD;
			regs[19] <= `ZERO_WORD;
			regs[20] <= `ZERO_WORD;
			regs[21] <= `ZERO_WORD;
			regs[22] <= `ZERO_WORD;
			regs[23] <= `ZERO_WORD;
			regs[24] <= `ZERO_WORD;
			regs[25] <= `ZERO_WORD;
			regs[26] <= `ZERO_WORD;
			regs[27] <= `ZERO_WORD;
			regs[28] <= `ZERO_WORD;
			regs[29] <= `ZERO_WORD;
			regs[30] <= `ZERO_WORD;
			regs[31] <= `ZERO_WORD;
		end
		else begin
			if ((we == `WRITE_ENABLE) && (wa != 5'h0))	
				regs[wa] <= wd;
		end
	end
	
	//读端口1的读操作 
	// ra1是读地址、wa是写地址、we是写使能、wd是要写入的数据 
	always @(*) begin
		if (cpu_rst_n == `RST_ENABLE)
			rd1 = `ZERO_WORD;
		else if (ra1 == `REG_NOP)
			rd1 = `ZERO_WORD;
		else if ((we == `WRITE_ENABLE) && (wa == ra1))
		    rd1 = wd;
		else 
			rd1 = regs[ra1];
	end
	
	//读端口2的读操作 
	// ra2是读地址、wa是写地址、we是写使能、wd是要写入的数据 
	always @(*) begin
		if (cpu_rst_n == `RST_ENABLE)
			rd2 = `ZERO_WORD;
		else if (ra2 == `REG_NOP)
			rd2 = `ZERO_WORD;
		else if ((we == `WRITE_ENABLE) && (wa == ra2))
		    rd2 = wd;
		else 
			rd2 = regs[ra2];
	end

	genvar i;
	generate
		for (i = 0; i < 32; i = i + 1) begin
			assign regs_o[i] = (we & wa == i & i != 0) ? wd : regs[i];
		end
	endgenerate

endmodule
