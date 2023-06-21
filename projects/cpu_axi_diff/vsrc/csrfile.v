
`include "defines.v"

module csrfile(
    input  wire                     cpu_clk_50M,
	input  wire                     cpu_rst_n,
	
	// 写端口
	input  wire  [`REG_ADDR_BUS]	csr_waddr,
	input  wire  [`REG_BUS 	   ]	csr_wdata,
	input  wire 				    csr_we,
	
	// 读端口
	input  wire  [`REG_ADDR_BUS]	csr_raddr,
	output reg   [`REG_BUS 	   ]    csr_rdata,

	//	异常指令添加信号
	output	wire 					global_trap_ena,
	output	wire 					time_trap_ena,
	output	wire [`REG_BUS	   ]	excep_mtvec,
	output	wire [`REG_BUS	   ]	excep_mret_mepc,

	input	wire [`EXCEP_CODE_BUS]	excep_code,

	output wire	 [`REG_BUS]	csrs_o[0:`CSR_NUM]
	
);

    //定义8个64位寄存器
    reg [`REG_BUS] 	csrs[0:`CSR_NUM];

	//将全局中断使能，计时器中断使能，mtvec寄存器值，mepc寄存器值输出
	assign	global_trap_ena = csrs[`MSTATUS][3];
	assign	time_trap_ena	= csrs[`MIE][7];
	assign	excep_mtvec 	= csrs[`MTVEC];
	assign	excep_mret_mepc = csrs[`MEPC];
    
	
	always @(posedge cpu_clk_50M) begin
		if (cpu_rst_n == `RST_ENABLE) begin
			csrs[ 0] <= `ZERO_WORD;
			csrs[ 1] <= `ZERO_WORD;
			csrs[ 2] <= `ZERO_WORD;
			csrs[ 3] <= `ZERO_WORD;
			csrs[ 4] <= `ZERO_WORD;
			csrs[ 5] <= `ZERO_WORD;
			csrs[ 6] <= `ZERO_WORD;
			csrs[ 7] <= `ZERO_WORD;
			csrs[ 8] <= `ZERO_WORD;
		end
		/*********************异常处理相关指令***************************/

		else if(excep_code == `TIME_TRAP_CODE) begin
			csrs[`MCAUSE]		<= 64'h80000000_00000007;
			csrs[`MSTATUS][7]	<= csrs[`MSTATUS][3];
			csrs[`MSTATUS][3]	<= `TRAP_OFF; 
			csrs[`MSTATUS][12:11]<=	`MACHINE_MODE;
		end
		else if(excep_code == `MRET_CODE) begin
			//csrs[`MSTATUS][7]	<=	`TRAP_OFF;
			//csrs[`MSTATUS][3]	<=	`TRAP_ON;
			csrs[`MSTATUS][7]	<=	`TRAP_ON;
			csrs[`MSTATUS][3]	<=	csrs[`MSTATUS][7];
			csrs[`MSTATUS][12:11]<=	2'b00;			//`MACHINE_MODE;
		end
		else if(excep_code == `ECALL_CODE) begin
			csrs[`MCAUSE]		<=	64'h00000000_0000000B;
			csrs[`MSTATUS][7]	<=	csrs[`MSTATUS][3];
			csrs[`MSTATUS][3]	<=	`TRAP_OFF;
			csrs[`MSTATUS][12:11]<=	`MACHINE_MODE;
		end


		/*********************异常处理相关指令***************************/
		else if((csr_we == `WRITE_ENABLE) && (csr_waddr == `MSTATUS)) begin
			csrs[csr_waddr] <= csr_wdata;
			csrs[`SSTATUS][63]	 <= csr_wdata[63];
			csrs[`SSTATUS][16:13]<= csr_wdata[16:13];
			csrs[`MCYCLE]	<= csrs[`MCYCLE] + 1;
		end
		else if((csr_we == `WRITE_ENABLE) && (csr_waddr != `MCYCLE))begin
			csrs[csr_waddr] <= csr_wdata;
			csrs[`MCYCLE]	<= csrs[`MCYCLE] + 1;
		end
		else if((csr_we == `WRITE_ENABLE) && (csr_waddr == `MCYCLE))begin
			csrs[csr_waddr] <= csr_wdata;
		end
		else begin
			csrs[`MCYCLE]	<= csrs[`MCYCLE] + 1;
		end
	end
	
	// 读csr寄存器的读操作 
	// ra1是读地址、wa是写地址、we是写使能、wd是要写入的数据 
	always @(*) begin
		if (cpu_rst_n == `RST_ENABLE) begin
			csr_rdata = `ZERO_WORD;
		end
		else if ((csr_we == `WRITE_ENABLE) && (csr_waddr == csr_raddr)) begin
			csr_rdata = csr_wdata;
		end
		else begin
			csr_rdata = csrs[csr_raddr];
		end	
	end
	
    // to diff

	assign csrs_o[`SSTATUS] = (csr_we && csr_waddr == `MSTATUS)	?	{csr_wdata[63],46'b0,csr_wdata[16:13],13'b0}	:	csrs[`SSTATUS];

	genvar i;
	generate
		for (i = 0; i <= `CSR_NUM-1; i = i + 1) begin
			assign csrs_o[i] = (csr_we && csr_waddr == i && i != 0) ? csr_wdata : csrs[i];
		end
	endgenerate

endmodule
