`timescale 1ns / 1ps

/*------------------- 全局参数 -------------------*/
`define RST_ENABLE      1'b1                // 复位信号有效  RST_ENABLE
`define RST_DISABLE     1'b0                // 复位信号无效
`define ZERO_WORD       64'b0               // 64位的数值0
`define ZERO_INST       32'b0               // 32位的数值0
`define WRITE_ENABLE    1'b1                // 使能写
`define WRITE_DISABLE   1'b0                // 禁止写
`define READ_ENABLE     1'b1                // 使能读
`define READ_DISABLE    1'b0                // 禁止读
`define ALUOP_BUS       7 : 0               // 译码阶段的输出aluop_o的宽度
`define SHIFT_ENABLE    1'b1                // 移位指令使能 
`define ALUTYPE_BUS     2 : 0               // 译码阶段的输出alutype_o的宽度  
`define TRUE_V          1'b1                // 逻辑"真"  
`define FALSE_V         1'b0                // 逻辑"假"  
`define JUMP_ENABLE     1'b1                // 预测跳转
`define JUMP_DISABLE    1'b0                // 预测不跳转
`define CHIP_ENABLE     1'b1                // 芯片使能  
`define CHIP_DISABLE    1'b0                // 芯片禁止  
`define WORD_BUS        63: 0               // 64位宽
`define DOUBLE_REG_BUS  127: 0              // 两倍的通用寄存器的数据线宽度
`define RT_ENABLE       1'b1                // rt选择使能
`define SIGNED_EXT      1'b1                // 符号扩展使能
`define IMM_ENABLE      1'b1                // 立即数选择使能
`define UPPER_ENABLE    1'b1                // 立即数移位使能
`define MREG_ENABLE     1'b1                // 写回阶段存储器结果选择信号
`define BSEL_BUS        3 : 0              // 数据存储器字节选择信号宽度
`define DATA_ADDR_BUS   63 : 0              // 数据存储器的地址宽度
/************************转移指令添加 begin*******************************/
`define JUMP_BUS        25: 0               // J型指令字中instr_index字段的宽度
//`define JTSEL_BUS       1 : 0               // 转移地址选择信号的宽度
/*********************** 转移指令添加 end*********************************/
`define PC_INIT         64'h00000000_80000000               // PC初始值
`define INST_INIT       32'b0               //指令初始值

/*------------------- 指令字参数 -------------------*/
`define INST_ADDR_BUS   63: 0               // 指令的地址宽度
`define INST_BUS        31: 0               // 指令的数据宽度

// 操作类型alutype
`define NOP             3'b000
`define ARITH           3'b001
`define LOGIC           3'b010
`define MOVE            3'B011
`define SHIFT           3'b100


//跳转指令alutype
`define JUMP            3'b101

//乘法指令alutype
`define MUL             3'b110

//异常处理指令alutype
`define CSR             3'b111

// 内部操作码aluop


`define RISCV64_ADDI        8'h11
`define RISCV64_SLTI        8'h12
`define RISCV64_SLTIU       8'h13
`define RISCV64_ADD         8'h14
`define RISCV64_SUB         8'h15
`define RISCV64_SLT         8'h16
`define RISCV64_SLTU        8'h17
`define RISCV64_ADDIW		8'h18
`define RISCV64_ADDW		8'h19
`define RISCV64_SUBW		8'h1A

`define RISCV64_ANDI        8'h21
`define RISCV64_ORI         8'h22
`define RISCV64_XORI        8'h23
`define RISCV64_AND         8'h24
`define RISCV64_OR          8'h25
`define RISCV64_XOR         8'h26

`define RISCV64_SLLI        8'h41
`define RISCV64_SRLI        8'h42
`define RISCV64_SRAI        8'h43
`define RISCV64_SLL         8'h44
`define RISCV64_SRL         8'h45
`define RISCV64_SRA         8'h46
`define RISCV64_SLLIW		8'h47
`define RISCV64_SRLIW		8'h48
`define RISCV64_SRAIW		8'h49
`define RISCV64_SLLW		8'h4A
`define RISCV64_SRLW		8'h4B
`define RISCV64_SRAW		8'h4C

`define RISCV64_LUI         8'h51
`define RISCV64_AUIPC       8'h52
`define RISCV64_JAL         8'h53
`define RISCV64_JALR        8'h54

`define RISCV64_MUL         8'h80
`define RISCV64_MULH        8'h81
`define RISCV64_MULHU       8'h82
`define RISCV64_MULHSU      8'h83

`define RISCV64_CSRRW       8'h01
`define RISCV64_CSRRS       8'h02
`define RISCV64_CSRRC       8'h03
`define RISCV64_CSRRWI      8'h04
`define RISCV64_CSRRSI      8'h05
`define RISCV64_CSRRCI      8'h06

`define RISCV64_ECALL       8'hf1
`define RISCV64_MRET        8'hf2


`define STOP            1'b1                //流水线暂停
`define NOSTOP          1'b0                //流水线不暂停
               
`define PRDT_ENABLE     1'b0                //预测跳转
`define PRDT_DISABLE    1'b0                //预测不跳转

/*------------------- 通用寄存器堆参数 -------------------*/
`define REG_BUS         63: 0               // 寄存器数据宽度
`define REG_ADDR_BUS    4 : 0               // 寄存器的地址宽度
`define REG_NUM         32                  // 寄存器数量32个
`define REG_NOP         5'b00000            // 零号寄存器

/*-------------------- btb模块参数 ------------------------*/
`define PC_ADDR_BTB     9:0                 //btb表中输入pc标志的位数
`define BUFFER_SIZE     1024                //btb表的深度
`define BUFFER_ADDR_LEN 63:0                //btb表pc的位数
`define WEAK_JUMP       2'b10               //2bit饱和计数器弱跳转
`define STRONG_JUMP     2'b11               //强跳转
`define WEAK_NOTJUMP    2'b01               //弱不跳转
`define STRONG_NOTJUMP  2'b00               //强不跳转

/*-------------------- axi模块参数 ------------------------*/

`define AXI_ADDR_WIDTH      64
`define AXI_DATA_WIDTH      64
`define AXI_ID_WIDTH        4
`define AXI_USER_WIDTH      1

`define SIZE_B              2'b00
`define SIZE_H              2'b01
`define SIZE_W              2'b10
`define SIZE_D              2'b11

`define REQ_READ            1'b0
`define REQ_WRITE           1'b1

`define INST_READ           1'b0
`define DATA_READ           1'b1

`define RISCV_PRIV_MODE_U   0
`define RISCV_PRIV_MODE_S   1
`define RISCV_PRIV_MODE_M   3



/*-------------------- CSR模块参数 ------------------------*/

`define TRAP_ON             1'b1
`define TRAP_OFF            1'b0

`define INT_ON              1'b1
`define INT_OFF             1'b0

`define MACHINE_MODE        2'b11

`define CSR_NUM             9
`define CSR_ADDR_BUS        3: 0

`define MTIMECMP_ADDR       64'h00000000_02004000
`define MTIME_ADDR          64'h00000000_0200BFF8

`define MTIME_INIT          64'h00000000_02000000
`define MTIMECMP_INIT       64'h00000000_02004000

`define MSTATUS             1
`define MTVEC               2
`define MEPC                3
`define MCAUSE              4
`define MIP                 5
`define MIE                 6
`define MCYCLE              7
`define MSCRATCH            8
`define SSTATUS             9

`define EXCEP_CODE_BUS      2:0          
`define ECALL_CODE          3'b001
`define MRET_CODE           3'b010
`define TIME_TRAP_CODE      3'b100

//cache
`define ICACHE_SIEZ         32
`define CACHE_BUS           127:0

/*-------------------- soc模块参数 ------------------------*/
`define CLINT_ADDR          12'h200
`define UART_ADDR           20'h10000
`define SPI_CTRL_ADDR       20'h10001
`define FLASH_ADDR          4'h3
`define DDR_ADDR            1'b1
