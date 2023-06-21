
/*------------------- 全局参数 -------------------*/
`define YSYX219999_RST_ENABLE      1'b1                // 复位信号有效  YSYX219999_RST_ENABLE
`define YSYX219999_RST_DISABLE     1'b0                // 复位信号无效
`define YSYX219999_ZERO_WORD       64'b0               // 64位的数值0
`define YSYX219999_ZERO_INST       32'b0               // 32位的数值0
`define YSYX219999_WRITE_ENABLE    1'b1                // 使能写
`define YSYX219999_WRITE_DISABLE   1'b0                // 禁止写
`define YSYX219999_READ_ENABLE     1'b1                // 使能读
`define YSYX219999_READ_DISABLE    1'b0                // 禁止读
`define YSYX219999_ALUOP_BUS       7 : 0               // 译码阶段的输出aluop_o的宽度
`define YSYX219999_YSYX219999_SHIFT_ENABLE    1'b1                // 移位指令使能 
`define YSYX219999_ALUTYPE_BUS     2 : 0               // 译码阶段的输出alutype_o的宽度  
`define YSYX219999_TRUE_V          1'b1                // 逻辑"真"  
`define YSYX219999_FALSE_V         1'b0                // 逻辑"假"  
`define YSYX219999_JUMP_ENABLE     1'b1                // 预测跳转
`define YSYX219999_JUMP_DISABLE    1'b0                // 预测不跳转
`define YSYX219999_CHIP_ENABLE     1'b1                // 芯片使能  
`define YSYX219999_CHIP_DISABLE    1'b0                // 芯片禁止  
`define YSYX219999_WORD_BUS        63: 0               // 64位宽
`define YSYX219999_DOUBLE_REG_BUS  127: 0              // 两倍的通用寄存器的数据线宽度
`define YSYX219999_RT_ENABLE       1'b1                // rt选择使能
`define YSYX219999_SIGNED_EXT      1'b1                // 符号扩展使能
`define YSYX219999_IMM_ENABLE      1'b1                // 立即数选择使能
`define YSYX219999_UPPER_ENABLE    1'b1                // 立即数移位使能
`define YSYX219999_MREG_ENABLE     1'b1                // 写回阶段存储器结果选择信号
`define YSYX219999_BSEL_BUS        3 : 0              // 数据存储器字节选择信号宽度
`define YSYX219999_DATA_ADDR_BUS   63 : 0              // 数据存储器的地址宽度
/************************转移指令添加 begin*******************************/
`define YSYX219999_JUMP_BUS        25: 0               // J型指令字中instr_index字段的宽度
//`define JTSEL_BUS       1 : 0               // 转移地址选择信号的宽度
/*********************** 转移指令添加 end*********************************/
`define YSYX219999_PC_INIT         64'h00000000_30000000               // PC初始值
`define YSYX219999_INST_INIT       32'b0               //指令初始值

/*------------------- 指令字参数 -------------------*/
`define YSYX219999_INST_ADDR_BUS   63: 0               // 指令的地址宽度
`define YSYX219999_INST_BUS        31: 0               // 指令的数据宽度

// 操作类型alutype
`define YSYX219999_NOP             3'b000
`define YSYX219999_ARITH           3'b001
`define YSYX219999_LOGIC           3'b010
`define YSYX219999_MOVE            3'B011
`define YSYX219999_SHIFT           3'b100


//跳转指令alutype
`define JUMP            3'b101

//乘法指令alutype
`define MUL             3'b110

//异常处理指令alutype
`define CSR             3'b111

// 内部操作码aluop


`define YSYX219999_RISCV64_ADDI        8'h11
`define YSYX219999_RISCV64_SLTI        8'h12
`define YSYX219999_RISCV64_SLTIU       8'h13
`define YSYX219999_RISCV64_ADD         8'h14
`define YSYX219999_RISCV64_SUB         8'h15
`define YSYX219999_RISCV64_SLT         8'h16
`define YSYX219999_RISCV64_SLTU        8'h17
`define YSYX219999_RISCV64_ADDIW		8'h18
`define YSYX219999_RISCV64_ADDW		8'h19
`define YSYX219999_RISCV64_SUBW		8'h1A

`define YSYX219999_RISCV64_ANDI        8'h21
`define YSYX219999_RISCV64_ORI         8'h22
`define YSYX219999_RISCV64_XORI        8'h23
`define YSYX219999_RISCV64_AND         8'h24
`define YSYX219999_RISCV64_OR          8'h25
`define YSYX219999_RISCV64_XOR         8'h26

`define YSYX219999_RISCV64_SLLI        8'h41
`define YSYX219999_RISCV64_SRLI        8'h42
`define YSYX219999_RISCV64_SRAI        8'h43
`define YSYX219999_RISCV64_SLL         8'h44
`define YSYX219999_RISCV64_SRL         8'h45
`define YSYX219999_RISCV64_SRA         8'h46
`define YSYX219999_RISCV64_SLLIW		8'h47
`define YSYX219999_RISCV64_SRLIW		8'h48
`define YSYX219999_RISCV64_SRAIW		8'h49
`define YSYX219999_RISCV64_SLLW		8'h4A
`define YSYX219999_RISCV64_SRLW		8'h4B
`define YSYX219999_RISCV64_SRAW		8'h4C

`define YSYX219999_RISCV64_LUI         8'h51
`define YSYX219999_RISCV64_AUIPC       8'h52
`define YSYX219999_RISCV64_JAL         8'h53
`define YSYX219999_RISCV64_JALR        8'h54

`define YSYX219999_RISCV64_MUL         8'h80
`define YSYX219999_RISCV64_MULH        8'h81
`define YSYX219999_RISCV64_MULHU       8'h82
`define YSYX219999_RISCV64_MULHSU      8'h83

`define YSYX219999_RISCV64_CSRRW       8'h01
`define YSYX219999_RISCV64_CSRRS       8'h02
`define YSYX219999_RISCV64_CSRRC       8'h03
`define YSYX219999_RISCV64_CSRRWI      8'h04
`define YSYX219999_RISCV64_CSRRSI      8'h05
`define YSYX219999_RISCV64_CSRRCI      8'h06

`define YSYX219999_RISCV64_ECALL       8'hf1
`define YSYX219999_RISCV64_MRET        8'hf2


`define STOP            1'b1                //流水线暂停
`define NOSTOP          1'b0                //流水线不暂停
               
`define PRDT_ENABLE     1'b0                //预测跳转
`define PRDT_DISABLE    1'b0                //预测不跳转

/*------------------- 通用寄存器堆参数 -------------------*/
`define REG_BUS         63: 0               // 寄存器数据宽度
`define REG_ADDR_BUS    4 : 0               // 寄存器的地址宽度
`define REG_NUM         32                  // 寄存器数量32个
`define REG_YSYX219999_NOP         5'b00000            // 零号寄存器

/*-------------------- btb模块参数 ------------------------*/
`define PC_ADDR_BTB     9:0                 //btb表中输入pc标志的位数
`define BUFFER_SIZE     1024                //btb表的深度
`define BUFFER_ADDR_LEN 63:0                //btb表pc的位数
`define WEAK_JUMP       2'b10               //2bit饱和计数器弱跳转
`define STRONG_JUMP     2'b11               //强跳转
`define WEAK_NOTJUMP    2'b01               //弱不跳转
`define STRONG_NOTJUMP  2'b00               //强不跳转

/*-------------------- axi模块参数 ------------------------*/

`define AXI_ADDR_WIDTH      32
`define AXI_DATA_WIDTH      64
`define AXI_ID_WIDTH        4

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

/*-------------------- soc模块参数 ------------------------*/
`define CLINT_ADDR          12'h200
`define UART_ADDR           20'h10000
`define SPI_CTRL_ADDR       20'h10001
`define FLASH_ADDR          4'h3
`define DDR_ADDR            1'b1

// Burst types
`define AXI_BURST_TYPE_FIXED                                2'b00
`define AXI_BURST_TYPE_INCR                                 2'b01
`define AXI_BURST_TYPE_WRAP                                 2'b10
// Access permissions
`define AXI_PROT_UNPRIVILEGED_ACCESS                        3'b000
`define AXI_PROT_PRIVILEGED_ACCESS                          3'b001
`define AXI_PROT_SECURE_ACCESS                              3'b000
`define AXI_PROT_NON_SECURE_ACCESS                          3'b010
`define AXI_PROT_DATA_ACCESS                                3'b000
`define AXI_PROT_INSTRUCTION_ACCESS                         3'b100
// Memory types (AR)
`define AXI_ARCACHE_DEVICE_NON_BUFFERABLE                   4'b0000
`define AXI_ARCACHE_DEVICE_BUFFERABLE                       4'b0001
`define AXI_ARCACHE_NORMAL_NON_CACHEABLE_NON_BUFFERABLE     4'b0010
`define AXI_ARCACHE_NORMAL_NON_CACHEABLE_BUFFERABLE         4'b0011
`define AXI_ARCACHE_WRITE_THROUGH_NO_ALLOCATE               4'b1010
`define AXI_ARCACHE_WRITE_THROUGH_READ_ALLOCATE             4'b1110
`define AXI_ARCACHE_WRITE_THROUGH_WRITE_ALLOCATE            4'b1010
`define AXI_ARCACHE_WRITE_THROUGH_READ_AND_WRITE_ALLOCATE   4'b1110
`define AXI_ARCACHE_WRITE_BACK_NO_ALLOCATE                  4'b1011
`define AXI_ARCACHE_WRITE_BACK_READ_ALLOCATE                4'b1111
`define AXI_ARCACHE_WRITE_BACK_WRITE_ALLOCATE               4'b1011
`define AXI_ARCACHE_WRITE_BACK_READ_AND_WRITE_ALLOCATE      4'b1111
// Memory types (AW)
`define AXI_AWCACHE_DEVICE_NON_BUFFERABLE                   4'b0000
`define AXI_AWCACHE_DEVICE_BUFFERABLE                       4'b0001
`define AXI_AWCACHE_NORMAL_NON_CACHEABLE_NON_BUFFERABLE     4'b0010
`define AXI_AWCACHE_NORMAL_NON_CACHEABLE_BUFFERABLE         4'b0011
`define AXI_AWCACHE_WRITE_THROUGH_NO_ALLOCATE               4'b0110
`define AXI_AWCACHE_WRITE_THROUGH_READ_ALLOCATE             4'b0110
`define AXI_AWCACHE_WRITE_THROUGH_WRITE_ALLOCATE            4'b1110
`define AXI_AWCACHE_WRITE_THROUGH_READ_AND_WRITE_ALLOCATE   4'b1110
`define AXI_AWCACHE_WRITE_BACK_NO_ALLOCATE                  4'b0111
`define AXI_AWCACHE_WRITE_BACK_READ_ALLOCATE                4'b0111
`define AXI_AWCACHE_WRITE_BACK_WRITE_ALLOCATE               4'b1111
`define AXI_AWCACHE_WRITE_BACK_READ_AND_WRITE_ALLOCATE      4'b1111

`define AXI_SIZE_BYTES_1                                    3'b000
`define AXI_SIZE_BYTES_2                                    3'b001
`define AXI_SIZE_BYTES_4                                    3'b010
`define AXI_SIZE_BYTES_8                                    3'b011
`define AXI_SIZE_BYTES_16                                   3'b100
`define AXI_SIZE_BYTES_32                                   3'b101
`define AXI_SIZE_BYTES_64                                   3'b110
`define AXI_SIZE_BYTES_128                                  3'b111


module ysyx_219999_axi_rw # (
    parameter RW_DATA_WIDTH     = 64,
    parameter AXI_DATA_WIDTH    = 64,
    parameter AXI_ADDR_WIDTH    = 32,
    parameter AXI_ID_WIDTH      = 4
)(
    input                               clock,
    input                               reset,

	input                               r_valid_i,
	output                              r_ready_o,
    input                               r_req_i,
    output wire [RW_DATA_WIDTH-1:0]      data_read_o,
    input  [AXI_ADDR_WIDTH-1:0]         r_addr_i,
    input  [1:0]                        r_size_i,
    output [1:0]                        r_resp_o,

    input                               w_valid_i,
    input                               w_req_i,
    input  [RW_DATA_WIDTH-1:0]          data_write_i,
    input  [RW_DATA_WIDTH-1:0]          w_addr_i,
    input   [ 7: 0]                     wmask,
    input                               w_ena,
    output                              write_is_down,

    // Advanced eXtensible Interface
    input                               axi_aw_ready_i,
    output                              axi_aw_valid_o,
    output [AXI_ADDR_WIDTH-1:0]         axi_aw_addr_o,
    output [AXI_ID_WIDTH-1:0]           axi_aw_id_o,
    output [7:0]                        axi_aw_len_o,
    output [2:0]                        axi_aw_size_o,
    output [1:0]                        axi_aw_burst_o,


    input                               axi_w_ready_i,
    output                              axi_w_valid_o,
    output [AXI_DATA_WIDTH-1:0]         axi_w_data_o,
    output [AXI_DATA_WIDTH/8-1:0]       axi_w_strb_o,
    output                              axi_w_last_o,
    
    output                              axi_b_ready_o,
    input                               axi_b_valid_i,
    input  [1:0]                        axi_b_resp_i,
    input  [AXI_ID_WIDTH-1:0]           axi_b_id_i,

    input                               axi_ar_ready_i,
    output                              axi_ar_valid_o,
    output [AXI_ADDR_WIDTH-1:0]         axi_ar_addr_o,
    output [AXI_ID_WIDTH-1:0]           axi_ar_id_o,
    output [7:0]                        axi_ar_len_o,
    output [2:0]                        axi_ar_size_o,
    output [1:0]                        axi_ar_burst_o,
    
    output                              axi_r_ready_o,
    input                               axi_r_valid_i,
    input  [1:0]                        axi_r_resp_i,
    input  [AXI_DATA_WIDTH-1:0]         axi_r_data_i,
    input                               axi_r_last_i,
    input  [AXI_ID_WIDTH-1:0]           axi_r_id_i

);

    wire apb_addr_rena;
    wire apb_addr_wena;
    wire uart_addr_rena;
    wire uart_addr_wena;
    assign uart_addr_rena= (r_addr_i[31:12] == `UART_ADDR);
    assign uart_addr_wena = w_addr_i[31:12] == `UART_ADDR;
    assign apb_addr_rena = ( r_addr_i[31:12] == `SPI_CTRL_ADDR || r_addr_i[31:28] == `FLASH_ADDR)   ?   `YSYX219999_TRUE_V :   `YSYX219999_FALSE_V;
    assign apb_addr_wena = ( w_addr_i[31:12] == `SPI_CTRL_ADDR || w_addr_i[31:28] == `FLASH_ADDR)   ?   `YSYX219999_TRUE_V :   `YSYX219999_FALSE_V;


    wire w_trans    = w_req_i == `REQ_WRITE;
    wire r_trans    = r_req_i == `REQ_READ;
    wire w_valid    = w_valid_i & w_trans;
    wire r_valid    = r_valid_i & r_trans;
    wire test_warning;

    localparam ALIGNED_WIDTH = $clog2(AXI_DATA_WIDTH / 8);
    localparam OFFSET_WIDTH  = $clog2(AXI_DATA_WIDTH);
    localparam AXI_SIZE      = $clog2(AXI_DATA_WIDTH / 8);
    localparam MASK_WIDTH    = AXI_DATA_WIDTH * 2;
    localparam TRANS_LEN     = RW_DATA_WIDTH / AXI_DATA_WIDTH;
    localparam BLOCK_TRANS   = TRANS_LEN > 1 ? 1'b1 : 1'b0;

    wire aligned            = BLOCK_TRANS | r_addr_i[ALIGNED_WIDTH-1:0] == 0;
    wire size_b             = r_size_i == `SIZE_B;
    wire size_h             = r_size_i == `SIZE_H;
    wire size_w             = r_size_i == `SIZE_W;
    wire size_d             = r_size_i == `SIZE_D;
    wire [3:0] addr_op1     = {{4-ALIGNED_WIDTH{1'b0}}, r_addr_i[ALIGNED_WIDTH-1:0]};
    wire [3:0] addr_op2     = ({4{size_b}} & {4'b0})
                                | ({4{size_h}} & {4'b1})
                                | ({4{size_w}} & {4'b11})
                                | ({4{size_d}} & {4'b111})
                                ;
    wire [3:0] addr_end     = addr_op1 + addr_op2;
    wire overstep           = addr_end[3:ALIGNED_WIDTH] != 0;

    //  for reYSYX219999_MOVE warning sign
    
    assign test_warning = (w_addr_i == `YSYX219999_ZERO_WORD && axi_b_id_i == 4'b0 && axi_r_id_i == 4'b0 && addr_end == 4'b0)  ?   `YSYX219999_FALSE_V    :   `YSYX219999_FALSE_V;


    // handshake
    wire aw_hs      = axi_aw_ready_i & axi_aw_valid_o;
    wire w_hs       = axi_w_ready_i  & axi_w_valid_o;
    wire b_hs       = axi_b_ready_o  & axi_b_valid_i;
    wire ar_hs      = axi_ar_ready_i & axi_ar_valid_o;
    wire r_hs       = axi_r_ready_o  & axi_r_valid_i;

    assign write_is_down = b_hs;
    

    wire w_done     =(test_warning == `YSYX219999_FALSE_V) ? w_hs & axi_w_last_o   :  w_hs & axi_w_last_o;
    wire r_done     = r_hs & axi_r_last_i;
    wire trans_done = w_trans ? b_hs : r_done;

    
    
    // ------------------State Machine------------------
    localparam [1:0] W_STATE_IDLE = 2'b00, W_STATE_ADDR = 2'b01, W_STATE_WRITE = 2'b10, W_STATE_RESP = 2'b11;
    localparam [1:0] R_STATE_IDLE = 2'b00, R_STATE_ADDR = 2'b01, R_STATE_READ  = 2'b10;

    reg [1:0] w_state, r_state;
    wire w_state_idle = w_state == W_STATE_IDLE, w_state_addr = w_state == W_STATE_ADDR, w_state_write = w_state == W_STATE_WRITE, w_state_resp = w_state == W_STATE_RESP;
    wire r_state_idle = r_state == R_STATE_IDLE, r_state_addr = r_state == R_STATE_ADDR, r_state_read  = r_state == R_STATE_READ;

    // Wirte State Machine
    always @(posedge clock) begin
        if (reset) begin
            w_state <= W_STATE_IDLE;
        end
        else begin
            if (w_valid) begin
                case (w_state)
                    W_STATE_IDLE:               w_state <= W_STATE_ADDR;
                    W_STATE_ADDR:  if (aw_hs)   w_state <= W_STATE_WRITE;
                    W_STATE_WRITE: if (w_done)  w_state <= W_STATE_RESP;
                    W_STATE_RESP:  if (b_hs)    w_state <= W_STATE_IDLE;
                endcase
            end
        end
    end

    // Read State Machine
    always @(posedge clock) begin
        if (reset) begin
            r_state <= R_STATE_IDLE;
        end
        else begin
            if (r_valid) begin
                case (r_state)
                    R_STATE_IDLE:               r_state <= R_STATE_ADDR;
                    R_STATE_ADDR: if (ar_hs)    r_state <= R_STATE_READ;
                    R_STATE_READ: if (r_done)   r_state <= R_STATE_IDLE;
                    default:;
                endcase
            end
        end
    end


    // ------------------Number of transmission------------------
    wire [7:0] axi_len      = aligned ? TRANS_LEN - 1 : {{7{1'b0}}, overstep};
    reg [7:0] len;
    wire len_reset      = reset | (w_trans & w_state_idle) | (r_trans & r_state_idle);
    wire len_incr_en    = (len != axi_len) & (w_hs | r_hs);
    always @(posedge clock) begin
        if (len_reset) begin
            len <= 0;
        end
        else if (len_incr_en) begin
            len <= len + 1;
        end
    end


    // ------------------Process Data------------------
    

    
    wire [2:0] axi_size     = AXI_SIZE[2:0];
    
    wire [AXI_ADDR_WIDTH-1:0] axi_addr          = {r_addr_i[AXI_ADDR_WIDTH-1:ALIGNED_WIDTH], {ALIGNED_WIDTH{1'b0}}};
    wire [OFFSET_WIDTH-1:0] aligned_offset_l    = {{OFFSET_WIDTH-ALIGNED_WIDTH{1'b0}}, {r_addr_i[ALIGNED_WIDTH-1:0]}} << 3;

    wire [OFFSET_WIDTH  :0] aligned_offset_h_tmp= AXI_DATA_WIDTH - aligned_offset_l;
    wire [OFFSET_WIDTH-1:0] aligned_offset_h    = aligned_offset_h_tmp[5:0];

    //wire [6:0]  aligned_test                    = {7'b1000000 - {1'b0,aligned_offset_l}};
    //wire [OFFSET_WIDTH-1:0] aligned_offset_h    = aligned_test[5:0];
    //wire [OFFSET_WIDTH-1:0] aligned_offset_h    = {7'b1000000 - {1'b0,aligned_offset_l}}[5:0];
    //wire [OFFSET_WIDTH-1:0] aligned_offset_h    = {7'b1000000 - {1'b0,aligned_offset_l}}[5:0];
    wire [MASK_WIDTH-1:0]   mask                  = (({MASK_WIDTH{size_b}} & {{MASK_WIDTH-8{1'b0}}, 8'hff})
                                                    | ({MASK_WIDTH{size_h}} & {{MASK_WIDTH-16{1'b0}}, 16'hffff})
                                                    | ({MASK_WIDTH{size_w}} & {{MASK_WIDTH-32{1'b0}}, 32'hffffffff})
                                                    | ({MASK_WIDTH{size_d}} & {{MASK_WIDTH-64{1'b0}}, 64'hffffffff_ffffffff})
                                                    ) << aligned_offset_l;
    wire [AXI_DATA_WIDTH-1:0] mask_l            = mask[AXI_DATA_WIDTH-1:0];
    wire [AXI_DATA_WIDTH-1:0] mask_h            = mask[MASK_WIDTH-1:AXI_DATA_WIDTH];

    wire [AXI_ID_WIDTH-1:0] axi_id              = {AXI_ID_WIDTH{1'b0}};

    reg r_ready;
    wire r_ready_nxt = r_done;
    wire r_ready_en      = r_done | r_ready;
    always @(posedge clock) begin
        if (reset) begin
            r_ready <= 0;
        end
        else if (r_ready_en) begin
            r_ready <= r_ready_nxt;
        end
    end
    assign r_ready_o     = r_ready;

    reg [1:0] rw_resp;
    wire[1:0] rw_resp_nxt = w_trans ? axi_b_resp_i : axi_r_resp_i;
    wire resp_en = trans_done;
    always @(posedge clock) begin
        if (reset) begin
            rw_resp <= 0;
        end
        else if (resp_en) begin
            rw_resp <= rw_resp_nxt;
        end
    end
    assign r_resp_o      = rw_resp;


    // ------------------Write Transaction------------------
    reg    aw_valid;

    always @(posedge clock) begin
        if(reset) begin
            aw_valid <= `YSYX219999_FALSE_V;
        end
        else if(w_ena == `YSYX219999_TRUE_V && aw_hs == `YSYX219999_FALSE_V) begin
            aw_valid <= `YSYX219999_TRUE_V;
        end
        else if(aw_hs == `YSYX219999_TRUE_V) begin
            aw_valid <= `YSYX219999_FALSE_V;
        end
    end 


    assign axi_aw_valid_o   = (aw_valid == `YSYX219999_TRUE_V && w_ena) ? w_state_addr : `YSYX219999_FALSE_V;
    assign axi_aw_addr_o    =   (uart_addr_wena == `YSYX219999_TRUE_V) ?   w_addr_i[31: 0]    :   
                                (apb_addr_wena == `YSYX219999_TRUE_V)  ?   {w_addr_i[31: 2],2'b0}  :   {w_addr_i[AXI_ADDR_WIDTH-1:ALIGNED_WIDTH], {ALIGNED_WIDTH{1'b0}}};
    assign axi_aw_id_o      = axi_id;
    assign axi_aw_len_o     = axi_len;
    assign axi_aw_size_o    =   (uart_addr_wena == `YSYX219999_TRUE_V)   ?   3'b000    :  
                                (apb_addr_wena == `YSYX219999_TRUE_V)    ?   3'b010  :   axi_size;
    assign axi_aw_burst_o   = `AXI_BURST_TYPE_INCR;

    //  Write data channel signals
    assign axi_w_valid_o    = w_state_write;
    assign axi_w_data_o     =   (uart_addr_wena == `YSYX219999_TRUE_V || apb_addr_wena == `YSYX219999_TRUE_V && r_addr_i[2] == 1'b0) ?   {32'b0,data_write_i[31: 0]} :
                                (uart_addr_wena == `YSYX219999_TRUE_V || apb_addr_wena == `YSYX219999_TRUE_V && r_addr_i[2] == 1'b1) ?   {32'b0,data_write_i[63:32]} :  data_write_i;
    assign axi_w_strb_o     =   (uart_addr_wena == `YSYX219999_TRUE_V || apb_addr_wena == `YSYX219999_TRUE_V && r_addr_i[2] == 1'b0) ?   wmask    :
                                (uart_addr_wena == `YSYX219999_TRUE_V || apb_addr_wena == `YSYX219999_TRUE_V && r_addr_i[2] == 1'b1) ?   {4'b0,wmask[ 7: 4]}  :
                                wmask;
    assign axi_w_last_o     = 1'b1;

    //  write en channel signals
    assign axi_b_ready_o    = w_state_resp;






    // ------------------Read Transaction------------------

    // Read address channel signals
    assign axi_ar_valid_o   = (r_valid_i) ? r_state_addr : `YSYX219999_FALSE_V;
    assign axi_ar_addr_o    =   (uart_addr_rena == `YSYX219999_TRUE_V)   ?   r_addr_i    :
                                (apb_addr_rena == `YSYX219999_TRUE_V)    ?   {r_addr_i[31:2],2'b0}   :   axi_addr;
    assign axi_ar_id_o      = axi_id;
    assign axi_ar_len_o     = axi_len;
    assign axi_ar_size_o    =   (uart_addr_rena == `YSYX219999_TRUE_V)   ?   3'b000  :
                                (apb_addr_rena == `YSYX219999_TRUE_V)    ?   3'b010  :   axi_size;
    assign axi_ar_burst_o   = `AXI_BURST_TYPE_INCR;

    // Read data channel signals
    assign axi_r_ready_o    = r_state_read;

    wire [63: 0] r_data_i_apb_select =  (size_w)    ?   {32'b0,axi_r_data_i[31: 0]} :
                                        (size_h == `YSYX219999_TRUE_V & r_addr_i[1] == 1'b0)   ?   {48'b0,axi_r_data_i[15: 0]} :
                                        (size_h == `YSYX219999_TRUE_V & r_addr_i[1] == 1'b1)   ?   {48'b0,axi_r_data_i[31:16]} :
                                        (size_b == `YSYX219999_TRUE_V & r_addr_i[1:0] == 2'b0)     ?   {56'b0,axi_r_data_i[ 7: 0]} :
                                        (size_b == `YSYX219999_TRUE_V & r_addr_i[1:0] == 2'b01)    ?   {56'b0,axi_r_data_i[15: 8]} :
                                        (size_b == `YSYX219999_TRUE_V & r_addr_i[1:0] == 2'b10)    ?   {56'b0,axi_r_data_i[23:16]} :
                                        (size_b == `YSYX219999_TRUE_V & r_addr_i[1:0] == 2'b11)    ?   {56'b0,axi_r_data_i[31:24]} :   `YSYX219999_ZERO_WORD;

    reg [63: 0]data_read_o_apb;

    always @(posedge clock) begin
        if(reset) begin
            data_read_o_apb <= 0;
        end
        else if (axi_r_ready_o & axi_r_valid_i) begin
            data_read_o_apb <= r_data_i_apb_select;
        end
    end

    reg [63: 0]data_read_o_ddr;

    wire [AXI_DATA_WIDTH-1:0] axi_r_data_l  = (axi_r_data_i & mask_l) >> aligned_offset_l;
    wire [AXI_DATA_WIDTH-1:0] axi_r_data_h  = (axi_r_data_i & mask_h) << aligned_offset_h;

    genvar i;

    generate
        for (i = 0; i < TRANS_LEN; i = i + 1) begin :loop1
            always @(posedge clock) begin
                if (reset) begin
                    data_read_o_ddr[i*AXI_DATA_WIDTH+:AXI_DATA_WIDTH] <= 0;
                end
                else if (axi_r_ready_o & axi_r_valid_i) begin
                    if (~aligned & overstep) begin
                        if (len[0]) begin
                            data_read_o_ddr[AXI_DATA_WIDTH-1:0] <= data_read_o_ddr[AXI_DATA_WIDTH-1:0] | axi_r_data_h;
                        end
                        else begin
                            data_read_o_ddr[AXI_DATA_WIDTH-1:0] <= axi_r_data_l;
                        end
                    end
                    else if (len == i) begin
                        data_read_o_ddr[i*AXI_DATA_WIDTH+:AXI_DATA_WIDTH] <= axi_r_data_l;
                    end
                end
            end
        end
    endgenerate

    assign data_read_o = (apb_addr_rena == `YSYX219999_TRUE_V) ?   data_read_o_apb : data_read_o_ddr;

endmodule



module ysyx_219999_btb(
        input   wire        cpu_rst_n,
        input   wire        cpu_clk_50M,
        
        input   wire[`PC_ADDR_BTB  ]     pc_tag,    			//输入PC的【11：2】位作为检索地址
        output  reg                      btb_jump_ena,          //对外输出信号，预测下条指令是否跳转
        output  reg [`YSYX219999_INST_ADDR_BUS]     btb_prepc_o,	        //从btb 中得到的预测PC
        input   wire                     wr_req,                //写请求信号
        input   wire[`PC_ADDR_BTB  ]     wr_pc_tag,             //要写入的分支PC_tag
        input   wire[`YSYX219999_INST_ADDR_BUS]     wr_predicted_pc,       //要写入的预测PC
        input   wire                     wr_jump_state	      	//传入当前指令跳转状态
         
    );
    
        reg                     valid     [0:`BUFFER_SIZE-1];
        reg [`BUFFER_ADDR_LEN]  pre_pc    [0:`BUFFER_SIZE-1];
        reg [1:0]               pre_state [0:`BUFFER_SIZE-1];
        
        wire [1:0]  update_jump_state   =   (valid[wr_pc_tag] 	  == `YSYX219999_FALSE_V 	  	&& wr_jump_state == `YSYX219999_FALSE_V)		?	`WEAK_NOTJUMP	        	:
											(valid[wr_pc_tag] 	  == `YSYX219999_FALSE_V     	&& wr_jump_state == `YSYX219999_TRUE_V )		?	`WEAK_JUMP		            :
											(pre_state[wr_pc_tag] == `STRONG_JUMP 	&& wr_jump_state == `YSYX219999_TRUE_V )		?	`STRONG_JUMP	            :
											(pre_state[wr_pc_tag] != `STRONG_JUMP 	&& wr_jump_state == `YSYX219999_TRUE_V )		?	(pre_state[wr_pc_tag] + 1)	:
											(pre_state[wr_pc_tag] == `STRONG_NOTJUMP&& wr_jump_state == `YSYX219999_FALSE_V)		?	`STRONG_NOTJUMP	:
											(pre_state[wr_pc_tag] != `STRONG_NOTJUMP&& wr_jump_state == `YSYX219999_FALSE_V)		?	(pre_state[wr_pc_tag] - 1)	:	`WEAK_JUMP;

        always @(posedge cpu_clk_50M)   begin
            if (cpu_rst_n == `YSYX219999_RST_ENABLE) begin
            end
			else begin
				if (wr_req == `YSYX219999_WRITE_ENABLE) 
				    valid       [wr_pc_tag]    <=  `YSYX219999_TRUE_V; 
				    pre_pc      [wr_pc_tag]    <=  wr_predicted_pc;
				    pre_state   [wr_pc_tag]    <=  update_jump_state;
		    end
		end 
       
		always @(*)   begin
            if (cpu_rst_n == `YSYX219999_RST_ENABLE)	begin
				btb_jump_ena		=	`YSYX219999_JUMP_DISABLE;
				btb_prepc_o			=	`YSYX219999_ZERO_WORD;
			end
			else if (valid[pc_tag] == `YSYX219999_FALSE_V) begin
                btb_jump_ena        =   `YSYX219999_JUMP_DISABLE;
				btb_prepc_o			=	`YSYX219999_ZERO_WORD;
			end
			else 	begin
				btb_jump_ena	=	pre_state[pc_tag][1];
				btb_prepc_o		=	pre_pc   [pc_tag];
					
			end
		end
		
		
     
 endmodule
 

module ysyx_219999_clint(
    input   wire                cpu_clk_50M,
    input   wire                cpu_rst_n,

    input   wire                read_en,
    input   wire[`YSYX219999_WORD_BUS]     read_addr,
    output  reg [`YSYX219999_WORD_BUS]     data_read,

    input   wire                write_en,
    input   wire[`YSYX219999_WORD_BUS]     write_addr,
    input   wire[`YSYX219999_WORD_BUS]     data_write,

    output  wire                interr,
    input   wire                interr_isdone
);

    reg[`REG_BUS]     mtime;
    reg[`REG_BUS]     mtimecmp;
    reg               interr_ena;

    assign  interr = (interr_ena == `YSYX219999_TRUE_V && mtime >= mtimecmp)   ?    `INT_ON :  `INT_OFF;

    always @(posedge cpu_clk_50M) begin
        if(cpu_rst_n == `YSYX219999_RST_ENABLE) begin
            interr_ena  <= `YSYX219999_TRUE_V;
        end
        else if(interr_isdone == `YSYX219999_TRUE_V) begin
            interr_ena  <= `YSYX219999_FALSE_V;
        end
        else if(mtime < mtimecmp /*&& interr_ena == `YSYX219999_FALSE_V*/) begin
            interr_ena  <= `YSYX219999_TRUE_V;
        end
    end

    always @(posedge cpu_clk_50M) begin
        if(cpu_rst_n == `YSYX219999_RST_ENABLE) begin
            mtime   <=  `MTIME_INIT;
        end
        else if(write_en == `YSYX219999_TRUE_V && write_addr == `MTIME_ADDR) begin
            mtime   <=  data_write;
        end
        else begin
            mtime   <=  mtime + 1;
        end
    end

    always @(posedge cpu_clk_50M) begin
        if(cpu_rst_n == `YSYX219999_RST_ENABLE) begin
            mtimecmp    <=  `MTIMECMP_INIT;
        end
        else if(write_en == `YSYX219999_TRUE_V && write_addr == `MTIMECMP_ADDR) begin
            mtimecmp    <=  data_write;
        end
    end

    always @(posedge cpu_clk_50M) begin
        if(cpu_rst_n == `YSYX219999_RST_ENABLE) begin
            data_read   <=   `YSYX219999_ZERO_WORD;
        end
        else if(read_en == `YSYX219999_TRUE_V && read_addr == `MTIME_ADDR) begin
            data_read   <=   mtime;
        end
        else if(read_en == `YSYX219999_TRUE_V && read_addr == `MTIMECMP_ADDR) begin
            data_read   <=   mtimecmp;
        end
        else begin
            data_read   <=   `YSYX219999_ZERO_WORD;
        end
    end

endmodule

module ysyx_219999_csrfile(
    input  wire                     cpu_clk_50M,
	input  wire                     cpu_rst_n,
	
	// 写端口
	input  wire  [`CSR_ADDR_BUS]	csr_waddr,
	input  wire  [`REG_BUS 	   ]	csr_wdata,
	input  wire 				    csr_we,
	
	// 读端口
	input  wire  [`CSR_ADDR_BUS]	csr_raddr,
	output reg   [`REG_BUS 	   ]    csr_rdata,

	//	异常指令添加信号
	output	wire 					global_trap_ena,
	output	wire 					time_trap_ena,
	output	wire [`REG_BUS	   ]	excep_mtvec,
	output	wire [`REG_BUS	   ]	excep_mret_mepc,

	input	wire [`EXCEP_CODE_BUS]	excep_code
	
);

    //定义8个64位寄存器
    reg [`REG_BUS] 	csrs[0:`CSR_NUM];

	//将全局中断使能，计时器中断使能，mtvec寄存器值，mepc寄存器值输出
	assign	global_trap_ena = csrs[`MSTATUS][3];
	assign	time_trap_ena	= csrs[`MIE][7];
	assign	excep_mtvec 	= csrs[`MTVEC];
	assign	excep_mret_mepc = csrs[`MEPC];
    
	
	always @(posedge cpu_clk_50M) begin
		if (cpu_rst_n == `YSYX219999_RST_ENABLE) begin
			csrs[ 0] <= `YSYX219999_ZERO_WORD;
			csrs[ 1] <= `YSYX219999_ZERO_WORD;
			csrs[ 2] <= `YSYX219999_ZERO_WORD;
			csrs[ 3] <= `YSYX219999_ZERO_WORD;
			csrs[ 4] <= `YSYX219999_ZERO_WORD;
			csrs[ 5] <= `YSYX219999_ZERO_WORD;
			csrs[ 6] <= `YSYX219999_ZERO_WORD;
			csrs[ 7] <= `YSYX219999_ZERO_WORD;
			csrs[ 8] <= `YSYX219999_ZERO_WORD;
		end
		/*********************异常处理相关指令***************************/

		else if(excep_code == `TIME_TRAP_CODE) begin
			csrs[`MCAUSE]		<= 64'h80000000_00000007;
			csrs[`MSTATUS][7]	<= csrs[`MSTATUS][3];
			csrs[`MSTATUS][3]	<= `TRAP_OFF; 
			csrs[`MSTATUS][12:11]<=	`MACHINE_MODE;
		end
		else if(excep_code == `MRET_CODE) begin
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
		else if((csr_we == `YSYX219999_WRITE_ENABLE) && (csr_waddr == `MSTATUS)) begin
			csrs[csr_waddr] <= csr_wdata;
			csrs[`SSTATUS][63]	 <= csr_wdata[63];
			csrs[`SSTATUS][16:13]<= csr_wdata[16:13];
			csrs[`MCYCLE]	<= csrs[`MCYCLE] + 1;
		end
		else if((csr_we == `YSYX219999_WRITE_ENABLE) && (csr_waddr != `MCYCLE))begin
			csrs[csr_waddr] <= csr_wdata;
			csrs[`MCYCLE]	<= csrs[`MCYCLE] + 1;
		end
		else if((csr_we == `YSYX219999_WRITE_ENABLE) && (csr_waddr == `MCYCLE))begin
			csrs[csr_waddr] <= csr_wdata;
		end
		else begin
			csrs[`MCYCLE]	<= csrs[`MCYCLE] + 1;
		end
	end
	
	//读csr寄存器的读操作 
	// ra1是读地址、wa是写地址、we是写使能、wd是要写入的数据 
	always @(*) begin
		if (cpu_rst_n == `YSYX219999_RST_ENABLE) begin
			csr_rdata = `YSYX219999_ZERO_WORD;
		end
		else if ((csr_we == `YSYX219999_WRITE_ENABLE) && (csr_waddr == csr_raddr)) begin
			csr_rdata = csr_wdata;
		end
		else begin
			csr_rdata = csrs[csr_raddr];
		end	
	end
	

endmodule



module ysyx_219999_exemem_reg(
	input	wire					cpu_clk_50M,
	input	wire					cpu_rst_n,
	
	//来自执行阶段的信息
	input	wire[`YSYX219999_ALUOP_BUS]		exe_aluop,
	input	wire[`REG_ADDR_BUS]		exe_wa,
	input	wire					exe_wreg,
	input	wire[`REG_BUS]			exe_wd,
	input	wire 					exe_mreg,
	input	wire[`REG_BUS]			exe_din,
	
	
	//送到访存阶段的信息
	output	reg[`YSYX219999_ALUOP_BUS]			mem_aluop,
	output	reg[`REG_ADDR_BUS]		mem_wa,
	output	reg						mem_wreg, 
	output	reg[`REG_BUS]			mem_wd,
	output	reg 					mem_mreg,
	output	reg[`REG_BUS]			mem_din,

	//访存产生的暂停信号
	input	wire					data_read_stall,

	//csr指令添加信号
	input	wire 					exe_csr_we,
	input	wire[`CSR_ADDR_BUS]		exe_csr_waddr,
	input	wire[`REG_BUS	  ]		exe_csr_wdata,

	output	reg 					mem_csr_we,
	output	reg[`CSR_ADDR_BUS]		mem_csr_waddr,
	output	reg[`REG_BUS	 ]		mem_csr_wdata,
	
	//异常处理flush冲刷信号

    input	wire 					excep_flush
	
);
	
    always@(posedge cpu_clk_50M) begin
		if(cpu_rst_n == `YSYX219999_RST_ENABLE) begin
		    mem_aluop   <=      `YSYX219999_RISCV64_SLL;
			mem_wa		<=		`REG_YSYX219999_NOP;
			mem_wreg	<=		`YSYX219999_WRITE_DISABLE;
			mem_wd		<=		`YSYX219999_ZERO_WORD;
			mem_mreg	<=		`YSYX219999_WRITE_DISABLE;
			mem_din		<=		`YSYX219999_ZERO_WORD;
			mem_csr_we	<=		`YSYX219999_FALSE_V;
			mem_csr_waddr	<=	4'b0;
			mem_csr_wdata	<=	`YSYX219999_ZERO_WORD;
		end
		if(excep_flush == `YSYX219999_TRUE_V) begin
		    mem_aluop   <=      `YSYX219999_RISCV64_SLL;
			mem_wa		<=		`REG_YSYX219999_NOP;
			mem_wreg	<=		`YSYX219999_WRITE_DISABLE;
			mem_wd		<=		`YSYX219999_ZERO_WORD;
			mem_mreg	<=		`YSYX219999_WRITE_DISABLE;
			mem_din		<=		`YSYX219999_ZERO_WORD;
			mem_csr_we	<=		exe_csr_we;
			mem_csr_waddr<=		exe_csr_waddr;
			mem_csr_wdata<=		exe_csr_wdata;
		end
		else if(data_read_stall == `NOSTOP) begin
			mem_aluop	<=		exe_aluop;
			mem_wa		<=		exe_wa;
			mem_wreg	<=		exe_wreg;
			mem_wd		<=		exe_wd;
			mem_mreg	<=		exe_mreg;
			mem_din		<=		exe_din;
			mem_csr_we	<=		exe_csr_we;
			mem_csr_waddr<=		exe_csr_waddr;
			mem_csr_wdata<=		exe_csr_wdata;
		end
	end

endmodule

module ysyx_219999_exe_stage(
    
    //从译码阶段获得的信息
	input		wire[`YSYX219999_ALUTYPE_BUS]		exe_alutype_i,
	input		wire[`YSYX219999_ALUOP_BUS]		exe_aluop_i,
    input		wire[`REG_BUS]			exe_src1_i,
	input		wire[`REG_BUS]			exe_src2_i,
	input		wire[`REG_ADDR_BUS]		exe_wa_i,
	input		wire					exe_wreg_i,
	input		wire					exe_mreg_i,
	input		wire[`REG_BUS]			exe_din_i,
	input		wire[`YSYX219999_INST_ADDR_BUS]	exe_pc_i,

	//转移指令添加
	input       wire[`YSYX219999_INST_ADDR_BUS]   ret_addr,
	
	//送至执行阶段的信息
	output		wire[`YSYX219999_ALUOP_BUS]		exe_aluop_o,
	output		wire[`REG_ADDR_BUS]		exe_wa_o,
	output		wire					exe_wreg_o,
	output		wire[`REG_BUS]			exe_wd_o,
	output		wire 					exe_mreg_o,
	output		wire[`REG_BUS]			exe_din_o,

	//csr指令添加
	input		wire[`CSR_ADDR_BUS]		exe_csr_waddr_i,
	output		wire 					exe_csr_we,
	output		wire[`CSR_ADDR_BUS]		exe_csr_waddr,
	output		wire[`REG_BUS]			exe_csr_wdata,

	input	wire[`REG_BUS	  ]	mtvec_read,
	input	wire[`REG_BUS	  ] mepc_read,

	//	异常指令添加信号
	input	wire 				global_trap_ena,
	input	wire 				time_trap_ena,

	output	wire 				  excep_flush,
	output	wire[`EXCEP_CODE_BUS] excep_code,
	output	wire[`YSYX219999_INST_ADDR_BUS ] excep_flush_pc,

	//	时钟中断信号
	input	wire 				clock_interr,
	output	wire 				interr_isdone
	
);

    //直接传到下一阶段
	assign  exe_aluop_o =  exe_aluop_i;
    assign  exe_wa_o    =  exe_wa_i;
    assign  exe_wreg_o  =  exe_wreg_i;
	assign	exe_mreg_o	=  exe_mreg_i;
	assign	exe_din_o	=  exe_din_i;

    wire		[`REG_BUS]				YSYX219999_LOGICres;           //保存逻辑运算的结果
	wire		[`REG_BUS]				YSYX219999_SHIFTres;           //保存移位运算的结果
	wire       	[`REG_BUS]              YSYX219999_MOVEres;           	//保存移动操作的结果
	wire		[`REG_BUS]			    YSYX219999_ARITHres;           //保存算术运算的结果

	//根据内部操作码aluop进行逻辑运算
	assign  YSYX219999_LOGICres = (exe_aluop_i == `YSYX219999_RISCV64_ANDI)   ?   (exe_src1_i & exe_src2_i)  :
	                   (exe_aluop_i == `YSYX219999_RISCV64_AND)    ?   (exe_src1_i & exe_src2_i)  :
	                   (exe_aluop_i == `YSYX219999_RISCV64_ORI)    ?   (exe_src1_i | exe_src2_i)  :
	                   (exe_aluop_i == `YSYX219999_RISCV64_OR)     ?   (exe_src1_i | exe_src2_i)  :
	                   (exe_aluop_i == `YSYX219999_RISCV64_XORI)   ?   (exe_src1_i ^ exe_src2_i)  :
	                   (exe_aluop_i == `YSYX219999_RISCV64_XOR)    ?   (exe_src1_i ^ exe_src2_i)  :
	                   (exe_aluop_i == `YSYX219999_RISCV64_LUI)    ?   exe_src2_i 				   :
	                   (exe_aluop_i == `YSYX219999_RISCV64_AUIPC)  ?   (exe_src1_i + exe_src2_i)  :  `YSYX219999_ZERO_WORD;

	wire		[`REG_BUS]			    add_test;
	wire 		[`REG_BUS]				sub_test;
	wire 		[`REG_BUS]				lYSYX219999_SHIFT_test;
	wire 		[`REG_BUS]				rYSYX219999_SHIFT_test;
	assign	add_test = (exe_src1_i + exe_src2_i);
	assign	sub_test = exe_src1_i + (~exe_src2_i) + 1;
	assign	lYSYX219999_SHIFT_test = exe_src1_i << exe_src2_i;
	assign	rYSYX219999_SHIFT_test = {32'b0,exe_src1_i[31:0] >> exe_src2_i};
	//根据内部操作码aluop进行算术运算
	assign	YSYX219999_ARITHres = (exe_aluop_i == `YSYX219999_RISCV64_ADDI)	?	(exe_src1_i + exe_src2_i)  															:
					   (exe_aluop_i == `YSYX219999_RISCV64_ADD)    ?   (exe_src1_i + exe_src2_i)  															:
					   (exe_aluop_i == `YSYX219999_RISCV64_ADDIW)	?	{{32{add_test[31]}},{add_test[31:0]}}												:
					   (exe_aluop_i == `YSYX219999_RISCV64_ADDW)	?	{{32{add_test[31]}},{add_test[31:0]}}												:
					   (exe_aluop_i == `YSYX219999_RISCV64_SUB)    ?   (exe_src1_i + (~exe_src2_i) + 1)   													:
					   (exe_aluop_i == `YSYX219999_RISCV64_SUBW)	?	{{32{sub_test[31]}},{sub_test[31:0]}}												:
					   (exe_aluop_i == `YSYX219999_RISCV64_SLTI)   ?   (($signed(exe_src1_i) < $signed(exe_src2_i))   ?   64'h00000000_00000001 : 64'b0)  	:
					   (exe_aluop_i == `YSYX219999_RISCV64_SLT)    ?   (($signed(exe_src1_i) < $signed(exe_src2_i))   ?   64'h00000000_00000001 : 64'b0)  	:
					   (exe_aluop_i == `YSYX219999_RISCV64_SLTU)   ?   ((exe_src1_i < exe_src2_i)   ?   64'h00000000_00000001 : 64'b0)    					:
					   (exe_aluop_i == `YSYX219999_RISCV64_SLTIU)  ?   ((exe_src1_i < exe_src2_i)   ?   64'h00000000_00000001 : 64'b0)    					:    `YSYX219999_ZERO_WORD;			   
	//根据内部操作码aluop进行移位运算
	assign  YSYX219999_SHIFTres = (exe_aluop_i == `YSYX219999_RISCV64_SLLI)   ?   (exe_src1_i << exe_src2_i)    																			:
					   (exe_aluop_i == `YSYX219999_RISCV64_SLLIW)	?	{{32{lYSYX219999_SHIFT_test[31]}},{lYSYX219999_SHIFT_test[31:0]}}																:
	                   (exe_aluop_i == `YSYX219999_RISCV64_SRLI)   ?   (exe_src1_i >> exe_src2_i)    																			:
					   (exe_aluop_i == `YSYX219999_RISCV64_SRLIW)	?	{{32{rYSYX219999_SHIFT_test[31]}},{rYSYX219999_SHIFT_test[31:0]}}																:
	                   (exe_aluop_i == `YSYX219999_RISCV64_SRAI)   ?   (({64{exe_src1_i[63]}} << (7'd64 - {1'b0,exe_src2_i[5:0]})) | (exe_src1_i >> exe_src2_i))    			:
					   (exe_aluop_i == `YSYX219999_RISCV64_SRAIW)	?	(({64{exe_src1_i[31]}} << (6'd32 - {1'b0,exe_src2_i[5:0]})) | {32'b0,(exe_src1_i[31:0] >> exe_src2_i)})			:
	                   (exe_aluop_i == `YSYX219999_RISCV64_SLL)    ?   (exe_src1_i << exe_src2_i)    																			:
					   (exe_aluop_i == `YSYX219999_RISCV64_SLLW)	?	(({64{exe_src1_i[31]}} << (6'd32 + {1'b0,exe_src2_i[4:0]}))| {32'b0,(exe_src1_i[31:0] << exe_src2_i[4:0])})		:
	                   (exe_aluop_i == `YSYX219999_RISCV64_SRL)    ?   (exe_src1_i >> exe_src2_i)    																			: 
					   (exe_aluop_i == `YSYX219999_RISCV64_SRLW)	?	{32'b0,exe_src1_i[31:0] >> exe_src2_i[4:0]}																:
	                   (exe_aluop_i == `YSYX219999_RISCV64_SRA)    ?   (({64{exe_src1_i[63]}} << (7'd64 - {1'b0,exe_src2_i[5:0]})) | (exe_src1_i >> exe_src2_i))    			:
					   (exe_aluop_i == `YSYX219999_RISCV64_SRAW)	?	(({64{exe_src1_i[31]}} << (6'd32 - {1'b0,exe_src2_i[4:0]})) | {32'b0,(exe_src1_i[31:0] >> exe_src2_i[4:0])})	:	`YSYX219999_ZERO_WORD;
											

	//根据内部类型码alutype确定数据存储器存取操作的地址
	assign     YSYX219999_MOVEres = (exe_alutype_i == `YSYX219999_MOVE) ? (exe_src1_i + exe_src2_i) : `YSYX219999_ZERO_WORD;
	   
							

	//根据内部操作码aluop进行csr指令操作
	assign	exe_csr_we	=	(exe_aluop_i == `YSYX219999_RISCV64_ECALL || (global_trap_ena == `TRAP_ON && time_trap_ena == `TRAP_ON && clock_interr == `YSYX219999_TRUE_V))	?	`YSYX219999_TRUE_V	:
							(exe_alutype_i == `CSR)			?	`YSYX219999_TRUE_V		:	`YSYX219999_FALSE_V;
	assign	exe_csr_waddr=	(exe_aluop_i == `YSYX219999_RISCV64_ECALL || (global_trap_ena == `TRAP_ON && time_trap_ena == `TRAP_ON && clock_interr == `YSYX219999_TRUE_V))	?	`MEPC			:
							(exe_alutype_i == `CSR)			?	exe_csr_waddr_i	:	4'b0;
	assign	exe_csr_wdata=	(exe_aluop_i == `YSYX219999_RISCV64_ECALL || (global_trap_ena == `TRAP_ON && time_trap_ena == `TRAP_ON && clock_interr == `YSYX219999_TRUE_V))	?	exe_pc_i		:
							(exe_aluop_i == `YSYX219999_RISCV64_CSRRW)	?	exe_src1_i	:
							(exe_aluop_i == `YSYX219999_RISCV64_CSRRS)	?	(exe_src1_i | exe_src2_i)	:
							(exe_aluop_i == `YSYX219999_RISCV64_CSRRC)	?	((~exe_src1_i) & exe_src2_i)	:
							(exe_aluop_i == `YSYX219999_RISCV64_CSRRWI)?	exe_src1_i	:
							(exe_aluop_i == `YSYX219999_RISCV64_CSRRSI)?	(exe_src1_i | exe_src2_i)	:
							(exe_aluop_i == `YSYX219999_RISCV64_CSRRCI)?	((~exe_src1_i) & exe_src2_i)	:	`YSYX219999_ZERO_WORD;

    wire    exe_warning_deltest = (add_test == `YSYX219999_ZERO_WORD && sub_test == `YSYX219999_ZERO_WORD && lYSYX219999_SHIFT_test == `YSYX219999_ZERO_WORD && rYSYX219999_SHIFT_test == `YSYX219999_ZERO_WORD)  ?   `YSYX219999_FALSE_V    :   `YSYX219999_FALSE_V;

    assign  exe_wd_o    =   (exe_alutype_i == `YSYX219999_ARITH)   	?   YSYX219999_ARITHres    :  
					  		(exe_alutype_i ==  `YSYX219999_LOGIC)      ?   YSYX219999_LOGICres    :
					  		(exe_alutype_i ==  `YSYX219999_SHIFT)      ?   YSYX219999_SHIFTres    :
					  		(exe_alutype_i ==  `YSYX219999_MOVE )      ?   YSYX219999_MOVEres     :
							(exe_alutype_i ==  `JUMP)       ?   ret_addr    :
							(exe_alutype_i ==  `CSR)		?	exe_src2_i	:
                            (exe_warning_deltest == `YSYX219999_TRUE_V)?   `YSYX219999_ZERO_WORD  :	`YSYX219999_ZERO_WORD;


    

	/********************************异常处理模块******************************/

	assign	excep_flush	=	((exe_aluop_i == `YSYX219999_RISCV64_ECALL) || exe_aluop_i == `YSYX219999_RISCV64_MRET || (global_trap_ena == `TRAP_ON && time_trap_ena == `TRAP_ON && clock_interr == `YSYX219999_TRUE_V && exe_pc_i != `YSYX219999_PC_INIT))	?	`YSYX219999_TRUE_V		:	`YSYX219999_FALSE_V;
	assign 	excep_flush_pc =(global_trap_ena == `TRAP_ON && time_trap_ena == `TRAP_ON && clock_interr == `YSYX219999_TRUE_V && exe_pc_i != `YSYX219999_PC_INIT)	?	mtvec_read	:				//(mtvec_read[ 1: 0] == 2'b0)		?	{2'b0,mtvec_read[63:2]}	:
							(exe_aluop_i == `YSYX219999_RISCV64_ECALL)	?	mtvec_read				:															//{2'b0,mtvec_read[63:2]}	:
							(exe_aluop_i == `YSYX219999_RISCV64_MRET)	?	mepc_read				:	`YSYX219999_ZERO_WORD;
	assign	excep_code	= 	(clock_interr== `YSYX219999_TRUE_V && global_trap_ena == `TRAP_ON && time_trap_ena == `TRAP_ON && exe_pc_i != `YSYX219999_PC_INIT)	?	`TIME_TRAP_CODE	:
							(exe_aluop_i == `YSYX219999_RISCV64_ECALL)	?	`ECALL_CODE	:	
							(exe_aluop_i == `YSYX219999_RISCV64_MRET)	?	`MRET_CODE	:	3'b0;

	assign	interr_isdone = clock_interr == `YSYX219999_TRUE_V && global_trap_ena == `TRAP_ON && time_trap_ena == `TRAP_ON && exe_pc_i != `YSYX219999_PC_INIT;

	/********************************异常处理模块******************************/


endmodule

module ysyx_219999_idexe_reg(
    input   wire    cpu_clk_50M,
    input   wire    cpu_rst_n,
    
    //来自译码阶段的信息
    input   wire [`YSYX219999_ALUTYPE_BUS] id_alutype,
    input   wire [`YSYX219999_ALUOP_BUS]   id_aluop,
    input   wire [`REG_BUS]     id_src1,
    input   wire [`REG_BUS]     id_src2,
    input   wire [`REG_ADDR_BUS]id_wa,
    input   wire                id_wreg,
    input   wire                id_mreg,
    input   wire[`REG_BUS]      id_din,
	input	wire[`YSYX219999_INST_ADDR_BUS]id_pc,
    
    //送至执行阶段的信息
    output  reg [`YSYX219999_ALUTYPE_BUS]  exe_alutype,
    output  reg [`YSYX219999_ALUOP_BUS]    exe_aluop,
    output  reg [`REG_BUS]      exe_src1,
    output  reg [`REG_BUS]      exe_src2,
    output  reg [`REG_ADDR_BUS] exe_wa,
    output  reg                 exe_wreg,
    output  reg                 exe_mreg,
    output  reg [`REG_BUS]      exe_din,
	output	reg [`YSYX219999_INST_ADDR_BUS]exe_pc,

    //流水线暂停信号
    input   wire                id_stall,
    input   wire                data_read_stall,

    //跳转指令添加
    input   wire [`REG_BUS  ]   id_ret_addr,
    output  reg  [`REG_BUS  ]   exe_ret_addr,

    //csr指令添加
    input   wire [`CSR_ADDR_BUS]id_csr_waddr,
    output  reg  [`CSR_ADDR_BUS] exe_csr_waddr,
	
	//异常处理flush冲刷信号
    input	wire 					excep_flush
    
);

    always@(posedge cpu_clk_50M) begin
        //复位的时候将送至执行阶段的信息清0
        if(cpu_rst_n == `YSYX219999_RST_ENABLE) begin
            exe_alutype <=      `YSYX219999_NOP;
            exe_aluop   <=      `YSYX219999_RISCV64_SLL;
            exe_src1    <=      `YSYX219999_ZERO_WORD;
            exe_src2    <=      `YSYX219999_ZERO_WORD;
            exe_wa      <=      `REG_YSYX219999_NOP;
            exe_wreg    <=      `YSYX219999_WRITE_DISABLE;
            exe_mreg    <=      `YSYX219999_WRITE_DISABLE;
            exe_din     <=      `YSYX219999_ZERO_WORD;
            exe_ret_addr<=      `YSYX219999_ZERO_WORD;
			exe_pc		<=		`YSYX219999_PC_INIT; 
            exe_csr_waddr<=     4'b0;
        end
        else  if(excep_flush == `YSYX219999_TRUE_V)    begin
            exe_alutype <=      `YSYX219999_NOP;
            exe_aluop   <=      `YSYX219999_RISCV64_SLL;
            exe_src1    <=      `YSYX219999_ZERO_WORD;
            exe_src2    <=      `YSYX219999_ZERO_WORD;
            exe_wa      <=      `REG_YSYX219999_NOP;
            exe_wreg    <=      `YSYX219999_WRITE_DISABLE;
            exe_mreg    <=      `YSYX219999_WRITE_DISABLE;
            exe_din     <=      `YSYX219999_ZERO_WORD;
            exe_ret_addr<=      `YSYX219999_ZERO_WORD;
			exe_pc		<=		`YSYX219999_PC_INIT;
            exe_csr_waddr<=     4'b0;
        end
        else  if(id_stall == `STOP)    begin
            exe_alutype <=      `YSYX219999_NOP;
            exe_aluop   <=      `YSYX219999_RISCV64_SLL;
            exe_src1    <=      `YSYX219999_ZERO_WORD;
            exe_src2    <=      `YSYX219999_ZERO_WORD;
            exe_wa      <=      `REG_YSYX219999_NOP;
            exe_wreg    <=      `YSYX219999_WRITE_DISABLE;
            exe_mreg    <=      `YSYX219999_WRITE_DISABLE;
            exe_din     <=      `YSYX219999_ZERO_WORD;
            exe_ret_addr<=      `YSYX219999_ZERO_WORD;
			exe_pc		<=		`YSYX219999_PC_INIT;
            exe_csr_waddr<=     4'b0;
        end
        else if(data_read_stall == `NOSTOP)  begin
            exe_alutype <=      id_alutype;
            exe_aluop   <=      id_aluop;
            exe_src1    <=      id_src1;
            exe_src2    <=      id_src2;
            exe_wa      <=      id_wa;
            exe_wreg    <=      id_wreg;
            exe_mreg    <=      id_mreg;
            exe_din     <=      id_din;
            exe_ret_addr<=      id_ret_addr;
			exe_pc		<=		id_pc;
            exe_csr_waddr<=     id_csr_waddr;
        end
    end
endmodule


module ysyx_219999_id_stage(
    
    input  wire                     cpu_rst_n,
    // 从取指阶段获得的pc值与指令
    input  wire [`YSYX219999_INST_ADDR_BUS]    id_pc_i,
    input  wire [`YSYX219999_INST_ADDR_BUS]    pc_plus_4,
    input  wire [`YSYX219999_INST_BUS     ]    id_inst_i,

    // 从通用寄存器堆读出的数据 
    input  wire [`REG_BUS      ]    rd1,
    input  wire [`REG_BUS      ]    rd2,

    // 送至执行阶段的译码信息
    output wire [`YSYX219999_ALUTYPE_BUS  ]    id_alutype_o,
    output wire [`YSYX219999_ALUOP_BUS    ]    id_aluop_o,
    output wire [`REG_ADDR_BUS ]    id_wa_o,
    output wire                     id_wreg_o,
    output wire                     id_mreg_o,
    output wire [`REG_BUS      ]    id_din_o,
	output wire [`YSYX219999_INST_ADDR_BUS]	id_pc_o,

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


    
    
    // 从取指阶段获得的分支预测信息
    input   wire                    id_jump_ena_i,      //预测跳转方向
    input   wire [`YSYX219999_INST_ADDR_BUS]   id_jump_pc_i,       //预测跳转目标pc值
    
    // 送回译码阶段的跳转地址
    output  wire                     flush,             //分支预测错误时回传流水线冲刷信号
    output  wire [`YSYX219999_INST_ADDR_BUS]    flush_addr,        //分支预测错误时回传流水线冲刷地址
    output  wire [`YSYX219999_INST_ADDR_BUS]    ret_addr,          //函数调用的返回地址
    
    // 送至btb表的更新信息
    output  wire                    btb_update_req,     //btb表写使能信号
    output  wire 		            btb_update_state,   //btb表当前写指令是否发生跳转
    output  wire [`PC_ADDR_BTB  ]   btb_update_pc,      //btb表当前写入指令pc
    output  wire [`YSYX219999_INST_ADDR_BUS]   btb_update_prepc,    //btb表当前写入指令跳转pc

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
    wire [`YSYX219999_INST_BUS] id_inst;
    assign id_inst = (cpu_rst_n == `YSYX219999_RST_ENABLE)	?	32'b0 : id_inst_i;                          //(cpu_rst_n == `YSYX219999_RST_ENABLE)   ?   `YSYX219999_ZERO_INST  :   {id_inst_i[7:0],id_inst_i[15:8],id_inst_i[23:16],id_inst_i[31:24]};


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
    assign id_aluop_o[4]   = inst_addi | inst_slti | inst_sltiu | inst_add | inst_sub | inst_slt | inst_sltu | inst_addiw | inst_addw | inst_subw | inst_lui | inst_auipc | inst_jal | inst_jalr 
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
    wire YSYX219999_SHIFT      =   inst_slli | inst_srli | inst_srai | inst_sll | inst_srl | inst_sra | inst_slliw | inst_srliw | inst_sraiw;
    wire YSYX219999_SHIFT_r    =   inst_sll | inst_srl | inst_sra | inst_sllw | inst_srlw | inst_sraw;

    //移位位数
    wire [`YSYX219999_INST_ADDR_BUS] YSYX219999_SHIFT_ra    = (YSYX219999_SHIFT_r == `YSYX219999_YSYX219999_SHIFT_ENABLE)  ?     {58'b0, rd2[5:0]}             :
                                        (YSYX219999_SHIFT   == `YSYX219999_YSYX219999_SHIFT_ENABLE)  ?     {58'b0,id_inst[25], rs2[4:0]} : `YSYX219999_ZERO_WORD; 

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
                                    (j_imm) ? {{44{id_inst[31]}},id_inst[19:12],id_inst[20],id_inst[30:21],1'b0} : `YSYX219999_ZERO_WORD;

    //获得待写入目的寄存器rd的地址  //无条件跳转指令默认x1寄存器
    assign id_wa_o =   rd;

    /********************csr指令添加*****************************/
    wire csr_i  =   inst_csrrwi | inst_csrrsi | inst_csrrci;
    wire csr_r  =   inst_csrrw | inst_csrrs | inst_csrrc;
    assign  csr_raddr   =   (mret)  ?   `MEPC   :
                            (ecall) ?   `MTVEC  :
                            (inst_csr == `YSYX219999_TRUE_V && csr == 12'h300)  ?   `MSTATUS   :
                            (inst_csr == `YSYX219999_TRUE_V && csr == 12'h305)  ?   `MTVEC     :
                            (inst_csr == `YSYX219999_TRUE_V && csr == 12'h341)  ?   `MEPC      :
                            (inst_csr == `YSYX219999_TRUE_V && csr == 12'h342)  ?   `MCAUSE    :
                            (inst_csr == `YSYX219999_TRUE_V && csr == 12'h304)  ?   `MIE       :
                            (inst_csr == `YSYX219999_TRUE_V && csr == 12'h344)  ?   `MIP       :
                            (inst_csr == `YSYX219999_TRUE_V && csr == 12'hB00)  ?   `MCYCLE    :
                            (inst_csr == `YSYX219999_TRUE_V && csr == 12'h340)  ?   `MSCRATCH  :       4'b0;
    wire[`YSYX219999_WORD_BUS] csr_imm =   (csr_i) ?   {59'b0,rs1} :   `YSYX219999_ZERO_WORD;
    assign  id_csr_waddr    =   (inst_csr && ~mret && ~ecall)  ?   csr_raddr   :  4'b0; 
        
    //csr定向回传添加
    wire[`YSYX219999_WORD_BUS] csr_rdata_select = (exe2id_csr_we == `YSYX219999_WRITE_ENABLE && exe2id_csr_waddr == csr_raddr && csr_raddr != 4'b0)   ?   exe2id_csr_wdata    :
                                       (mem2id_csr_we == `YSYX219999_WRITE_ENABLE && mem2id_csr_waddr == csr_raddr && csr_raddr != 4'b0)   ?   mem2id_csr_wdata    :   csr_rdata;
        
        
    /********************csr指令添加*****************************/

    /********************定向回传添加*****************************/
    //源操作数1选择信号（可能来自当前pc值或寄存器读端口1或定向回传）
    wire [1:0]  fwrd1   =   (exe2id_wreg == `YSYX219999_WRITE_ENABLE && exe2id_wa == ra1 && ra1 != 5'b0)  ?   2'b01   :
                            (mem2id_wreg == `YSYX219999_WRITE_ENABLE && mem2id_wa == ra1 && ra1 != 5'b0)  ?   2'b10   :   2'b11;

    //源操作数2选择信号（可能来自立即数或寄存器读端口2或定向回传）
    wire [1:0]  fwrd2   =   (exe2id_wreg == `YSYX219999_WRITE_ENABLE && exe2id_wa == ra2 && ra2 != 5'b0)  ?   2'b01   :
                            (mem2id_wreg == `YSYX219999_WRITE_ENABLE && mem2id_wa == ra2 && ra2 != 5'b0)  ?   2'b10   :   2'b11;                      

    /********************定向回传添加*****************************/

    
    //获得源操作数1来自读端口rs1
    assign  id_src1_o   =   (csr_i)             ?   csr_imm     :
                            (csr_r)             ?   rd1         :
                            (inst_auipc)        ?   id_pc_i     :
                            (fwrd1 == 2'b01)    ?   exe2id_wd   :   
                            (fwrd1 == 2'b10)    ?   mem2id_wd   :  rd1;

    //获得源操作数2
    assign  id_src2_o   =   (csr_i | csr_r)             ?   csr_rdata_select   :
                            (immsel == `YSYX219999_IMM_ENABLE )    ?   imm_ext     : 
                            (YSYX219999_SHIFT == `YSYX219999_YSYX219999_SHIFT_ENABLE)    ?   YSYX219999_SHIFT_ra    :
                            (fwrd2 == 2'b01)            ?   exe2id_wd   :
                            (fwrd2 == 2'b10)            ?   mem2id_wd   :   rd2;

    //获得访存阶段要存入数据存储器的数据sw sb sh 指令来自rs2寄存器
    assign  id_din_o    =   (fwrd2 == 2'b01)    ?   exe2id_wd    :
                            (fwrd2 == 2'b10)    ?   mem2id_wd    :   rd2 ;

    //加载相关暂停信号
    assign  id_stall    =   ((fwrd1 == 2'b01 || fwrd2 == 2'b01) && (exe2id_mreg == `YSYX219999_TRUE_V))    ?   `STOP   :
                            ((fwrd1 == 2'b10 || fwrd2 == 2'b10) && (mem2id_mreg == `YSYX219999_TRUE_V))    ?   `STOP   :   `NOSTOP;

    //获得转移控制信号
    wire equ    =   (inst_beq   )   ?   (id_src1_o  ==  id_src2_o)  :
                    (inst_bne   )   ?   (id_src1_o  !=  id_src2_o)  :
                    (inst_blt   )   ?   ($signed(id_src1_o)     <   $signed(id_src2_o))   :
                    (inst_bltu  )   ?   ($unsigned(id_src1_o)   <   $unsigned(id_src2_o))   :
                    (inst_bge   )   ?   ($signed(id_src1_o)     >=  $signed(id_src2_o))     :
                    (inst_bgeu  )   ?   ($unsigned(id_src1_o)   >=  $unsigned(id_src2_o))   :   1'b0;
                    
    //生成转移地址计算所需信号
    wire [`YSYX219999_INST_ADDR_BUS] imm_jump = (inst_jalr) ? {{52{id_inst[31]}},id_inst[31: 20]}  :
                                     (inst_jal ) ? {{44{id_inst[31]}},id_inst[19:12],id_inst[20],id_inst[30:21],1'b0}  :
                                     (b_imm)     ? {{52{id_inst[31]}},id_inst[7],id_inst[30:25],id_inst[11:8],1'b0}  :   `YSYX219999_ZERO_WORD;

    wire    jtsel;
    //获得转移指令使能信号
    assign  jtsel     = inst_jal | inst_jalr | equ;
    
    //获得转移地址
    wire [`YSYX219999_INST_ADDR_BUS] jump_addr;
    assign  jump_addr = (inst_jal | equ)    ?   id_pc_i + imm_jump  :
                        (inst_jalr)         ?   id_src1_o + imm_jump      :   pc_plus_4;
                        
    assign  flush   =   (id_stall == `STOP)    ?   `YSYX219999_JUMP_DISABLE   :
                        (jtsel == `YSYX219999_JUMP_ENABLE  && id_jump_ena_i == `YSYX219999_JUMP_DISABLE)                              ?   1'b1    :
                        (jtsel == `YSYX219999_JUMP_DISABLE && id_jump_ena_i == `YSYX219999_JUMP_ENABLE )                              ?   1'b1    :
                        (jtsel == `YSYX219999_JUMP_ENABLE  && id_jump_ena_i == `YSYX219999_JUMP_ENABLE && jump_addr != id_jump_pc_i)  ?   1'b1    :    1'b0;   
    
    assign  flush_addr = (flush == `YSYX219999_FALSE_V)    ?   `YSYX219999_ZERO_WORD  :
                         (flush == `YSYX219999_TRUE_V && jtsel == `YSYX219999_JUMP_ENABLE)     ?   jump_addr   :
                         (flush == `YSYX219999_TRUE_V && jtsel == `YSYX219999_JUMP_DISABLE)    ?   pc_plus_4   :   `YSYX219999_ZERO_WORD;
    
    assign  btb_update_req  =   (id_stall == `STOP)    ?   `YSYX219999_FALSE_V : (inst_jal | inst_jalr | b_imm);                               
    assign  btb_update_state=   jtsel;
    assign  btb_update_pc   =   (btb_update_req == `YSYX219999_TRUE_V)   ?   id_pc_i[11:2]   :   10'b0;
    assign  btb_update_prepc=   (btb_update_req == `YSYX219999_TRUE_V)   ?   jump_addr 	     :   `YSYX219999_ZERO_WORD; 
    
    //生成子程序调用返回地址
    assign ret_addr   = pc_plus_4;

	//pc值与指令向下传递//difftest needed
    assign id_pc_o   = id_pc_i;
endmodule


module ysyx_219999_ifid_reg(
    input   wire    cpu_clk_50M,
    input   wire    cpu_rst_n,
    
    //来自取指阶段的信息
    input   wire [`YSYX219999_INST_ADDR_BUS]   if_pc,
    input   wire [`YSYX219999_INST_BUS     ]   if_inst,
    input   wire [`YSYX219999_INST_ADDR_BUS]   if_pc_plus_4,
    
    //送至译码阶段的信息
    output  reg [`YSYX219999_INST_ADDR_BUS]    id_pc,
    output  reg [`YSYX219999_INST_BUS     ]    id_inst,
    output  reg [`YSYX219999_INST_ADDR_BUS]    id_pc_plus_4,

    //流水线暂停信号
    input   wire                    id_stall,
    input   wire                    data_read_stall,

    //axi读指令完成信号
    input   wire                    handshake_done,

    //动态预测添加
    input   wire                    if_jump_ena,
    input   wire [`YSYX219999_INST_ADDR_BUS]   if_jump_pc,
    input   wire                    flush,
    
    output  reg                     id_jump_ena,
    output  reg [`YSYX219999_INST_ADDR_BUS]    id_jump_pc,

    //异常处理flush冲刷信号添加
    input   wire                    excep_flush
 
);

    always@(posedge cpu_clk_50M) begin
        if(cpu_rst_n == `YSYX219999_RST_ENABLE) begin //复位或流水线冲刷的时候将译码阶段的信息清0
            id_pc           <=  `YSYX219999_PC_INIT;
            id_inst         <=  `YSYX219999_ZERO_INST;
            id_pc_plus_4    <=  `YSYX219999_PC_INIT;
            id_jump_ena     <=  `YSYX219999_JUMP_DISABLE;
            id_jump_pc      <=  `YSYX219999_PC_INIT;
        end
        else if(handshake_done == `YSYX219999_FALSE_V || flush == `YSYX219999_TRUE_V || excep_flush == `YSYX219999_TRUE_V) begin       //指令未取到时暂停译码
            id_pc           <=  `YSYX219999_PC_INIT;
            id_inst         <=  `YSYX219999_ZERO_INST;
            id_pc_plus_4    <=  `YSYX219999_PC_INIT;
            id_jump_ena     <=  `YSYX219999_JUMP_DISABLE;
            id_jump_pc      <=  `YSYX219999_PC_INIT;
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


module ysyx_219999_if_stage(
    input       wire        cpu_clk_50M,
    input       wire        cpu_rst_n,
    
    output 	    wire  [`YSYX219999_INST_ADDR_BUS] 	    if_pc_o,
    output      wire  [`YSYX219999_INST_BUS     ]       if_inst_o,

    //流水线暂停信号
    input                                   stall,

    //axi访问ram信号
    output                                  if_valid,
	input                                   if_ready,
	input       wire [31: 0]            if_data_read,
	output      wire  [31: 0]       if_addr,
	output      wire [1:0]                  if_size,
	input       wire [1:0]                  if_resp,

    output      wire                        handshake_done,

    //分支预测添加
    input       wire                        if_pre_jumpena,
    input       wire [`YSYX219999_INST_ADDR_BUS]       if_pre_pc,
    output      wire                        jump_ena_o, //跳转使能，是否预测跳转
    output      wire [`YSYX219999_INST_ADDR_BUS]       jump_pc_o, 	//预测跳转地址
    
    //预测错误回传信号
    input       wire [`YSYX219999_INST_ADDR_BUS]       flush_addr,
    input       wire                        flush,
    output      wire [`YSYX219999_INST_ADDR_BUS]       pc_plus_4,

    //异常处理跳转执行相关信号添加
    input       wire                        excep_flush,
    input       wire [`YSYX219999_INST_ADDR_BUS]       excep_flush_pc
    
    );

    reg [`YSYX219999_INST_ADDR_BUS] pc;

    assign pc_plus_4 = (cpu_rst_n == `YSYX219999_RST_ENABLE) ? `YSYX219999_PC_INIT   :    pc+4;

    reg ce;
    always@(posedge cpu_clk_50M) begin
        if(cpu_rst_n == `YSYX219999_RST_ENABLE) begin
            ce <= `YSYX219999_CHIP_DISABLE;          //复位的时候指令存储器禁用
        end else begin
            ce <= `YSYX219999_CHIP_ENABLE;            //复位结束时，指令存储器使能
        end
    end
    
    /************************axi访问信号添加************************/
    reg hs_ena;
    always@(posedge cpu_clk_50M) begin
        if(cpu_rst_n == `YSYX219999_RST_ENABLE) begin
            hs_ena <= `YSYX219999_FALSE_V;
        end
        else if(flush == `YSYX219999_TRUE_V) begin
            hs_ena <= `YSYX219999_FALSE_V;
        end
        else if(if_ready == `YSYX219999_TRUE_V) begin
            hs_ena <= `YSYX219999_TRUE_V;
        end
    end

    reg excep_inst_del;
    reg [1:0] excep_inst_del_delay;   //信号边缘有问题不得不加。。

    assign  handshake_done = (hs_ena == `YSYX219999_FALSE_V)   ?   `YSYX219999_FALSE_V    :   if_valid & if_ready;

    assign  if_valid    =   (excep_flush == `YSYX219999_TRUE_V)                        ?   ce              :
                            (cpu_rst_n == `YSYX219999_RST_ENABLE || flush == `YSYX219999_TRUE_V)  ?   `YSYX219999_CHIP_DISABLE   :   ce;
    assign  if_size     = `SIZE_W;
    assign  if_addr     = pc[31: 0];
    assign  if_inst_o   = (excep_inst_del == `YSYX219999_FALSE_V && handshake_done == `YSYX219999_TRUE_V) ?  if_data_read          :   `YSYX219999_ZERO_INST;
    assign  if_pc_o     = (excep_inst_del == `YSYX219999_FALSE_V && handshake_done == `YSYX219999_TRUE_V) ?  pc                    :   `YSYX219999_PC_INIT;
    /************************axi访问信号添加************************/

    /************************分支预测添加************************/
    wire is_jal  = (if_inst_o[6:0] == 7'b1101111)    ?   `YSYX219999_TRUE_V    :    `YSYX219999_FALSE_V;
    wire is_jalr = (if_inst_o[6:0] == 7'b1100111)    ?   `YSYX219999_TRUE_V    :    `YSYX219999_FALSE_V;
    wire is_bj   = (if_inst_o[6:0] == 7'b1100011)    ?   `YSYX219999_TRUE_V    :    `YSYX219999_FALSE_V;
    wire is_jump = is_jal | is_jalr | is_bj;   
     
    wire [`YSYX219999_INST_ADDR_BUS] pc_next;

    assign  jump_ena_o  = (is_jump    == `YSYX219999_TRUE_V && if_pre_jumpena == `YSYX219999_JUMP_ENABLE)	  ?   	`YSYX219999_JUMP_ENABLE	:	`YSYX219999_JUMP_DISABLE;
    assign  jump_pc_o   = (jump_ena_o == `YSYX219999_JUMP_ENABLE)	            ?   if_pre_pc       : 	`YSYX219999_ZERO_WORD;
    assign  pc_next     = (excep_flush== `YSYX219999_TRUE_V)                   ?   excep_flush_pc  :
                          (flush 	  == `YSYX219999_TRUE_V)     		        ?   flush_addr  	:   
                          (jump_ena_o == `YSYX219999_JUMP_ENABLE)          	?   jump_pc_o  		:   pc_plus_4;
    /***********************分支预测添加*************************/

    

    always @(posedge cpu_clk_50M) begin
        if(cpu_rst_n == `YSYX219999_RST_ENABLE) begin
            excep_inst_del  <= `YSYX219999_FALSE_V;
            excep_inst_del_delay <= 2'b0;
        end
        else if(excep_flush == `YSYX219999_TRUE_V) begin
            excep_inst_del  <= `YSYX219999_TRUE_V;
            excep_inst_del_delay <= 2'b01;
        end
        else if(handshake_done == `YSYX219999_TRUE_V && excep_inst_del == `YSYX219999_TRUE_V) begin
            excep_inst_del_delay <= 2'b10;
        end
        else if(excep_inst_del_delay == 2'b10 ) begin
            excep_inst_del       <= `YSYX219999_FALSE_V;
            excep_inst_del_delay <= 2'b01;
        end
    end

    wire    pc_stall; 
    
    assign  pc_stall=  (excep_flush == `YSYX219999_TRUE_V)    ?   `YSYX219999_FALSE_V    :
                        (flush == `YSYX219999_TRUE_V)          ?   `YSYX219999_FALSE_V    :
                        (stall == `NOSTOP && handshake_done == `YSYX219999_TRUE_V && excep_inst_del == `YSYX219999_FALSE_V) ?   `YSYX219999_FALSE_V    :
                        (if_resp == 2'b0)   ?   `YSYX219999_TRUE_V  :   `YSYX219999_TRUE_V;


    always@(posedge cpu_clk_50M) begin
        if(cpu_rst_n == `YSYX219999_RST_ENABLE) begin
            pc <= `YSYX219999_PC_INIT;
        end
        if(ce == `YSYX219999_CHIP_DISABLE)
            //指令存储器禁用的时候，PC保持初始值
            pc <= `YSYX219999_PC_INIT;
        else if(pc_stall == `YSYX219999_FALSE_V) begin
            pc <= pc_next;
        end
    end
     
endmodule

module ysyx_219999_mem_stage(

	
	//	从执行阶段获得的信息
	input	wire[`YSYX219999_ALUOP_BUS]	mem_aluop_i,
	input	wire[`REG_ADDR_BUS]	mem_wa_i,
	input	wire				mem_wreg_i,
	input	wire[`YSYX219999_WORD_BUS]		mem_wd_i,
	input	wire 				mem_mreg_i,
	input	wire[`YSYX219999_WORD_BUS]		mem_din_i,
	
	//	送至写回阶段的信息
	output	wire[`REG_ADDR_BUS]	mem_wa_o,
	output	wire				mem_wreg_o,
	output	wire[`REG_BUS]		mem_dreg_o,
	output	wire				mem_mreg_o,
	output	wire[`YSYX219999_BSEL_BUS]		dre,
	output	wire[`YSYX219999_WORD_BUS]		mem_dm_o,

	//	送至数据存储器的读信号
	output	wire 				rvalid,
	input						r_ready,
	input	wire[`YSYX219999_WORD_BUS]		data_read,
	output	wire[`YSYX219999_WORD_BUS]		raddr,
	output	wire[ 1: 0]			rsize,
	input	wire[ 1: 0]			resp,
	
	//	送至数据存储器的写信号
	output	wire[`YSYX219999_WORD_BUS]		waddr,
	output	wire[`YSYX219999_WORD_BUS]		wdata,
	output	wire[ 7: 0]			wmask,
	output	wire				d_wce,
    input   wire                write_is_down,
	
	//	访问数据存储器引起的流水线暂停信号
	output	wire 				d_stall,

	output	wire 				sign,

	//	csr指令添加信号
	input	wire 				mem_csr_we_i,
	input	wire[`CSR_ADDR_BUS]	mem_csr_waddr_i,
	input	wire[`REG_BUS	  ]	mem_csr_wdata_i,
	
	output	wire 				mem_csr_we_o,
	output	wire[`CSR_ADDR_BUS]	mem_csr_waddr_o,
	output	wire[`REG_BUS	  ]	mem_csr_wdata_o

);


    //	如果当前指令不是访存指令，则只需将从执行阶段获得的信息直接输出
	assign mem_wa_o    	=(resp == 2'b0) ?   mem_wa_i    :   mem_wa_i;
	assign mem_wreg_o  	=   mem_wreg_i;
	assign mem_dreg_o  	=   mem_wd_i;
	assign mem_mreg_o	=	mem_mreg_i;


	//	ecall指令时向mepc寄存器写入当前异常pc，否则将执行阶段获得的信息直接输出
	assign mem_csr_we_o   = mem_csr_we_i;		
	assign mem_csr_waddr_o= mem_csr_waddr_i;
	assign mem_csr_wdata_o=	mem_csr_wdata_i;
	
		
	// 确定当前的访存指令
    wire inst_lb = (mem_aluop_i == 8'h74);
    wire inst_lw = (mem_aluop_i == 8'h71);
    wire inst_sb = (mem_aluop_i == 8'h70);
    wire inst_sw = (mem_aluop_i == 8'h76);
    wire inst_lh = (mem_aluop_i == 8'h72);
    wire inst_lbu= (mem_aluop_i == 8'h75);
    wire inst_lhu= (mem_aluop_i == 8'h73);
    wire inst_sh = (mem_aluop_i == 8'h77);
    wire inst_ld = (mem_aluop_i == 8'h90);  //RISCV64I ONLY
	wire inst_lwu= (mem_aluop_i == 8'h91);	//RISCV64I ONLY
	wire inst_sd = (mem_aluop_i == 8'h92);	//RISCV64I ONLY

	//	有无符号加载存储器
	assign	sign	=	inst_lb | inst_lw | inst_lh;


	/******************************************axi读数据存储器信号***************************************/
	wire	read_handshake_done;
	assign	read_handshake_done = rvalid & r_ready;

	//	获得数据存储器读使能信号
	wire 	d_rce	=	inst_lb | inst_lbu | inst_lw | inst_lwu | inst_lh | inst_lhu | inst_ld;

	assign 	rvalid	=	(d_rce)	?	`YSYX219999_TRUE_V	:	`YSYX219999_FALSE_V;

	//	获得数据存储器的访问地址
	assign raddr	=	mem_wd_i;
	assign waddr	=	mem_wd_i;

	//	获得数据存储器的访问类型
	assign rsize	=	(inst_ld)	?	`SIZE_D	:
						(inst_lw | inst_lwu)	?	`SIZE_W	:
						(inst_lh | inst_lhu)	?	`SIZE_H	:
						(inst_lb | inst_lbu)	?	`SIZE_B	:	`SIZE_D	;

	//	获得数据存储器的访问数据
	assign mem_dm_o =	(read_handshake_done == `YSYX219999_TRUE_V)	? data_read	:	`YSYX219999_ZERO_WORD;

	//	读数据存储器产生的流水线暂停信号
	assign	d_stall	=	((d_wce == `YSYX219999_TRUE_V && write_is_down == `YSYX219999_FALSE_V && waddr[27:16] != `CLINT_ADDR) || (d_rce == `YSYX219999_TRUE_V && read_handshake_done == `YSYX219999_FALSE_V))	?	`STOP	:	`NOSTOP;

	/******************************************axi读数据存储器信号***************************************/

	//	获得数据存储器读字节选择信号
    assign dre[0]	= inst_lb | inst_lbu;
	assign dre[1]	= inst_lh | inst_lhu;
	assign dre[2]	= inst_lw | inst_lwu;
	assign dre[3]	= inst_ld;

    // 	获得数据存储器写使能信号
    assign d_wce   = 
    (inst_sb | inst_sw | inst_sh | inst_sd);

    // 获得数据存储器写字节使能信号
	assign wmask[0] = ((inst_sb & (mem_wd_i[2 : 0] == 3'b000)) | (inst_sw & (mem_wd_i[2 : 0] == 3'b000)) | (inst_sh & (mem_wd_i[2 : 0] == 3'b000)) | inst_sd)	?	1'b1	:	1'b0;
	assign wmask[1] = ((inst_sb & (mem_wd_i[2 : 0] == 3'b001)) | (inst_sw & (mem_wd_i[2 : 0] == 3'b000)) | (inst_sh & (mem_wd_i[2 : 0] == 3'b000)) | inst_sd)	?	1'b1	:	1'b0;
	assign wmask[2] = ((inst_sb & (mem_wd_i[2 : 0] == 3'b010)) | (inst_sw & (mem_wd_i[2 : 0] == 3'b000)) | (inst_sh & (mem_wd_i[2 : 0] == 3'b010)) | inst_sd)	?	1'b1	:	1'b0;
	assign wmask[3] = ((inst_sb & (mem_wd_i[2 : 0] == 3'b011)) | (inst_sw & (mem_wd_i[2 : 0] == 3'b000)) | (inst_sh & (mem_wd_i[2 : 0] == 3'b010)) | inst_sd)	?	1'b1	:	1'b0;
    assign wmask[4] = ((inst_sb & (mem_wd_i[2 : 0] == 3'b100)) | (inst_sw & (mem_wd_i[2 : 0] == 3'b100)) | (inst_sh & (mem_wd_i[2 : 0] == 3'b100)) | inst_sd)	?	1'b1	:	1'b0;
    assign wmask[5] = ((inst_sb & (mem_wd_i[2 : 0] == 3'b101)) | (inst_sw & (mem_wd_i[2 : 0] == 3'b100)) | (inst_sh & (mem_wd_i[2 : 0] == 3'b100)) | inst_sd)	?	1'b1	:	1'b0;
    assign wmask[6] = ((inst_sb & (mem_wd_i[2 : 0] == 3'b110)) | (inst_sw & (mem_wd_i[2 : 0] == 3'b100)) | (inst_sh & (mem_wd_i[2 : 0] == 3'b110)) | inst_sd)	?	1'b1	:	1'b0;
    assign wmask[7] = ((inst_sb & (mem_wd_i[2 : 0] == 3'b111)) | (inst_sw & (mem_wd_i[2 : 0] == 3'b100)) | (inst_sh & (mem_wd_i[2 : 0] == 3'b110)) | inst_sd)	?	1'b1	:	1'b0;
                   
    // 确定待写入数据存储器的数据
    assign  wdata   = (d_wce == `YSYX219999_FALSE_V)			?	`YSYX219999_ZERO_WORD	:
					  (wmask == 8'b1111_1111      ) ? mem_din_i  :
                      (wmask == 8'b1111_0000       ) ? {mem_din_i[31:0],{32{1'b0}}} :
					  (wmask == 8'b0000_1111       ) ? {{32{1'b0}},mem_din_i[31:0]} :
                      (wmask == 8'b1000_0000       ) ? {mem_din_i[7:0],56'b0} : 
                      (wmask == 8'b0100_0000       ) ? {{8'b0},mem_din_i[7:0],{48'b0}} :
                      (wmask == 8'b0010_0000       ) ? {{16'b0},mem_din_i[7:0],{40'b0}} :
                      (wmask == 8'b0001_0000       ) ? {{24'b0},mem_din_i[7:0],{32'b0}} : 
					  (wmask == 8'b0000_1000       ) ? {{32'b0},mem_din_i[7:0],{24'b0}} : 
                      (wmask == 8'b0000_0100       ) ? {{40'b0},mem_din_i[7:0],{16'b0}} :
                      (wmask == 8'b0000_0010       ) ? {{48'b0},mem_din_i[7:0],{8'b0}} :
                      (wmask == 8'b0000_0001       ) ? {{56'b0},mem_din_i[7:0]} : 
                      (wmask == 8'b1100_0000       ) ? {mem_din_i[15:0], {48{1'b0}}} :
                      (wmask == 8'b0011_0000      ) ? {{16{1'b0}}, mem_din_i[15:0],{32{1'b0}}}  :
					  (wmask == 8'b0000_1100      ) ? {{32{1'b0}}, mem_din_i[15:0],{16{1'b0}}}  :
					  (wmask == 8'b0000_0011       ) ? {{48{1'b0}}, mem_din_i[15:0]}    : `YSYX219999_ZERO_WORD;

endmodule

module	ysyx_219999_memwb_reg(
	input		wire				cpu_clk_50M,
	input		wire				cpu_rst_n,
	
	//来自访存阶段的信息
	input 		wire[`REG_ADDR_BUS]	mem_wa,
	input		wire				mem_wreg,
	input		wire[`REG_BUS]		mem_dreg,
	input		wire 				mem_mreg,
	input		wire[`YSYX219999_BSEL_BUS]		mem_dre,
	input		wire 				mem_sign,
	input		wire[`YSYX219999_WORD_BUS]		mem_dm,
	
	//送至写回阶段的信息
	output		reg[`REG_ADDR_BUS]	wb_wa,
	output		reg					wb_wreg,
	output		reg[`REG_BUS]		wb_dreg,
	output		reg 				wb_mreg,
	output		reg[`YSYX219999_BSEL_BUS]		wb_dre,
	output		reg 				wb_sign,
	output		reg[`YSYX219999_WORD_BUS]		wb_dm,

	//访存产生的暂停信号
	input	wire 					data_read_stall,

	//csr指令添加
	input	wire 					mem_csr_we,
	input	wire[`CSR_ADDR_BUS]		mem_csr_waddr,
	input	wire[`REG_BUS	 ]		mem_csr_wdata,

	output	reg 					wb_csr_we,
	output	reg[`CSR_ADDR_BUS]		wb_csr_waddr,
	output	reg[`REG_BUS	 ]		wb_csr_wdata
	
);
	
	always@(posedge cpu_clk_50M) begin
		//复位的时候将送至写回阶段的信息清0
		if(cpu_rst_n == `YSYX219999_RST_ENABLE) begin
			wb_wa		<=			`REG_YSYX219999_NOP;
			wb_wreg		<=			`YSYX219999_WRITE_DISABLE;
			wb_dreg		<=			`YSYX219999_ZERO_WORD;
			wb_mreg		<=			`YSYX219999_WRITE_DISABLE;
			wb_dre		<=			4'b0;
			wb_sign		<=			`YSYX219999_FALSE_V;
			wb_dm		<=			`YSYX219999_ZERO_WORD;
			wb_csr_we	<=			`YSYX219999_FALSE_V;
			wb_csr_waddr<=			4'b0;
			wb_csr_wdata<=			`YSYX219999_ZERO_WORD;
		end
		//将来自访存阶段的信息寄存并送至写回阶段
		else if(data_read_stall == `NOSTOP) begin
			wb_wa		<=			mem_wa;
			wb_wreg		<=			mem_wreg;
			wb_dreg		<=			mem_dreg;
			wb_mreg		<=			mem_mreg;
			wb_dre		<=			mem_dre;
			wb_sign		<=			mem_sign;
			wb_dm		<=			mem_dm;
			wb_csr_we	<=			mem_csr_we;
			wb_csr_waddr<=			mem_csr_waddr;
			wb_csr_wdata<=			mem_csr_wdata;
		end
	end
	
endmodule

module ysyx_219999_regfile(
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
	output reg   [`REG_BUS 	   ]    rd2
	
);

    //定义32个64位寄存器
    reg [`REG_BUS] 	regs[0:`REG_NUM-1];
    
	
	always @(posedge cpu_clk_50M) begin
		if (cpu_rst_n == `YSYX219999_RST_ENABLE) begin
			regs[ 0] <= `YSYX219999_ZERO_WORD;
			regs[ 1] <= `YSYX219999_ZERO_WORD;
			regs[ 2] <= `YSYX219999_ZERO_WORD;
			regs[ 3] <= `YSYX219999_ZERO_WORD;
			regs[ 4] <= `YSYX219999_ZERO_WORD;
			regs[ 5] <= `YSYX219999_ZERO_WORD;
			regs[ 6] <= `YSYX219999_ZERO_WORD;
			regs[ 7] <= `YSYX219999_ZERO_WORD;
			regs[ 8] <= `YSYX219999_ZERO_WORD;
			regs[ 9] <= `YSYX219999_ZERO_WORD;
			regs[10] <= `YSYX219999_ZERO_WORD;
			regs[11] <= `YSYX219999_ZERO_WORD;
			regs[12] <= `YSYX219999_ZERO_WORD;
			regs[13] <= `YSYX219999_ZERO_WORD;
			regs[14] <= `YSYX219999_ZERO_WORD;
			regs[15] <= `YSYX219999_ZERO_WORD;
			regs[16] <= `YSYX219999_ZERO_WORD;
			regs[17] <= `YSYX219999_ZERO_WORD;
			regs[18] <= `YSYX219999_ZERO_WORD;
			regs[19] <= `YSYX219999_ZERO_WORD;
			regs[20] <= `YSYX219999_ZERO_WORD;
			regs[21] <= `YSYX219999_ZERO_WORD;
			regs[22] <= `YSYX219999_ZERO_WORD;
			regs[23] <= `YSYX219999_ZERO_WORD;
			regs[24] <= `YSYX219999_ZERO_WORD;
			regs[25] <= `YSYX219999_ZERO_WORD;
			regs[26] <= `YSYX219999_ZERO_WORD;
			regs[27] <= `YSYX219999_ZERO_WORD;
			regs[28] <= `YSYX219999_ZERO_WORD;
			regs[29] <= `YSYX219999_ZERO_WORD;
			regs[30] <= `YSYX219999_ZERO_WORD;
			regs[31] <= `YSYX219999_ZERO_WORD;
		end
		else begin
			if ((we == `YSYX219999_WRITE_ENABLE) && (wa != 5'h0))	
				regs[wa] <= wd;
		end
	end
	
	//读端口1的读操作 
	// ra1是读地址、wa是写地址、we是写使能、wd是要写入的数据 
	always @(*) begin
		if (cpu_rst_n == `YSYX219999_RST_ENABLE)
			rd1 = `YSYX219999_ZERO_WORD;
		else if (ra1 == `REG_YSYX219999_NOP)
			rd1 = `YSYX219999_ZERO_WORD;
		else if ((we == `YSYX219999_WRITE_ENABLE) && (wa == ra1))
		    rd1 = wd;
		else 
			rd1 = regs[ra1];
	end
	
	//读端口2的读操作 
	// ra2是读地址、wa是写地址、we是写使能、wd是要写入的数据 
	always @(*) begin
		if (cpu_rst_n == `YSYX219999_RST_ENABLE)
			rd2 = `YSYX219999_ZERO_WORD;
		else if (ra2 == `REG_YSYX219999_NOP)
			rd2 = `YSYX219999_ZERO_WORD;
		else if ((we == `YSYX219999_WRITE_ENABLE) && (wa == ra2))
		    rd2 = wd;
		else 
			rd2 = regs[ra2];
	end

endmodule

 
 module ysyx_219999_rvcpu(
	input		wire					cpu_clk_50M,
	input 		wire					cpu_rst_n,
	

    //传入axi的取指信号
    output                              if_valid,
    input                               if_ready,
    input  [31: 0]                      if_data_read,
    output [31: 0]                      if_addr,
    output [ 1: 0]                      if_size,
    input  [ 1: 0]                      if_resp,

    //传入axi的数据读信号
    output                              d_rvalid,
    input                               d_rready,
    input  [63: 0]                      d_data_read,
    output [63: 0]                      d_raddr,
    output [ 1: 0]                      d_rsize,
    input  [ 1: 0]                      d_resp,

    //传入axi的数据写信号
    output      wire                    d_wce,
    output      wire[ 7: 0]             d_wmask,
    output      wire[`YSYX219999_WORD_BUS]         d_waddr,
    output      wire[`YSYX219999_WORD_BUS]         d_wdata,
    input       wire                    write_is_down,

    //clint模块传出的时钟中断信号
    input       wire                    clock_interr,
    output      wire                    interr_isdone
	

);

    //连接取指阶段到取指译码寄存器阶段的信号
    wire[`YSYX219999_INST_ADDR_BUS]        pc;
    wire[`YSYX219999_INST_BUS]             if_inst_o;

    wire                        handshake_done; //取指完成信号

    //连接取指译码寄存器到译码阶段的信号
    wire[`YSYX219999_INST_ADDR_BUS]        id_pc_i;
    wire[`YSYX219999_INST_BUS]             id_inst_i;

    //连接译码阶段到寄存器堆的信号
    wire[`REG_ADDR_BUS]         ra1;
    wire[`REG_ADDR_BUS]         ra2;
    wire[`REG_BUS]              rd1;
    wire[`REG_BUS]              rd2;

    //转移指令添加
    wire [`YSYX219999_INST_ADDR_BUS ] 	if_pc_plus_4;
    wire [`YSYX219999_INST_ADDR_BUS ] 	id_pc_plus_4;
    wire [`REG_BUS       ] 	id_ret_addr;
    wire [`REG_BUS       ] 	exe_ret_addr;

    //连接译码阶段到译码执行寄存器的信号
    wire[`YSYX219999_ALUTYPE_BUS]          id_alutype_o;
    wire[`YSYX219999_ALUOP_BUS]            id_aluop_o;
    wire[`REG_ADDR_BUS]         id_wa_o;
    wire                        id_wreg_o;
    wire[`REG_BUS]              id_src1_o;
    wire[`REG_BUS]              id_src2_o;
    wire                        id_mreg_o;
    wire[`REG_BUS]              id_din_o;
    wire[`YSYX219999_INST_ADDR_BUS]        id_pc_o;

    //连接译码执行寄存器到执行阶段的信号
    wire[`YSYX219999_ALUTYPE_BUS]          exe_alutype_i;
    wire[`YSYX219999_ALUOP_BUS]            exe_aluop_i;
    wire[`REG_ADDR_BUS]         exe_wa_i;
    wire                        exe_wreg_i;
    wire[`REG_BUS]              exe_src1_i;
    wire[`REG_BUS]              exe_src2_i;
    wire                        exe_mreg_i;
    wire[`REG_BUS]              exe_din_i;
    wire[`YSYX219999_INST_ADDR_BUS]        exe_pc_i;

    //连接执行阶段到执行访存寄存器的信号
    wire[`YSYX219999_ALUOP_BUS]            exe_aluop_o; 
    wire[`REG_ADDR_BUS]         exe_wa_o;
    wire                        exe_wreg_o;
    wire[`REG_BUS]              exe_wd_o;
    wire                        exe_mreg_o;
    wire[`REG_BUS]              exe_din_o;

    //连接执行访存寄存器到访存阶段的信号
    wire[`YSYX219999_ALUOP_BUS]            mem_aluop_i;
    wire[`REG_ADDR_BUS]         mem_wa_i;
    wire                        mem_wreg_i;
    wire[`REG_BUS]              mem_wd_i;
    wire                        mem_mreg_i;
    wire[`REG_BUS]              mem_din_i;

    //连接访存阶段到访存写回寄存器的信号
    wire[`REG_ADDR_BUS]         mem_wa_o;
    wire                        mem_wreg_o;
    wire[`REG_BUS]              mem_dreg_o;
    wire                        mem_mreg_o;
    wire[`YSYX219999_BSEL_BUS]             mem_dre_o;
    wire                        mem_sign_o;
    wire[`YSYX219999_WORD_BUS]             mem_dm_o;


    //连接访存写回寄存器与写回阶段的信号
    wire[`REG_ADDR_BUS]         wb_wa_i;
    wire                        wb_wreg_i;
    wire[`REG_BUS]              wb_dreg_i;
    wire                        wb_mreg_i;
    wire[`YSYX219999_BSEL_BUS]             wb_dre_i;
    wire                        wb_sign_i;
    wire[`YSYX219999_WORD_BUS]             wb_dm_i;
	
	//连接写回阶段与通用寄存器堆的信号
	wire[`REG_ADDR_BUS]			wb_wa_o;
	wire						wb_wreg_o;
	wire[`REG_BUS]				wb_wd_o;
    
    /************************流水线暂停信号**********************/

    wire                        id_stall;
    wire                        data_read_stall;

    /************************流水线暂停信号**********************/

    /************************分支预测**********************/
	//btb到取指阶段的信号
	wire                   btb_jump_ena;
	wire[`YSYX219999_INST_ADDR_BUS]   btb_prepc_o;
	
	//取指到取指/译码寄存器的信号
	wire                   if_jump_ena;
	wire[`YSYX219999_INST_ADDR_BUS]   if_jump_pc;
	
	//取指/译码寄存器到译码阶段的信号
	wire                   id_jump_ena;
	wire[`YSYX219999_INST_ADDR_BUS]   id_jump_pc;
	
	//译码阶段到btb的更新信号
	wire                   btb_update_req;
	wire                   btb_update_state;
	wire[`PC_ADDR_BTB  ]   btb_update_pc;
	wire[`YSYX219999_INST_ADDR_BUS]   btb_update_prepc;
	
	//译码阶段传出的流水线冲刷信号
	wire                   flush;
	wire[`YSYX219999_INST_ADDR_BUS]   flush_addr;

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
    wire[`YSYX219999_INST_ADDR_BUS]    excep_flush_pc;
    wire[`EXCEP_CODE_BUS]   excep_code;


    /************************异常指令添加*********************/


    ysyx_219999_if_stage    if_stage0(
        .cpu_clk_50M(cpu_clk_50M),.cpu_rst_n(cpu_rst_n),
        .pc_plus_4(if_pc_plus_4),.if_pc_o(pc),
        .if_inst_o(if_inst_o),
        //连接axi的取指信号
        .if_valid(if_valid),.if_ready(if_ready),.if_data_read(if_data_read),
        .if_addr(if_addr),.if_size(if_size),.if_resp(if_resp),
        //取指完成信号
        .handshake_done(handshake_done),
        //流水线暂停信号
        .stall(data_read_stall),
        //分支预测添加
		.if_pre_jumpena(btb_jump_ena),.if_pre_pc(btb_prepc_o),
		.jump_ena_o(if_jump_ena),.jump_pc_o(if_jump_pc),
		//预测错误回传信号
		.flush(flush),.flush_addr(flush_addr),
        //异常处理冲刷信号添加
        .excep_flush(excep_flush),.excep_flush_pc(excep_flush_pc)
    );

    ysyx_219999_ifid_reg    ifid_reg0(
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

    ysyx_219999_id_stage    id_stage0(
        .cpu_rst_n(cpu_rst_n),
        .id_pc_i(id_pc_i),.id_inst_i(id_inst_i),
        .pc_plus_4(id_pc_plus_4),
        .rd1(rd1),.rd2(rd2),
        .ra1(ra1),.ra2(ra2),
        .id_alutype_o(id_alutype_o),.id_aluop_o(id_aluop_o),
        .id_wreg_o(id_wreg_o),.id_wa_o(id_wa_o),
        .id_pc_o(id_pc_o),
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
    
        //csr指令添加
        .csr_raddr(csr_raddr),.csr_rdata(csr_rdata),.id_csr_waddr(id_csr_waddr_o),
        .exe2id_csr_we(exe_csr_we_o),.exe2id_csr_waddr(exe_csr_waddr_o),.exe2id_csr_wdata(exe_csr_wdata_o),
        .mem2id_csr_we(mem_csr_we_o),.mem2id_csr_waddr(mem_csr_waddr_o),.mem2id_csr_wdata(mem_csr_wdata_o)
    );

    ysyx_219999_idexe_reg   idexe_reg0(
        .cpu_clk_50M(cpu_clk_50M),.cpu_rst_n(cpu_rst_n),
        .id_alutype(id_alutype_o),.id_aluop(id_aluop_o),
        .id_wa(id_wa_o),.id_wreg(id_wreg_o),
        .id_src1(id_src1_o),.id_src2(id_src2_o),
        .id_mreg(id_mreg_o),.id_din(id_din_o),
        .id_pc(id_pc_o),
        .exe_alutype(exe_alutype_i),.exe_aluop(exe_aluop_i),
        .exe_wa(exe_wa_i),.exe_wreg(exe_wreg_i),
        .exe_src1(exe_src1_i),.exe_src2(exe_src2_i),
        .exe_mreg(exe_mreg_i),.exe_din(exe_din_i),
		.id_ret_addr(id_ret_addr),.exe_ret_addr(exe_ret_addr),
        .exe_pc(exe_pc_i),
        //流水线暂停信号
        .id_stall(id_stall),.data_read_stall(data_read_stall),
        //csr指令添加
        .id_csr_waddr(id_csr_waddr_o),.exe_csr_waddr(exe_csr_waddr_i),
        //异常处理flush冲刷信号
        .excep_flush(excep_flush)
    );

    ysyx_219999_exe_stage   exe_stage0(
        .exe_alutype_i(exe_alutype_i),.exe_aluop_i(exe_aluop_i),
        .exe_wa_i(exe_wa_i),.exe_wreg_i(exe_wreg_i),
        .exe_src1_i(exe_src1_i),.exe_src2_i(exe_src2_i),
        .exe_mreg_i(exe_mreg_i),.exe_din_i(exe_din_i),
        .ret_addr(exe_ret_addr),
        .exe_pc_i(exe_pc_i),
        .exe_aluop_o(exe_aluop_o),
        .exe_wa_o(exe_wa_o),.exe_wreg_o(exe_wreg_o),.exe_wd_o(exe_wd_o),
        .exe_mreg_o(exe_mreg_o),.exe_din_o(exe_din_o),
        //csr指令添加
        .exe_csr_waddr_i(exe_csr_waddr_i),
        .exe_csr_waddr(exe_csr_waddr_o),.exe_csr_wdata(exe_csr_wdata_o),.exe_csr_we(exe_csr_we_o),
        //异常指令添加
        .global_trap_ena(global_trap_ena),.time_trap_ena(time_trap_ena),
        .mtvec_read(ecall_mtvec_read),.mepc_read(mret_mepc_read),
        .excep_flush(excep_flush),.excep_flush_pc(excep_flush_pc),.excep_code(excep_code),
        //clint模块传出的时钟中断信号
        .clock_interr(clock_interr),.interr_isdone(interr_isdone)     
    );

    ysyx_219999_exemem_reg  exemem_reg0(
        .cpu_clk_50M(cpu_clk_50M),.cpu_rst_n(cpu_rst_n),
        .exe_aluop(exe_aluop_o),
        .exe_wa(exe_wa_o),.exe_wreg(exe_wreg_o),.exe_wd(exe_wd_o),
        .exe_mreg(exe_mreg_o),.exe_din(exe_din_o),
        .mem_aluop(mem_aluop_i),
        .mem_wa(mem_wa_i),.mem_wreg(mem_wreg_i),.mem_wd(mem_wd_i),
        .mem_mreg(mem_mreg_i),.mem_din(mem_din_i),
        //流水线暂停信号
        .data_read_stall(data_read_stall),
        //csr指令信号
        .exe_csr_we(exe_csr_we_o),.exe_csr_waddr(exe_csr_waddr_o),.exe_csr_wdata(exe_csr_wdata_o),
        .mem_csr_we(mem_csr_we_i),.mem_csr_waddr(mem_csr_waddr_i),.mem_csr_wdata(mem_csr_wdata_i),
        //异常处理flush冲刷信号
        .excep_flush(excep_flush)
    );

    ysyx_219999_mem_stage   mem_stage0(
        .mem_aluop_i(mem_aluop_i),
        .mem_wa_i(mem_wa_i),.mem_wreg_i(mem_wreg_i),.mem_wd_i(mem_wd_i),
        .mem_mreg_i(mem_mreg_i),.mem_din_i(mem_din_i),
        .mem_wa_o(mem_wa_o),.mem_wreg_o(mem_wreg_o),.mem_dreg_o(mem_dreg_o),
        .mem_mreg_o(mem_mreg_o),.dre(mem_dre_o),.sign(mem_sign_o),.mem_dm_o(mem_dm_o),
        //axi-data-read
        .rvalid(d_rvalid),.r_ready(d_rready),.data_read(d_data_read),
        .raddr(d_raddr),.rsize(d_rsize),.resp(d_resp),
        //axi-data-write
        .d_wce(d_wce),.waddr(d_waddr),.wdata(d_wdata),.wmask(d_wmask),.write_is_down(write_is_down),
        //stall
        .d_stall(data_read_stall),
        //csr指令添加
        .mem_csr_we_i(mem_csr_we_i),.mem_csr_waddr_i(mem_csr_waddr_i),.mem_csr_wdata_i(mem_csr_wdata_i),
        .mem_csr_we_o(mem_csr_we_o),.mem_csr_waddr_o(mem_csr_waddr_o),.mem_csr_wdata_o(mem_csr_wdata_o)
    );

    ysyx_219999_memwb_reg   memwb_reg0(
        .cpu_clk_50M(cpu_clk_50M),.cpu_rst_n(cpu_rst_n),
        .mem_wa(mem_wa_o),.mem_wreg(mem_wreg_o),.mem_dreg(mem_dreg_o),
        .mem_mreg(mem_mreg_o),.mem_dre(mem_dre_o),.mem_sign(mem_sign_o),
        .mem_dm(mem_dm_o),
        .wb_wa(wb_wa_i),.wb_wreg(wb_wreg_i),.wb_dreg(wb_dreg_i),
        .wb_mreg(wb_mreg_i),.wb_dre(wb_dre_i),.wb_sign(wb_sign_i),
        .wb_dm(wb_dm_i),
        //流水线暂停信号
        .data_read_stall(data_read_stall),
        //csr指令添加信号
        .mem_csr_we(mem_csr_we_o),.mem_csr_waddr(mem_csr_waddr_o),.mem_csr_wdata(mem_csr_wdata_o),
        .wb_csr_we(wb_csr_we_i),.wb_csr_waddr(wb_csr_waddr_i),.wb_csr_wdata(wb_csr_wdata_i)
    );

    ysyx_219999_wb_stage    wb_stage0(
        .wb_wa_i(wb_wa_i),.wb_wreg_i(wb_wreg_i),.wb_dreg_i(wb_dreg_i),
        .wb_mreg_i(wb_mreg_i),.wb_dre_i(wb_dre_i),.sign(wb_sign_i),
        .dm(wb_dm_i),
        .wb_wa_o(wb_wa_o),.wb_wreg_o(wb_wreg_o),.wb_wd_o(wb_wd_o),
        //csr指令添加信号
        .wb_csr_we_i(wb_csr_we_i),.wb_csr_waddr_i(wb_csr_waddr_i),.wb_csr_wdata_i(wb_csr_wdata_i),
        .csr_we(csr_we),.csr_waddr(csr_waddr),.csr_wdata(csr_wdata)
    );

    ysyx_219999_regfile     regfile0(
        .cpu_clk_50M(cpu_clk_50M),.cpu_rst_n(cpu_rst_n),
        .ra1(ra1),.ra2(ra2),
        .rd1(rd1),.rd2(rd2),
        .wa(wb_wa_o),.we(wb_wreg_o),.wd(wb_wd_o)
    );

    ysyx_219999_btb btb0(
	    .cpu_rst_n(cpu_rst_n),.cpu_clk_50M(cpu_clk_50M),
	    .pc_tag(pc[11:2]),
	    .btb_jump_ena(btb_jump_ena),.btb_prepc_o(btb_prepc_o),
	    .wr_req(btb_update_req),.wr_jump_state(btb_update_state),
	    .wr_pc_tag(btb_update_pc),.wr_predicted_pc(btb_update_prepc)
	           
	);

    ysyx_219999_csrfile     csrfile0(
        .cpu_clk_50M(cpu_clk_50M),.cpu_rst_n(cpu_rst_n),
        .csr_raddr(csr_raddr),.csr_rdata(csr_rdata),
        .csr_we(csr_we),.csr_waddr(csr_waddr),.csr_wdata(csr_wdata),
        //异常处理指令添加
        .global_trap_ena(global_trap_ena),.time_trap_ena(time_trap_ena),
        .excep_mtvec(ecall_mtvec_read),.excep_mret_mepc(mret_mepc_read),
        .excep_code(excep_code)
    );

endmodule

`define AXI_TOP_INTERFACE(name) io_memAXI_0_``name

module ysyx_219999(
    input         clock,
    input         reset,

    input         io_interrupt,

	input         io_master_awready,
	output        io_master_awvalid,
	output [31:0] io_master_awaddr,
	output [3:0]  io_master_awid,
	output [7:0]  io_master_awlen,
	output [2:0]  io_master_awsize,
	output [1:0]  io_master_awburst,
    
    input         io_master_wready,
	output        io_master_wvalid,
	output [63:0] io_master_wdata,
	output [7:0]  io_master_wstrb,
	output        io_master_wlast,
    
    output        io_master_bready,
	input         io_master_bvalid,
	input  [1:0]  io_master_bresp,
	input  [3:0]  io_master_bid,

    input         io_master_arready,
	output        io_master_arvalid,
	output [31:0] io_master_araddr,
	output [3:0]  io_master_arid,
	output [7:0]  io_master_arlen,
	output [2:0]  io_master_arsize,
	output [1:0]  io_master_arburst,
    
    output        io_master_rready,
	input         io_master_rvalid,
	input  [1:0]  io_master_rresp,
	input  [63:0] io_master_rdata,
	input         io_master_rlast,
	input  [3:0]  io_master_rid,

    output        io_slave_awready,
    input         io_slave_awvalid,
    input  [31:0] io_slave_awaddr,
    input  [3:0]  io_slave_awid,
    input  [7:0]  io_slave_awlen,
    input  [2:0]  io_slave_awsize,
    input  [1:0]  io_slave_awburst,

    output        io_slave_wready,
    input         io_slave_wvalid,
    input  [63:0] io_slave_wdata,
    input  [7:0]  io_slave_wstrb,
    input         io_slave_wlast,

    input         io_slave_bready,
    output        io_slave_bvalid,
    output [1:0]  io_slave_bresp,
    output [3:0]  io_slave_bid,

    output        io_slave_arready,
    input         io_slave_arvalid,
    input  [31:0] io_slave_araddr,
    input  [3:0]  io_slave_arid,
    input  [7:0]  io_slave_arlen,
    input  [2:0]  io_slave_arsize,
    input  [1:0]  io_slave_arburst,

    input         io_slave_rready,
    output        io_slave_rvalid,
    output [1:0]  io_slave_rresp,
    output [63:0] io_slave_rdata,
    output        io_slave_rlast,
    output [3:0]  io_slave_rid
);


    //连接axi总线的信号
    wire aw_ready;
    wire aw_valid;
    wire [`AXI_ADDR_WIDTH-1:0] aw_addr;
    wire [`AXI_ID_WIDTH-1:0] aw_id;
    wire [7:0] aw_len;
    wire [2:0] aw_size;
    wire [1:0] aw_burst;

    wire w_ready;
    wire w_valid;
    wire [`AXI_DATA_WIDTH-1:0] w_data;
    wire [`AXI_DATA_WIDTH/8-1:0] w_strb;
    wire w_last;
    
    wire b_ready;
    wire b_valid;
    wire [1:0] b_resp;
    wire [`AXI_ID_WIDTH-1:0] b_id;

    wire ar_ready;
    wire ar_valid;
    wire [`AXI_ADDR_WIDTH-1:0] ar_addr;
    wire [`AXI_ID_WIDTH-1:0] ar_id;
    wire [7:0] ar_len;
    wire [2:0] ar_size;
    wire [1:0] ar_burst;
    
    wire r_ready;
    wire r_valid;
    wire [1:0] r_resp;
    wire [`AXI_DATA_WIDTH-1:0] r_data;
    wire r_last;
    wire [`AXI_ID_WIDTH-1:0] r_id;

    //if-read
    wire if_valid;
    wire if_ready;
    wire [31: 0]                   if_data_read;
    wire [`AXI_ADDR_WIDTH-1 : 0]   if_addr;
    wire [1:0] if_size;
    wire [1:0] if_resp;

    //mem-read
    wire d_rvalid;
    wire d_rready;
    wire [`YSYX219999_WORD_BUS]  d_data_read;
    wire [`YSYX219999_WORD_BUS]  d_raddr;
    wire [ 1: 0]      d_rsize;
    wire [ 1: 0]      d_resp;

    //mem-write
    wire [`YSYX219999_WORD_BUS]        d_waddr;
    wire [`YSYX219999_WORD_BUS]        data_write;
    wire [ 7: 0]            d_wmask;
    wire                    d_wce;
    wire                    write_is_down;
    wire w_req = `REQ_WRITE;

    //axi-connect
    wire r_valid_o;
    wire r_ready_i;
    wire r_req = `REQ_READ;
    wire [`YSYX219999_WORD_BUS]      data_read_i;
    wire [`AXI_ADDR_WIDTH-1 : 0]      r_addr_o;
    wire [ 1: 0]          r_size_o;
    wire [ 1: 0]          r_resp_i;

    reg [`YSYX219999_WORD_BUS]        axi_d_waddr;
    reg [`YSYX219999_WORD_BUS]        axi_data_write;
    reg [ 7: 0]            axi_d_wmask;
    reg                    axi_d_wce;

    assign ar_ready                                 = io_master_arready;
    assign io_master_arvalid                        = ar_valid;
    assign io_master_araddr                         = ar_addr;
    assign io_master_arid                           = ar_id;
    assign io_master_arlen                          = ar_len;
    assign io_master_arsize                         = ar_size;
    assign io_master_arburst                        = ar_burst;
    
    assign io_master_rready                         = r_ready;
    assign r_valid                                  = io_master_rvalid;
    assign r_resp                                   = io_master_rresp;
    assign r_data                                   = io_master_rdata;
    assign r_last                                   = io_master_rlast;
    assign r_id                                     = io_master_rid;

    assign aw_ready                                 = io_master_awready;
    assign io_master_awvalid                        = aw_valid;
    assign io_master_awaddr                         = aw_addr;
    assign io_master_awid                           = aw_id;
    assign io_master_awlen                          = aw_len;
    assign io_master_awsize                         = aw_size;
    assign io_master_awburst                        = aw_burst;

    assign w_ready                                  = io_master_wready;
    assign io_master_wvalid                         = w_valid;
    assign io_master_wdata                          = w_data;
    assign io_master_wstrb                          = w_strb;
    assign io_master_wlast                          = w_last;

    assign io_master_bready                         = b_ready;
    assign b_valid                                  = io_master_bvalid;
    assign b_resp                                   = io_master_bresp;
    assign b_id                                     = io_master_bid;

    assign io_slave_awready = 0;
    assign io_slave_wready  = 0;
    assign io_slave_bvalid  = 0;
    assign io_slave_bresp   = 2'b0;
    assign io_slave_bid     = 4'b0;
    assign io_slave_arready = 0;
    assign io_slave_rvalid  = 0;
    assign io_slave_rresp   = 2'b0;
    assign io_slave_rdata   = `YSYX219999_ZERO_WORD;
    assign io_slave_rlast   = 0;
    assign io_slave_rid     = 4'b0;

    ysyx_219999_axi_rw axi_rw0 (
        .clock                          (clock),
        .reset                          (reset),

        .r_valid_i                      (r_valid_o),
        .r_ready_o                      (r_ready_i),
        .r_req_i                        (r_req),
        .data_read_o                    (data_read_i),
        .r_addr_i                       (r_addr_o),
        .r_size_i                       (r_size_o),
        .r_resp_o                       (r_resp_i),

        .w_valid_i                      (1'b1),
        .w_req_i                        (w_req),
        .data_write_i                   (axi_data_write),
        .w_addr_i                       (axi_d_waddr),
        .wmask                          (axi_d_wmask),
        .w_ena                          (axi_d_wce),
        .write_is_down                  (write_is_down),

        .axi_aw_ready_i                 (aw_ready),
        .axi_aw_valid_o                 (aw_valid),
        .axi_aw_addr_o                  (aw_addr),
        .axi_aw_id_o                    (aw_id),
        .axi_aw_len_o                   (aw_len),
        .axi_aw_size_o                  (aw_size),
        .axi_aw_burst_o                 (aw_burst),

        .axi_w_ready_i                  (w_ready),
        .axi_w_valid_o                  (w_valid),
        .axi_w_data_o                   (w_data),
        .axi_w_strb_o                   (w_strb),
        .axi_w_last_o                   (w_last),

        
        .axi_b_ready_o                  (b_ready),
        .axi_b_valid_i                  (b_valid),
        .axi_b_resp_i                   (b_resp),
        .axi_b_id_i                     (b_id),

        .axi_ar_ready_i                 (ar_ready),
        .axi_ar_valid_o                 (ar_valid),
        .axi_ar_addr_o                  (ar_addr),
        .axi_ar_id_o                    (ar_id),
        .axi_ar_len_o                   (ar_len),
        .axi_ar_size_o                  (ar_size),
        .axi_ar_burst_o                 (ar_burst),
        
        .axi_r_ready_o                  (r_ready),
        .axi_r_valid_i                  (r_valid),
        .axi_r_resp_i                   (r_resp),
        .axi_r_data_i                   (r_data),
        .axi_r_last_i                   (r_last),
        .axi_r_id_i                     (r_id)
    );


    always @(posedge clock) begin
      if(d_wce && (d_waddr[27:16] != `CLINT_ADDR)) begin
        axi_d_waddr     <=  d_waddr;
        axi_data_write  <=  data_write;
        axi_d_wmask     <=  d_wmask;
        axi_d_wce       <=  d_wce;
      end
      else begin
        axi_d_wce       <=  `YSYX219999_FALSE_V;
      end
    end
    

    reg read_state;
    reg [4:0] count;

    always@(negedge clock) begin
      if(reset == `YSYX219999_RST_ENABLE) begin
        read_state  <=  `INST_READ;
        count       <=  0;
      end 
      else if(r_ready_i == `YSYX219999_TRUE_V && (d_rvalid == `YSYX219999_TRUE_V && d_waddr[27:16] != `CLINT_ADDR)) begin
        read_state  <=  `DATA_READ;
        count       <=  count + 1;
      end
      else if(d_rvalid == `YSYX219999_FALSE_V) begin
        read_state  <=  `INST_READ;
        count       <=  0;
      end
    end

    wire[`YSYX219999_WORD_BUS] clint_data_read;

    assign r_valid_o = (read_state == `DATA_READ)  ? d_rvalid  : if_valid;
    assign r_addr_o  = (read_state == `DATA_READ)  ? d_raddr[31: 0]   : if_addr;
    assign r_size_o  = (read_state == `DATA_READ)  ? d_rsize   : if_size;
    assign d_resp     = r_resp_i;
    assign d_rready   = (clint_data_read != `YSYX219999_ZERO_WORD) ? `YSYX219999_TRUE_V : 
                        (read_state == `DATA_READ && count != 1)  ? r_ready_i  : `YSYX219999_FALSE_V;
    assign d_data_read= (clint_data_read != `YSYX219999_ZERO_WORD) ? clint_data_read : 
                        (read_state == `DATA_READ)  ? data_read_i : `YSYX219999_ZERO_WORD;
    assign if_resp    = r_resp_i;
    assign if_ready   = (read_state == `INST_READ)  ? r_ready_i  : `YSYX219999_FALSE_V;
    assign if_data_read=(read_state == `INST_READ)  ? data_read_i[31: 0] : 32'b0;



    //时钟中断模块及添加信号
    
    wire            clint_clock_interr;
    wire            clint_interr_isdone;
 

    ysyx_219999_rvcpu   rvcpu0(
        .cpu_clk_50M(clock),.cpu_rst_n(reset),
        //传给axi总线的取指信号
        .if_valid(if_valid),.if_ready(if_ready),.if_data_read(if_data_read),
        .if_addr(if_addr),.if_size(if_size),.if_resp(if_resp),
        //传给axi总选的数据读信号
        .d_rvalid(d_rvalid),.d_rready(d_rready),.d_data_read(d_data_read),
        .d_raddr(d_raddr),.d_rsize(d_rsize),.d_resp(d_resp),
        //传给axi总线的数据写信号
        .d_wce(d_wce),.d_waddr(d_waddr),.d_wdata(data_write),.d_wmask(d_wmask),.write_is_down(write_is_down),
        //串口输出信号
        //时钟中断信号
        .clock_interr(clint_clock_interr),.interr_isdone(clint_interr_isdone)
        
    );


    
    wire clint_we = (d_wce == `YSYX219999_TRUE_V && (d_waddr[27:16] != `CLINT_ADDR)) ? `YSYX219999_TRUE_V : `YSYX219999_FALSE_V;
    ysyx_219999_clint clint0(
        .cpu_clk_50M(clock),.cpu_rst_n(reset),
        .read_en(d_rvalid),.write_en(clint_we),
        .read_addr(d_raddr),.write_addr(d_waddr),
        .data_read(clint_data_read),.data_write(data_write),
        .interr(clint_clock_interr),.interr_isdone(clint_interr_isdone)
    );


endmodule

module ysyx_219999_wb_stage(
	
	//从访存阶段获得的信息
	input	wire[`REG_ADDR_BUS]	wb_wa_i,
	input	wire				wb_wreg_i,
	input	wire[`REG_BUS]		wb_dreg_i,
	input	wire				wb_mreg_i,
	input	wire[`YSYX219999_BSEL_BUS]		wb_dre_i,
	input	wire 				sign,

	
	//写回目的寄存器的数据
	output	wire[`REG_ADDR_BUS]	wb_wa_o,
	output	wire				wb_wreg_o,
	output	wire[`YSYX219999_WORD_BUS]		wb_wd_o,

	//来自数据存储器的数据
	input	wire[`YSYX219999_WORD_BUS]		dm,

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
    assign  csr_wdata   =   (wb_csr_we_i == `YSYX219999_TRUE_V && wb_csr_waddr_i == `MSTATUS && (wb_csr_wdata_i[16:15] == 2'b11 || wb_csr_wdata_i[14:13] == 2'b11))    ?   {1'b1,wb_csr_wdata_i[62:0]} :
                            (wb_csr_we_i == `YSYX219999_TRUE_V && wb_csr_waddr_i == `MSTATUS)  ?   {1'b0,wb_csr_wdata_i[62:0]} : wb_csr_wdata_i;

	//传至通用寄存器堆的信息
	assign	wb_wa_o		=	wb_wa_i;
	assign	wb_wreg_o	=	wb_wreg_i;

    wire    [`YSYX219999_WORD_BUS] data    =   (wb_dre_i[3] == `YSYX219999_TRUE_V)    ?   dm  :
                                    (wb_dre_i[2] == `YSYX219999_TRUE_V && sign)    ?   {{32{dm[31]}},dm[31: 0]}    :
                                    (wb_dre_i[2] == `YSYX219999_TRUE_V && ~sign)   ?   {{32{1'b0}},dm[31: 0]}      :
                                    (wb_dre_i[1] == `YSYX219999_TRUE_V && sign)    ?   {{48{dm[15]}},dm[15: 0]}    :
                                    (wb_dre_i[1] == `YSYX219999_TRUE_V && ~sign)   ?   {{48{1'b0}},dm[15: 0]}      :
                                    (wb_dre_i[0] == `YSYX219999_TRUE_V && sign)    ?   {{56{dm[7]}},dm[ 7: 0]}    :
                                    (wb_dre_i[0] == `YSYX219999_TRUE_V && ~sign)   ?   {{56{1'b0}},dm[ 7: 0]}      :   `YSYX219999_ZERO_WORD;

    assign wb_wd_o = (wb_mreg_i == `YSYX219999_MREG_ENABLE) ?  data  :   wb_dreg_i;		
    					
endmodule
	
