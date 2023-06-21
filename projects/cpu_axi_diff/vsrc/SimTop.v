`include "defines.v"
`define AXI_TOP_INTERFACE(name) io_memAXI_0_``name

module SimTop (
    input         clock,
    input         reset,

    input  [63:0] io_logCtrl_log_begin,
    input  [63:0] io_logCtrl_log_end,
    input  [63:0] io_logCtrl_log_level,
    input         io_perfInfo_clean,
    input         io_perfInfo_dump,

    output        io_uart_out_valid,
    output [7:0]  io_uart_out_ch,
    output        io_uart_in_valid,
    input  [7:0]  io_uart_in_ch,

    input                               `AXI_TOP_INTERFACE(aw_ready),
    output                              `AXI_TOP_INTERFACE(aw_valid),
    output [`AXI_ADDR_WIDTH-1:0]        `AXI_TOP_INTERFACE(aw_bits_addr),
    output [2:0]                        `AXI_TOP_INTERFACE(aw_bits_prot),
    output [`AXI_ID_WIDTH-1:0]          `AXI_TOP_INTERFACE(aw_bits_id),
    output [`AXI_USER_WIDTH-1:0]        `AXI_TOP_INTERFACE(aw_bits_user),
    output [7:0]                        `AXI_TOP_INTERFACE(aw_bits_len),
    output [2:0]                        `AXI_TOP_INTERFACE(aw_bits_size),
    output [1:0]                        `AXI_TOP_INTERFACE(aw_bits_burst),
    output                              `AXI_TOP_INTERFACE(aw_bits_lock),
    output [3:0]                        `AXI_TOP_INTERFACE(aw_bits_cache),
    output [3:0]                        `AXI_TOP_INTERFACE(aw_bits_qos),
    
    input                               `AXI_TOP_INTERFACE(w_ready),
    output                              `AXI_TOP_INTERFACE(w_valid),
    output [`AXI_DATA_WIDTH-1:0]        `AXI_TOP_INTERFACE(w_bits_data)         [3:0],
    output [`AXI_DATA_WIDTH/8-1:0]      `AXI_TOP_INTERFACE(w_bits_strb),
    output                              `AXI_TOP_INTERFACE(w_bits_last),
     
    output                              `AXI_TOP_INTERFACE(b_ready),
    input                               `AXI_TOP_INTERFACE(b_valid),
    input  [1:0]                        `AXI_TOP_INTERFACE(b_bits_resp),
    input  [`AXI_ID_WIDTH-1:0]          `AXI_TOP_INTERFACE(b_bits_id),
    input  [`AXI_USER_WIDTH-1:0]        `AXI_TOP_INTERFACE(b_bits_user),

    input                               `AXI_TOP_INTERFACE(ar_ready),
    output                              `AXI_TOP_INTERFACE(ar_valid),
    output [`AXI_ADDR_WIDTH-1:0]        `AXI_TOP_INTERFACE(ar_bits_addr),
    output [2:0]                        `AXI_TOP_INTERFACE(ar_bits_prot),
    output [`AXI_ID_WIDTH-1:0]          `AXI_TOP_INTERFACE(ar_bits_id),
    output [`AXI_USER_WIDTH-1:0]        `AXI_TOP_INTERFACE(ar_bits_user),
    output [7:0]                        `AXI_TOP_INTERFACE(ar_bits_len),
    output [2:0]                        `AXI_TOP_INTERFACE(ar_bits_size),
    output [1:0]                        `AXI_TOP_INTERFACE(ar_bits_burst),
    output                              `AXI_TOP_INTERFACE(ar_bits_lock),
    output [3:0]                        `AXI_TOP_INTERFACE(ar_bits_cache),
    output [3:0]                        `AXI_TOP_INTERFACE(ar_bits_qos),
    
    output                              `AXI_TOP_INTERFACE(r_ready),
    input                               `AXI_TOP_INTERFACE(r_valid),
    input  [1:0]                        `AXI_TOP_INTERFACE(r_bits_resp),
    input  [`AXI_DATA_WIDTH-1:0]        `AXI_TOP_INTERFACE(r_bits_data)         [3:0],
    input                               `AXI_TOP_INTERFACE(r_bits_last),
    input  [`AXI_ID_WIDTH-1:0]          `AXI_TOP_INTERFACE(r_bits_id),
    input  [`AXI_USER_WIDTH-1:0]        `AXI_TOP_INTERFACE(r_bits_user)
);


    //连接axi总线的信号
    wire aw_ready;
    wire aw_valid;
    wire [`AXI_ADDR_WIDTH-1:0] aw_addr;
    wire [2:0] aw_prot;
    wire [`AXI_ID_WIDTH-1:0] aw_id;
    wire [`AXI_USER_WIDTH-1:0] aw_user;
    wire [7:0] aw_len;
    wire [2:0] aw_size;
    wire [1:0] aw_burst;
    wire aw_lock;
    wire [3:0] aw_cache;
    wire [3:0] aw_qos;
    wire [3:0] aw_region;

    wire w_ready;
    wire w_valid;
    wire [`AXI_DATA_WIDTH-1:0] w_data;
    wire [`AXI_DATA_WIDTH/8-1:0] w_strb;
    wire w_last;
    wire [`AXI_USER_WIDTH-1:0] w_user;
    
    wire b_ready;
    wire b_valid;
    wire [1:0] b_resp;
    wire [`AXI_ID_WIDTH-1:0] b_id;
    wire [`AXI_USER_WIDTH-1:0] b_user;

    wire ar_ready;
    wire ar_valid;
    wire [`AXI_ADDR_WIDTH-1:0] ar_addr;
    wire [2:0] ar_prot;
    wire [`AXI_ID_WIDTH-1:0] ar_id;
    wire [`AXI_USER_WIDTH-1:0] ar_user;
    wire [7:0] ar_len;
    wire [2:0] ar_size;
    wire [1:0] ar_burst;
    wire ar_lock;
    wire [3:0] ar_cache;
    wire [3:0] ar_qos;
    wire [3:0] ar_region;
    
    wire r_ready;
    wire r_valid;
    wire [1:0] r_resp;
    wire [`AXI_DATA_WIDTH-1:0] r_data;
    wire r_last;
    wire [`AXI_ID_WIDTH-1:0] r_id;
    wire [`AXI_USER_WIDTH-1:0] r_user;

    assign ar_ready                                 = `AXI_TOP_INTERFACE(ar_ready);
    assign `AXI_TOP_INTERFACE(ar_valid)             = ar_valid;
    assign `AXI_TOP_INTERFACE(ar_bits_addr)         = ar_addr;
    assign `AXI_TOP_INTERFACE(ar_bits_prot)         = ar_prot;
    assign `AXI_TOP_INTERFACE(ar_bits_id)           = ar_id;
    assign `AXI_TOP_INTERFACE(ar_bits_user)         = ar_user;
    assign `AXI_TOP_INTERFACE(ar_bits_len)          = ar_len;
    assign `AXI_TOP_INTERFACE(ar_bits_size)         = ar_size;
    assign `AXI_TOP_INTERFACE(ar_bits_burst)        = ar_burst;
    assign `AXI_TOP_INTERFACE(ar_bits_lock)         = ar_lock;
    assign `AXI_TOP_INTERFACE(ar_bits_cache)        = ar_cache;
    assign `AXI_TOP_INTERFACE(ar_bits_qos)          = ar_qos;
    
    assign `AXI_TOP_INTERFACE(r_ready)              = r_ready;
    assign r_valid                                  = `AXI_TOP_INTERFACE(r_valid);
    assign r_resp                                   = `AXI_TOP_INTERFACE(r_bits_resp);
    assign r_data                                   = `AXI_TOP_INTERFACE(r_bits_data)[0];
    assign r_last                                   = `AXI_TOP_INTERFACE(r_bits_last);
    assign r_id                                     = `AXI_TOP_INTERFACE(r_bits_id);
    assign r_user                                   = `AXI_TOP_INTERFACE(r_bits_user);

    assign aw_ready                                 = `AXI_TOP_INTERFACE(aw_ready);
    assign `AXI_TOP_INTERFACE(aw_valid)             = aw_valid;
    assign `AXI_TOP_INTERFACE(aw_bits_addr)         = aw_addr;
    assign `AXI_TOP_INTERFACE(aw_bits_prot)         = aw_prot;
    assign `AXI_TOP_INTERFACE(aw_bits_id)           = aw_id;
    assign `AXI_TOP_INTERFACE(aw_bits_user)         = aw_user;
    assign `AXI_TOP_INTERFACE(aw_bits_len)          = aw_len;
    assign `AXI_TOP_INTERFACE(aw_bits_size)         = aw_size;
    assign `AXI_TOP_INTERFACE(aw_bits_burst)        = aw_burst;
    assign `AXI_TOP_INTERFACE(aw_bits_lock)         = aw_lock;
    assign `AXI_TOP_INTERFACE(aw_bits_cache)        = aw_cache;
    assign `AXI_TOP_INTERFACE(aw_bits_qos)          = aw_qos;

    assign w_ready                                  = `AXI_TOP_INTERFACE(w_ready);
    assign `AXI_TOP_INTERFACE(w_valid)              = w_valid;
    assign `AXI_TOP_INTERFACE(w_bits_data)[0]       = w_data;
    assign `AXI_TOP_INTERFACE(w_bits_strb)          = w_strb;
    assign `AXI_TOP_INTERFACE(w_bits_last)          = w_last;

    assign `AXI_TOP_INTERFACE(b_ready)              = b_ready;
    assign b_valid                                  = `AXI_TOP_INTERFACE(b_valid);
    assign b_resp                                   = `AXI_TOP_INTERFACE(b_bits_resp);
    assign b_id                                     = `AXI_TOP_INTERFACE(b_bits_id);
    assign b_user                                   = `AXI_TOP_INTERFACE(b_bits_user);

    axi_rw axi_rw0 (
        .clock                          (clock),
        .reset                          (reset),

        .r_valid_i                      (r_valid_o),
        .r_ready_o                      (r_ready_i),
        .icache_ready                   (icache_ready),
        .r_req_i                        (r_req),
        .data_read_o                    (data_read_i),
        .r_addr_i                       (r_addr_o),
        .r_size_i                       (r_size_o),
        .r_resp_o                       (r_resp_i),
        .r_len_i                        (r_len_o),

        .w_valid_i                      (1'b1),
        .w_req_i                        (w_req),
        .data_write_i                   (data_write),
        .w_addr_i                       (d_waddr),
        .wmask                          (d_wmask),
        .w_ena                          (d_wce),
        //.write_is_down                  (write_is_down),

        .axi_aw_ready_i                 (aw_ready),
        .axi_aw_valid_o                 (aw_valid),
        .axi_aw_addr_o                  (aw_addr),
        .axi_aw_prot_o                  (aw_prot),
        .axi_aw_id_o                    (aw_id),
        .axi_aw_user_o                  (aw_user),
        .axi_aw_len_o                   (aw_len),
        .axi_aw_size_o                  (aw_size),
        .axi_aw_burst_o                 (aw_burst),
        .axi_aw_lock_o                  (aw_lock),
        .axi_aw_cache_o                 (aw_cache),
        .axi_aw_qos_o                   (aw_qos),
        .axi_aw_region_o                (aw_region),

        .axi_w_ready_i                  (w_ready),
        .axi_w_valid_o                  (w_valid),
        .axi_w_data_o                   (w_data),
        .axi_w_strb_o                   (w_strb),
        .axi_w_last_o                   (w_last),
        .axi_w_user_o                   (w_user),
        
        .axi_b_ready_o                  (b_ready),
        .axi_b_valid_i                  (b_valid),
        .axi_b_resp_i                   (b_resp),
        .axi_b_id_i                     (b_id),
        .axi_b_user_i                   (b_user),

        .axi_ar_ready_i                 (ar_ready),
        .axi_ar_valid_o                 (ar_valid),
        .axi_ar_addr_o                  (ar_addr),
        .axi_ar_prot_o                  (ar_prot),
        .axi_ar_id_o                    (ar_id),
        .axi_ar_user_o                  (ar_user),
        .axi_ar_len_o                   (ar_len),
        .axi_ar_size_o                  (ar_size),
        .axi_ar_burst_o                 (ar_burst),
        .axi_ar_lock_o                  (ar_lock),
        .axi_ar_cache_o                 (ar_cache),
        .axi_ar_qos_o                   (ar_qos),
        .axi_ar_region_o                (ar_region),
        
        .axi_r_ready_o                  (r_ready),
        .axi_r_valid_i                  (r_valid),
        .axi_r_resp_i                   (r_resp),
        .axi_r_data_i                   (r_data),
        .axi_r_last_i                   (r_last),
        .axi_r_id_i                     (r_id),
        .axi_r_user_i                   (r_user)
    );

    //if-read
    wire if_valid;
    wire if_ready;
    wire [`WORD_BUS]        if_data_read;
    wire [`INST_ADDR_BUS]   if_addr;
    wire [1:0] if_size;
    wire [1:0] if_len;
    wire [1:0] if_resp;
    wire cpu_icache_ready;

    //mem-read
    wire d_rvalid;
    wire d_rready;
    wire [`WORD_BUS]  d_data_read;
    wire [`WORD_BUS]  d_raddr;
    wire [ 1: 0]      d_rsize;
    wire [ 1: 0]      d_rlen;
    wire [ 1: 0]      d_resp;

    //mem-write
    wire [`WORD_BUS]        d_waddr;
    wire [`WORD_BUS]        data_write;
    wire [ 7: 0]            d_wmask;
    wire                    d_wce;
    wire                    axi_w_isdone;
    wire w_req = `REQ_WRITE;

    //axi-connect
    wire r_valid_o;
    wire r_ready_i;
    wire icache_ready;
    wire r_req = `REQ_READ;
    wire [`WORD_BUS]      data_read_i;
    wire [`WORD_BUS]      r_addr_o;
    wire [ 1: 0]          r_size_o;
    wire [ 1: 0]          r_resp_i;
    wire [ 1: 0]          r_len_o;

    parameter IDLE_READ = 2'b00,IF_READ = 2'b01,DATA_READ = 2'b10;
    wire  state_idle_read = read_state == IDLE_READ; 
    wire  state_if_read   = read_state == IF_READ;
    wire  state_data_read = read_state == DATA_READ;

    reg [1:0] read_state;

    always@(posedge clock) begin
      if(reset == `RST_ENABLE) begin
        read_state  <=  IDLE_READ;
      end 
      else begin
        case (read_state)
            IDLE_READ:  if(d_rvalid && d_raddr[27:16] != `CLINT_ADDR)      read_state <= DATA_READ;
                        else if(if_valid) read_state <= IF_READ;
            DATA_READ:  if(r_ready_i)     read_state <= IDLE_READ;
            IF_READ  :  if(r_ready_i)     read_state <= IDLE_READ;
            default:;
        endcase
      end
    end


    assign r_valid_o = (state_data_read)  ? d_rvalid  :
                       (state_if_read)    ? if_valid  : `FALSE_V;
    assign cpu_icache_ready = (state_data_read) ? `FALSE_V  : icache_ready;
    assign r_addr_o  = (state_data_read)  ? d_raddr   :
                       (state_if_read)    ? if_addr   : `PC_INIT;
    assign r_size_o  = (state_data_read)  ? d_rsize   : if_size;
    assign r_len_o   =  (state_data_read)  ? d_rlen     : if_len;
    assign d_resp     = r_resp_i;
    assign d_rready   = (clint_data_read != `ZERO_WORD) ? `TRUE_V : 
                        (state_data_read)  ? icache_ready  : `FALSE_V;
    assign d_data_read= (clint_data_read != `ZERO_WORD) ? clint_data_read : 
                        (state_data_read)  ? data_read_i : `ZERO_WORD;
    assign axi_w_isdone = (d_waddr == `MTIME_ADDR || d_waddr == `MTIMECMP_ADDR) ? clint_write_isdone  : b_valid;
    assign if_resp    = r_resp_i;
    assign if_ready   = (state_if_read)  ? r_ready_i  : `FALSE_V;
    assign if_data_read=(state_if_read)  ? data_read_i : `ZERO_WORD;

    reg test;                   //??????
    always @(posedge clock) begin
          if(reset) begin
            test  <= 1'b0;
          end
          else if(d_rready == 1'b1) begin
            test  <= 1'b1;
          end
    end

    //从寄存器堆到difftest的信号
    wire[`REG_BUS]  regs[0 : `REG_NUM - 1];
    wire[`REG_BUS]  csrs[0 : `CSR_NUM];

    //从写回阶段到difftest的写信号
    wire                    wb_wreg;
    wire[`REG_ADDR_BUS]     wb_wa;
    wire[`WORD_BUS]         wb_wd;

    //传给difftest的当前pc与指令值与异常编号
    wire[`INST_ADDR_BUS]    wb_pc;
    wire[`INST_BUS]         wb_inst;
    wire[`EXCEP_CODE_BUS]   wb_excep_code;
    wire                    wb2diff_skip;

    //传给串口的信号
    wire                    core_uart_ena;
    wire[ 7: 0]             core_uart_out;
    
    assign  io_uart_out_valid = (core_uart_ena == `TRUE_V)  ?  `TRUE_V : `FALSE_V;
    assign  io_uart_out_ch    = (core_uart_ena == `TRUE_V)  ?   core_uart_out  : 8'b0;

    rvcpu   rvcpu0(
        .cpu_clk_50M(clock),.cpu_rst_n(reset),
        //传给axi总线的取值信号
        .if_burst_valid(if_valid),.if_burst_ready(cpu_icache_ready),.if_burst_data(if_data_read),
        .if_burst_addr(if_addr),.if_burst_size(if_size),.if_burst_len(if_len),
        .if_resp(if_resp),
        //传给axi总选的数据读信号
        .mem_burst_valid(d_rvalid),.mem_burst_ready(d_rready),.mem_burst_data(d_data_read),
        .mem_burst_addr(d_raddr),.mem_burst_len(d_rlen),.mem_burst_size(d_rsize),.d_resp(d_resp),
        //传给axi总线的数据写信号
        .d_wvalid(d_wce),.d_waddr(d_waddr),.d_wdata(data_write),.d_wmask(d_wmask),
        .axi_w_isdone(axi_w_isdone),
        //to diff
        .wb_wreg_o(wb_wreg),.wb_wa_o(wb_wa),.wb_wd_o(wb_wd),
        .wb_pc_o(wb_pc),.wb_inst_o(wb_inst),
        .wb_excep_code_o(wb_excep_code),.wb2diff_skip_o(wb2diff_skip),
        .regs_o(regs),
        .csrs_o(csrs),
        //串口输出信号
        .uart_ena(core_uart_ena),.uart_data_out(core_uart_out),
        //时钟中断信号
        .clock_interr(clock_interr),.interr_isdone(interr_isdone)
        
    );


    //时钟中断模块及添加信号
    wire[`WORD_BUS] clint_data_read;
    wire            clint_write_isdone;
    wire            clock_interr;
    wire            interr_isdone;
    wire clint_we = (d_wce == `TRUE_V && (d_waddr[27:16] == `CLINT_ADDR)) ? `TRUE_V : `FALSE_V;
    clint clint0(
        .cpu_clk_50M(clock),.cpu_rst_n(reset),
        .read_en(d_rvalid),.write_en(clint_we),
        .read_addr(d_raddr),.write_addr(d_waddr),
        .data_read(clint_data_read),.data_write(data_write),.clint_write_isdone(clint_write_isdone),
        .interr(clock_interr),.interr_isdone(interr_isdone)
    );
    
// Difftest
reg cmt_wen;
reg [7:0] cmt_wdest;
reg [`REG_BUS] cmt_wdata;
reg [`REG_BUS] cmt_pc;
reg [`INST_BUS] cmt_inst;
reg cmt_valid;
reg skip_en;
reg [31:0]intrNO_en;
reg [4:0] cause_code;
reg trap;
reg [7:0] trap_code;
reg [`WORD_BUS] cycleCnt;
reg [`WORD_BUS] instrCnt;
reg [`REG_BUS] regs_diff [0 : `REG_NUM-1];
reg [`REG_BUS] csrs_diff [0 : `CSR_NUM];

wire inst_valid = (wb_pc != `PC_INIT) | (wb_inst != `ZERO_INST);
wire inst_skip  = (wb2diff_skip)  ? `TRUE_V : `FALSE_V;
wire [31:0]cpu_interr = (wb_excep_code == `TIME_TRAP_CODE) ?  32'h00000007 : 0;
//wire [31:0]cpu_cause  = (cpu_interr == `TRAP_ON) ?  7 : 0;
always @(negedge clock) begin
  if (reset) begin
    {cmt_wen, cmt_wdest, cmt_wdata, cmt_pc, cmt_inst, cmt_valid, trap, trap_code, cycleCnt, instrCnt,skip_en,intrNO_en,cause_code} <= 0;
  end
  else if (~trap && wb_excep_code != `TIME_TRAP_CODE) begin
    cmt_wen <= wb_wreg;
    cmt_wdest <= {3'd0, wb_wa};
    cmt_wdata <= wb_wd;
    cmt_pc    <= wb_pc;                                                                                                                            
    cmt_inst  <= wb_inst;
    cmt_valid <= inst_valid;
    skip_en   <= inst_skip;

    intrNO_en <= cpu_interr;
    //cause_code<= cpu_cause;

	  regs_diff <= regs;
    csrs_diff <= csrs;

    trap <= wb_inst[6:0] == 7'h6b;
    trap_code <= regs[10][7:0];
    cycleCnt <= cycleCnt + 1;
    instrCnt <= instrCnt + inst_valid;
  end
  else if (~trap && wb_excep_code == `TIME_TRAP_CODE) begin
    cmt_wen   <= 0;
    cmt_wdest <= 0;
    cmt_wdata <= 0;
    cmt_pc    <= wb_pc;
    cmt_inst  <= wb_inst;
    cmt_valid <= 0;
    skip_en   <= 0;

    intrNO_en <= 32'h00000007;
    //cause_code<= cpu_cause;


	  regs_diff <= regs;
    csrs_diff <= csrs;

    trap <= wb_inst[6:0] == 7'h6b;
    trap_code <= regs[10][7:0];
    cycleCnt <= cycleCnt + 1;
    instrCnt <= instrCnt + inst_valid;
  end
end

DifftestInstrCommit DifftestInstrCommit(
  .clock              (clock),
  .coreid             (0),
  .index              (0),
  .valid              (cmt_valid),
  .pc                 (cmt_pc),
  .instr              (cmt_inst),
  .special            (0),
  .skip               (skip_en),
  .isRVC              (0),
  .scFailed           (0),
  .wen                (cmt_wen),
  .wdest              (cmt_wdest),
  .wdata              (cmt_wdata)
);

DifftestArchIntRegState DifftestArchIntRegState (
  .clock              (clock),
  .coreid             (0),
  .gpr_0              (regs_diff[0]),
  .gpr_1              (regs_diff[1]),
  .gpr_2              (regs_diff[2]),
  .gpr_3              (regs_diff[3]),
  .gpr_4              (regs_diff[4]),
  .gpr_5              (regs_diff[5]),
  .gpr_6              (regs_diff[6]),
  .gpr_7              (regs_diff[7]),
  .gpr_8              (regs_diff[8]),
  .gpr_9              (regs_diff[9]),
  .gpr_10             (regs_diff[10]),
  .gpr_11             (regs_diff[11]),
  .gpr_12             (regs_diff[12]),
  .gpr_13             (regs_diff[13]),
  .gpr_14             (regs_diff[14]),
  .gpr_15             (regs_diff[15]),
  .gpr_16             (regs_diff[16]),
  .gpr_17             (regs_diff[17]),
  .gpr_18             (regs_diff[18]),
  .gpr_19             (regs_diff[19]),
  .gpr_20             (regs_diff[20]),
  .gpr_21             (regs_diff[21]),
  .gpr_22             (regs_diff[22]),
  .gpr_23             (regs_diff[23]),
  .gpr_24             (regs_diff[24]),
  .gpr_25             (regs_diff[25]),
  .gpr_26             (regs_diff[26]),
  .gpr_27             (regs_diff[27]),
  .gpr_28             (regs_diff[28]),
  .gpr_29             (regs_diff[29]),
  .gpr_30             (regs_diff[30]),
  .gpr_31             (regs_diff[31])
);

DifftestArchEvent DifftestArchEvent(
  .clock              (clock),
  .coreid             (0),
  .intrNO             (intrNO_en),
  .cause              (0),
  .exceptionPC        (cmt_pc),
  .exceptionInst      (cmt_inst)
);

DifftestTrapEvent DifftestTrapEvent(
  .clock              (clock),
  .coreid             (0),
  .valid              (trap),
  .code               (trap_code),
  .pc                 (cmt_pc),
  .cycleCnt           (cycleCnt),
  .instrCnt           (instrCnt)
);

DifftestCSRState DifftestCSRState(
  .clock              (clock),
  .coreid             (0),
  .priviledgeMode     (3),
  .mstatus            (csrs_diff[`MSTATUS]),
  .sstatus            (csrs_diff[`SSTATUS]),
  .mepc               (csrs_diff[`MEPC]),
  .sepc               (0),
  .mtval              (0),
  .stval              (0),
  .mtvec              (csrs_diff[`MTVEC]),
  .stvec              (0),
  .mcause             (csrs_diff[`MCAUSE]),
  .scause             (0),
  .satp               (0),
  .mip                (csrs_diff[`MIP]),
  .mie                (csrs_diff[`MIE]),
  .mscratch           (csrs_diff[`MSCRATCH]),
  .sscratch           (0),
  .mideleg            (0),
  .medeleg            (0)
);

DifftestArchFpRegState DifftestArchFpRegState(
  .clock              (clock),
  .coreid             (0),
  .fpr_0              (0),
  .fpr_1              (0),
  .fpr_2              (0),
  .fpr_3              (0),
  .fpr_4              (0),
  .fpr_5              (0),
  .fpr_6              (0),
  .fpr_7              (0),
  .fpr_8              (0),
  .fpr_9              (0),
  .fpr_10             (0),
  .fpr_11             (0),
  .fpr_12             (0),
  .fpr_13             (0),
  .fpr_14             (0),
  .fpr_15             (0),
  .fpr_16             (0),
  .fpr_17             (0),
  .fpr_18             (0),
  .fpr_19             (0),
  .fpr_20             (0),
  .fpr_21             (0),
  .fpr_22             (0),
  .fpr_23             (0),
  .fpr_24             (0),
  .fpr_25             (0),
  .fpr_26             (0),
  .fpr_27             (0),
  .fpr_28             (0),
  .fpr_29             (0),
  .fpr_30             (0),
  .fpr_31             (0)
);

endmodule