
`include "defines.v"

module icache(
    input   wire       cpu_clk_50M,
    input   wire       cpu_rst_n,

    output  wire                    if_burst_valid,
    output  wire[63: 4]             if_burst_addr,
    output  wire[ 1: 0]             if_burst_len,
    output  wire[ 1: 0]             if_burst_size,

    input   wire                    if_burst_ready,
    input   wire[`WORD_BUS]         if_burst_data,

    input   wire                    cpu_if_ena,
    input   wire[`INST_ADDR_BUS]    cpu_if_pc,
    output  reg                     axi_isused,

    output  wire                    read_isdone,
    output  wire[`INST_BUS]         icache_data_read,

    //flush 
    input   wire                    flush     
);

    wire[54: 0] cache_tag   = (cpu_if_ena == `TRUE_V)   ?   cpu_if_pc[63: 9]    :   55'b0;
    wire[ 4: 0] cache_index = (cpu_if_ena == `TRUE_V)   ?   cpu_if_pc[ 8: 4]    :   8'b0;
    wire[ 3: 0] cache_offset= (cpu_if_ena == `TRUE_V)   ?   cpu_if_pc[ 3: 0]    :   4'b0;

    wire cache_hit = cpu_if_ena == `TRUE_V & v_indexs[cache_index] [64] == `TRUE_V & v_indexs[cache_index] [63: 9] == cpu_if_pc [63: 9];
    wire cache_miss= cpu_if_ena == `TRUE_V & cache_hit == `FALSE_V;

    assign read_isdone  = (cache_hit)   ?   `TRUE_V :   `FALSE_V;

    assign icache_data_read =   (cache_hit == `TRUE_V && cache_offset == 4'b0000)   ?   icaches[cache_index] [31: 0]    :
                                (cache_hit == `TRUE_V && cache_offset == 4'b0100)   ?   icaches[cache_index] [63:32]    :
                                (cache_hit == `TRUE_V && cache_offset == 4'b1000)   ?   icaches[cache_index] [95:64]    :
                                (cache_hit == `TRUE_V && cache_offset == 4'b1100)   ?   icaches[cache_index] [127:96]   :  `ZERO_INST;

    reg [`INST_ADDR_BUS]    cache_axi_addr;
    reg [ 4: 0]             cache_axi_index;
    always @(posedge cpu_clk_50M) begin
        if(cpu_rst_n == `RST_ENABLE) begin
            cache_axi_addr  <=  `PC_INIT;
            cache_axi_index <=  5'b0;
        end
        else if(axi_isused == `FALSE_V) begin
            cache_axi_addr  <=  cpu_if_pc;
            cache_axi_index <=  cache_index;
        end
    end

    reg axi_isdelete;
    always @(posedge cpu_clk_50M) begin
        if(cpu_rst_n == `RST_ENABLE) begin
            axi_isdelete    <=  `FALSE_V;
        end
        else if(axi_isused == `TRUE_V && flush == `TRUE_V && axi_isdelete == `FALSE_V) begin
            axi_isdelete    <=  `TRUE_V;
        end
        else if(trans_count == `TRUE_V) begin
            axi_isdelete    <=  `FALSE_V;
        end
    end

    always @(posedge cpu_clk_50M) begin
        if(cpu_rst_n == `RST_ENABLE) begin
            axi_isused <= `FALSE_V;
        end
        else if(if_burst_valid == `TRUE_V && axi_isused == `FALSE_V) begin
            axi_isused <= `TRUE_V;
        end
        else if(axi_isused == `TRUE_V && trans_count == `TRUE_V) begin
            axi_isused <= `FALSE_V;
        end
    end

    /*****************************State Machine**********************************************/
    parameter    R_STATE_IDLE = 1'b0, R_STATE_READ = 1'b1;

    reg  cache_state;
    wire r_state_idle = cache_state == R_STATE_IDLE, r_state_read = cache_state == R_STATE_READ;

    // Read State Machine
    always @(posedge cpu_clk_50M) begin
        if(cpu_rst_n == `RST_ENABLE) begin
            cache_state <= R_STATE_IDLE;
        end
        else begin
            case (cache_state)
                R_STATE_IDLE:   if(cache_miss && axi_isused == `FALSE_V)    cache_state <= R_STATE_READ;
                R_STATE_READ:   if(trans_count==`TRUE_V)                    cache_state <= R_STATE_IDLE;
            endcase
        end
    end


    /*****************************State Machine**********************************************/

    assign if_burst_valid = cache_state == R_STATE_READ;
    assign if_burst_addr  = {cache_axi_addr[63: 4],4'b0};
    assign if_burst_size  = `SIZE_D;
    assign if_burst_len   = 2'b10;

    reg burst_count;
    reg trans_count;
    reg [64: 0]       v_indexs[ 0: `ICACHE_SIEZ - 1];
    reg [`CACHE_BUS]  icaches [ 0: `ICACHE_SIEZ - 1];

    always @(posedge cpu_clk_50M) begin
        if(cpu_rst_n == `RST_ENABLE) begin
            trans_count     <=  1'b0;
        end
        else if(if_burst_ready == `TRUE_V && trans_count == 1'b0) begin
            trans_count <=  1'b1;
        end
        else if(if_burst_ready == `TRUE_V && trans_count == 1'b1) begin
            trans_count <=  1'b0;
        end
    end

    always @(posedge cpu_clk_50M) begin
        if(cpu_rst_n == `RST_ENABLE) begin
            v_indexs[0]   <= 65'b0;
            v_indexs[1]   <= 65'b0;
            v_indexs[2]   <= 65'b0;
            v_indexs[3]   <= 65'b0;
            v_indexs[4]   <= 65'b0;
            v_indexs[5]   <= 65'b0;
            v_indexs[6]   <= 65'b0;
            v_indexs[7]   <= 65'b0;
            v_indexs[8]   <= 65'b0;
            v_indexs[9]   <= 65'b0;
            v_indexs[10]   <= 65'b0;
            v_indexs[11]   <= 65'b0;
            v_indexs[12]   <= 65'b0;
            v_indexs[13]   <= 65'b0;
            v_indexs[14]   <= 65'b0;
            v_indexs[15]   <= 65'b0;
            v_indexs[16]   <= 65'b0;
            v_indexs[17]   <= 65'b0;
            v_indexs[18]   <= 65'b0;
            v_indexs[19]   <= 65'b0;
            v_indexs[20]   <= 65'b0;
            v_indexs[21]   <= 65'b0;
            v_indexs[22]   <= 65'b0;
            v_indexs[23]   <= 65'b0;
            v_indexs[24]   <= 65'b0;
            v_indexs[25]   <= 65'b0;
            v_indexs[26]   <= 65'b0;
            v_indexs[27]   <= 65'b0;
            v_indexs[28]   <= 65'b0;
            v_indexs[29]   <= 65'b0;
            v_indexs[30]   <= 65'b0;
            v_indexs[31]   <= 65'b0;
            icaches[0]   <= 128'b0;
            icaches[1]   <= 128'b0;
            icaches[2]   <= 128'b0;
            icaches[3]   <= 128'b0;
            icaches[4]   <= 128'b0;
            icaches[5]   <= 128'b0;
            icaches[6]   <= 128'b0;
            icaches[7]   <= 128'b0;
            icaches[8]   <= 128'b0;
            icaches[9]   <= 128'b0;
            icaches[10]   <= 128'b0;
            icaches[11]   <= 128'b0;
            icaches[12]   <= 128'b0;
            icaches[13]   <= 128'b0;
            icaches[14]   <= 128'b0;
            icaches[15]   <= 128'b0;
            icaches[16]   <= 128'b0;
            icaches[17]   <= 128'b0;
            icaches[18]   <= 128'b0;
            icaches[19]   <= 128'b0;
            icaches[20]   <= 128'b0;
            icaches[21]   <= 128'b0;
            icaches[22]   <= 128'b0;
            icaches[23]   <= 128'b0;
            icaches[24]   <= 128'b0;
            icaches[25]   <= 128'b0;
            icaches[26]   <= 128'b0;
            icaches[27]   <= 128'b0;
            icaches[28]   <= 128'b0;
            icaches[29]   <= 128'b0;
            icaches[30]   <= 128'b0;
            icaches[31]   <= 128'b0;

            burst_count   <= 1'b0;
           
        end
        else if(if_burst_ready == `TRUE_V && burst_count == 1'b0 && axi_isdelete == `FALSE_V) begin
            burst_count                 <= 1'b1;
            icaches[cache_axi_index][63: 0] <= if_burst_data;
        end
        else if(if_burst_ready == `TRUE_V && burst_count == 1'b1 && axi_isdelete == `FALSE_V) begin
            burst_count                 <= 1'b0;
            icaches[cache_axi_index][127:64]<= if_burst_data;
            v_indexs[cache_axi_index][63: 0]<= cache_axi_addr;
            v_indexs[cache_axi_index][64]   <= `TRUE_V;
        end 
    end


endmodule
