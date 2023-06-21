
`include "defines.v"

module dcache(
    input   wire       cpu_clk_50M,
    input   wire       cpu_rst_n,

    output  wire                    mem_burst_valid,
    output  wire[`WORD_BUS]         mem_burst_addr,
    output  wire[ 1: 0]             mem_burst_len,
    output  wire[ 1: 0]             mem_burst_size,

    input   wire                    mem_burst_ready,
    input   wire[`WORD_BUS]         mem_burst_data,

    input   wire                    cpu_dataread_ena,
    input   wire[`WORD_BUS]         cpu_dataread_addr,
    input   wire[ 1: 0]             cpu_dataread_size,
    output  reg                     axi_isused,

    output  wire                    mem_read_isdone,
    output  wire[`WORD_BUS]         dcache_data_read,

    input   wire                    cpu_write_ena,
    input   wire[`WORD_BUS]         cpu_write_addr,
    input   wire[`WORD_BUS]         cpu_write_data,
    input   wire[ 7: 0]             cpu_write_mask,

    output  reg                     mem_w_valid,
    output  reg[`WORD_BUS]          mem_w_addr,
    output  reg[`WORD_BUS]          mem_w_data,
    output  reg[ 7: 0]              mem_w_mask,
    input   wire                    axi_w_isdone,

    output  reg                     axi_w_isbusy
    
);

    wire[54: 0] cache_tag   = (cpu_dataread_ena == `TRUE_V)   ?   cpu_dataread_addr[63: 9]    :
                              (cpu_write_ena    == `TRUE_V)   ?   cpu_write_addr[63: 9]       :     55'b0;
    wire[ 4: 0] cache_index = (cpu_dataread_ena == `TRUE_V)   ?   cpu_dataread_addr[ 8: 4]    :
                              (cpu_write_ena    == `TRUE_V)   ?   cpu_write_addr[ 8: 4]       :   8'b0;
    wire[ 3: 0] cache_offset= (cpu_dataread_ena == `TRUE_V)   ?   cpu_dataread_addr[ 3: 0]    :   4'b0;

    wire cache_hit = (cpu_dataread_ena == `TRUE_V || cpu_write_ena == `TRUE_V) & v_indexs[cache_index] [64] == `TRUE_V & v_indexs[cache_index] [63: 9] == cache_tag;
    wire cache_miss= (cpu_dataread_ena == `TRUE_V || cpu_write_ena == `TRUE_V) & cache_hit == `FALSE_V;

    assign mem_read_isdone  =   (mem_burst_ready == `TRUE_V && mem_burst_data != `ZERO_WORD && (cpu_dataread_addr == `MTIME_ADDR || cpu_dataread_addr == `MTIMECMP_ADDR))   ?   `TRUE_V :    
                                (cache_hit)   ?   `TRUE_V :   `FALSE_V;

    wire[`WORD_BUS] double_read_select =   (cache_hit == `TRUE_V && cache_offset[3] == 1'b0)   ?   dcaches[cache_index] [63: 0]    :
                                (cache_hit == `TRUE_V && cache_offset[3] == 1'b1)   ?   dcaches[cache_index] [127:64]   :  `ZERO_WORD;

    assign  dcache_data_read=   (mem_burst_ready == `TRUE_V && (cpu_dataread_addr == `MTIME_ADDR || cpu_dataread_addr == `MTIMECMP_ADDR))   ?   mem_burst_data  :
                                (cpu_dataread_size == `SIZE_D)                      ?   double_read_select              :
                                (cpu_dataread_size == `SIZE_W && cache_offset[2:0] == 3'b000)   ?   {32'b0,double_read_select[31: 0]}   :   
                                (cpu_dataread_size == `SIZE_W && cache_offset[2:0] == 3'b100)   ?   {32'b0,double_read_select[63:32]}   :
                                (cpu_dataread_size == `SIZE_H && cache_offset[2:0] == 3'b000)   ?   {48'b0,double_read_select[15: 0]}   :
                                (cpu_dataread_size == `SIZE_H && cache_offset[2:0] == 3'b010)   ?   {48'b0,double_read_select[31:16]}   :
                                (cpu_dataread_size == `SIZE_H && cache_offset[2:0] == 3'b100)   ?   {48'b0,double_read_select[47:32]}   :
                                (cpu_dataread_size == `SIZE_H && cache_offset[2:0] == 3'b110)   ?   {48'b0,double_read_select[63:48]}   :
                                (cpu_dataread_size == `SIZE_B && cache_offset[2:0] == 3'b000)   ?   {56'b0,double_read_select[ 7: 0]}   :
                                (cpu_dataread_size == `SIZE_B && cache_offset[2:0] == 3'b001)   ?   {56'b0,double_read_select[15: 8]}   :
                                (cpu_dataread_size == `SIZE_B && cache_offset[2:0] == 3'b010)   ?   {56'b0,double_read_select[23:16]}   :
                                (cpu_dataread_size == `SIZE_B && cache_offset[2:0] == 3'b011)   ?   {56'b0,double_read_select[31:24]}   :
                                (cpu_dataread_size == `SIZE_B && cache_offset[2:0] == 3'b100)   ?   {56'b0,double_read_select[39:32]}   :
                                (cpu_dataread_size == `SIZE_B && cache_offset[2:0] == 3'b101)   ?   {56'b0,double_read_select[47:40]}   :
                                (cpu_dataread_size == `SIZE_B && cache_offset[2:0] == 3'b110)   ?   {56'b0,double_read_select[55:48]}   :
                                (cpu_dataread_size == `SIZE_B && cache_offset[2:0] == 3'b111)   ?   {56'b0,double_read_select[63:56]}   :   `ZERO_WORD;

    reg [`INST_ADDR_BUS]    cache_axi_addr;
    reg [ 4: 0]             cache_axi_index;
    always @(posedge cpu_clk_50M) begin
        if(cpu_rst_n == `RST_ENABLE) begin
            cache_axi_addr  <=  `PC_INIT;
            cache_axi_index <=  5'b0;
        end
        else if(axi_isused == `FALSE_V) begin
            cache_axi_addr  <=  cpu_dataread_addr;
            cache_axi_index <=  cache_index;
        end
    end

    always @(posedge cpu_clk_50M) begin
        if(cpu_rst_n == `RST_ENABLE) begin
            axi_isused <= `FALSE_V;
        end
        else if(mem_burst_valid == `TRUE_V && axi_isused == `FALSE_V && cpu_dataread_addr != `MTIME_ADDR && cpu_dataread_addr != `MTIMECMP_ADDR) begin
            axi_isused <= `TRUE_V;
        end
        else if(axi_isused == `TRUE_V && burst_count == `TRUE_V) begin
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
                R_STATE_IDLE:   if(cache_miss && axi_isused == `FALSE_V && cpu_dataread_ena == `TRUE_V && cpu_dataread_addr != `MTIME_ADDR && cpu_dataread_addr != `MTIMECMP_ADDR)     cache_state <= R_STATE_READ;
                R_STATE_READ:   if(burst_count==`TRUE_V)                                                    cache_state <= R_STATE_IDLE;
            endcase
        end
    end


    /*****************************State Machine**********************************************/

    assign mem_burst_valid = (cpu_dataread_ena == `TRUE_V && (cpu_dataread_addr == `MTIME_ADDR || cpu_dataread_addr == `MTIMECMP_ADDR))  ?   `TRUE_V :   cache_state == R_STATE_READ;
    assign mem_burst_addr  = (cpu_dataread_ena == `TRUE_V && (cpu_dataread_addr == `MTIME_ADDR || cpu_dataread_addr == `MTIMECMP_ADDR))  ?   cpu_dataread_addr  :   {cache_axi_addr[63: 4],4'b0};
    assign mem_burst_size  = `SIZE_D;
    assign mem_burst_len   = 2'b10;

    reg burst_count;
    reg [64: 0]       v_indexs[ 0: `ICACHE_SIEZ - 1];
    reg [`CACHE_BUS]  dcaches [ 0: `ICACHE_SIEZ - 1];

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
            dcaches[0]   <= 128'b0;
            dcaches[1]   <= 128'b0;
            dcaches[2]   <= 128'b0;
            dcaches[3]   <= 128'b0;
            dcaches[4]   <= 128'b0;
            dcaches[5]   <= 128'b0;
            dcaches[6]   <= 128'b0;
            dcaches[7]   <= 128'b0;
            dcaches[8]   <= 128'b0;
            dcaches[9]   <= 128'b0;
            dcaches[10]   <= 128'b0;
            dcaches[11]   <= 128'b0;
            dcaches[12]   <= 128'b0;
            dcaches[13]   <= 128'b0;
            dcaches[14]   <= 128'b0;
            dcaches[15]   <= 128'b0;
            dcaches[16]   <= 128'b0;
            dcaches[17]   <= 128'b0;
            dcaches[18]   <= 128'b0;
            dcaches[19]   <= 128'b0;
            dcaches[20]   <= 128'b0;
            dcaches[21]   <= 128'b0;
            dcaches[22]   <= 128'b0;
            dcaches[23]   <= 128'b0;
            dcaches[24]   <= 128'b0;
            dcaches[25]   <= 128'b0;
            dcaches[26]   <= 128'b0;
            dcaches[27]   <= 128'b0;
            dcaches[28]   <= 128'b0;
            dcaches[29]   <= 128'b0;
            dcaches[30]   <= 128'b0;
            dcaches[31]   <= 128'b0;

            burst_count   <= 1'b0;
           
        end
        else if(mem_burst_ready == `TRUE_V && burst_count == 1'b0 && cache_axi_addr != `MTIME_ADDR && cache_axi_addr != `MTIMECMP_ADDR) begin
            burst_count                 <= 1'b1;
            dcaches[cache_axi_index][63: 0] <= mem_burst_data;
        end
        else if(mem_burst_ready == `TRUE_V && burst_count == 1'b1 && cache_axi_addr != `MTIME_ADDR && cache_axi_addr != `MTIMECMP_ADDR) begin
            burst_count                 <= 1'b0;
            dcaches[cache_axi_index][127:64]<= mem_burst_data;
            v_indexs[cache_axi_index][63: 0]<= cache_axi_addr;
            v_indexs[cache_axi_index][64]   <= `TRUE_V;
        end 
    end

    always @(posedge cpu_clk_50M) begin
        if(cpu_rst_n == `RST_ENABLE) begin
            axi_w_isbusy    <=  `FALSE_V;
            mem_w_valid     <=  `FALSE_V;
            mem_w_addr      <=  `ZERO_WORD;
            mem_w_data      <=  `ZERO_WORD;
            mem_w_mask      <=  8'b0;
        end
        else if(cpu_write_ena == `TRUE_V && cache_hit == `TRUE_V) begin
            axi_w_isbusy    <=  `TRUE_V;
            mem_w_valid     <=  `TRUE_V;
            mem_w_addr      <=  cpu_write_addr;
            mem_w_data      <=  cpu_write_data;
            mem_w_mask      <=  cpu_write_mask;
            v_indexs[cache_index][64]   <=  `FALSE_V;    
        end
        else if(cpu_write_ena == `TRUE_V) begin
            axi_w_isbusy    <=  `TRUE_V;
            mem_w_valid     <=  `TRUE_V;
            mem_w_addr      <=  cpu_write_addr;
            mem_w_data      <=  cpu_write_data;
            mem_w_mask      <=  cpu_write_mask;
        end
        else if(axi_w_isdone == `TRUE_V) begin
            axi_w_isbusy    <=  `FALSE_V;
            mem_w_valid     <=  `FALSE_V;
            mem_w_addr      <=  `ZERO_WORD;
            mem_w_data      <=  `ZERO_WORD;
            mem_w_mask      <=  8'b0;
        end
    end


endmodule
