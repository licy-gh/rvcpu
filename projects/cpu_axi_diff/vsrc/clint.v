
`include "defines.v"

module clint(
    input   wire                cpu_clk_50M,
    input   wire                cpu_rst_n,

    input   wire                read_en,
    input   wire[`WORD_BUS]     read_addr,
    output  reg [`WORD_BUS]     data_read,

    input   wire                write_en,
    input   wire[`WORD_BUS]     write_addr,
    input   wire[`WORD_BUS]     data_write,
    output  reg                 clint_write_isdone,

    output  wire                interr,
    input   wire                interr_isdone
);

    reg[`REG_BUS]     mtime;
    reg[`REG_BUS]     mtimecmp;
    reg               interr_ena;

    assign  interr = (interr_ena == `TRUE_V && mtime >= mtimecmp)   ?    `INT_ON :  `INT_OFF;

    always @(posedge cpu_clk_50M) begin
        if(cpu_rst_n == `RST_ENABLE) begin
            interr_ena  <= `TRUE_V;
            clint_write_isdone  <=  `FALSE_V;
        end
        else if(interr_isdone == `TRUE_V) begin
            interr_ena  <= `FALSE_V;
        end
        else if(mtime < mtimecmp /*&& interr_ena == `FALSE_V*/) begin
            interr_ena  <= `TRUE_V;
        end
    end

    always @(posedge cpu_clk_50M) begin
        if(cpu_rst_n == `RST_ENABLE) begin
            mtime   <=  `MTIME_INIT;
        end
        else if(write_en == `TRUE_V && write_addr == `MTIME_ADDR) begin
            mtime   <=  data_write;
        end
        else begin
            mtime   <=  mtime + 1;
        end
    end

    always @(posedge cpu_clk_50M) begin
        if(cpu_rst_n == `RST_ENABLE) begin
            mtimecmp    <=  `MTIMECMP_INIT;
        end
        else if(write_en == `TRUE_V && write_addr == `MTIMECMP_ADDR) begin
            mtimecmp    <=  data_write;
            clint_write_isdone  <=  `TRUE_V;
        end
        else begin
            clint_write_isdone  <=  `FALSE_V;
        end
    end

    always @(posedge cpu_clk_50M) begin
        if(cpu_rst_n == `RST_ENABLE) begin
            data_read   <=   `ZERO_WORD;
        end
        else if(read_en == `TRUE_V && read_addr == `MTIME_ADDR) begin
            data_read   <=   mtime;
        end
        else if(read_en == `TRUE_V && read_addr == `MTIMECMP_ADDR) begin
            data_read   <=   mtimecmp;
        end
        else begin
            data_read   <=   `ZERO_WORD;
        end
    end

endmodule