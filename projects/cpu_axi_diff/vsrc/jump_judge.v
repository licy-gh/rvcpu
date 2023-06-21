
`include "defines.v"

module jump_judge (
    input   cpu_clk_50M,
    input   cpu_rst_n,

    input   wire    flush_i,
    output  wire    flush_if,
    output  reg     flush_reg

);
    assign  flush_if = flush_i;

    always @(cpu_clk_50M) begin
        flush_reg   <=  flush_i;
    end 
endmodule