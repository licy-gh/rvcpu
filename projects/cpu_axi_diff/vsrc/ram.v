
`include "defines.v"

module ram(
    input cpu_clk_50M,
    
    //input[`INST_ADDR_BUS]   inst_addr,
    //input                   inst_ena,
    //output[`INST_BUS]       inst,

    // DATA PORT
    input ram_wr_en,
    input ram_rd_en,
    input [`WORD_BUS]ram_wmask,
    input [`WORD_BUS]ram_addr,
    input [`WORD_BUS]ram_wr_data,
    output reg [`WORD_BUS]ram_rd_data
);

    // INST PORT

    //wire[`WORD_BUS] inst_2 = ram_read_helper(inst_ena,{3'b000,(inst_addr-`PC_INIT)>>3});

    //assign inst = inst_addr[2] ? inst_2[63:32] : inst_2[31:0];

    // DATA PORT 
    assign ram_rd_data = ram_read_helper(ram_rd_en, {3'b000,(ram_addr-64'h0000_0000_8000_0000)>>3});

    always @(posedge cpu_clk_50M) begin
        ram_write_helper((ram_addr-64'h0000_0000_8000_0000)>>3, ram_wr_data, ram_wmask, ram_wr_en);
    end

endmodule
