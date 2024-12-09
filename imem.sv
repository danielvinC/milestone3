module imem(input logic [31:0] a,
    output logic [31:0] rd);
    logic [31:0] RAM[63:0];
    initial
    $readmemh("C:/Users/Dell/dan/intelFPGA_lite/18.1/2023-march/da/example.txt",RAM);
    assign rd = RAM[a[31:2]]; // word aligned
endmodule