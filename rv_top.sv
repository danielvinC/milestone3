module rv_top(
    input logic clk, reset,
    output logic [31:0] WriteDataM, DataAdrM,
    output logic [1:0] MemReadWriteM,
    output logic PCstall,
    output logic [31:0] PCF, InstrF, ReadDataM,
    output logic [31:0] dec,a,b,
    output logic mispredict, sourceF, PCSrcE,
    output logic [31:0] PCTargetE,
    input logic [31:0] i_io_sw,
    input logic [3:0] i_io_btn,
    output logic [31:0] o_io_ledr,
    output logic [31:0] o_io_ledg,
    output logic [6:0] o_io_hex0,
    output logic [6:0] o_io_hex1,
    output logic [6:0] o_io_hex2,
    output logic [6:0] o_io_hex3,
    output logic [6:0] o_io_hex4,
    output logic [6:0] o_io_hex5,
    output logic [6:0] o_io_hex6,
    output logic [6:0] o_io_hex7,
    output logic [31:0] o_io_lcd,
    output logic [17:0] t,
    output logic ins_vld
);

    // SRAM signals
    logic [17:0] SRAM_ADDR;
    logic [15:0] SRAM_Q;
    logic [15:0] SRAM_D;
    logic SRAM_CE_N, SRAM_WE_N, SRAM_LB_N, SRAM_UB_N, SRAM_OE_N;

    // Instantiate processor and memories
    riscv riscv(
        .clk(clk),
        .reset(reset),
        .PCF(PCF),
        .InstrF(InstrF),
        .MemReadWriteM(MemReadWriteM),
        .PCstall(PCstall),
        .ALUResultM(DataAdrM),
        .WriteDataM(WriteDataM),
        .ReadDataM(ReadDataM),
        .i_io_sw(i_io_sw),
        .i_io_btn(i_io_btn),
        .o_io_ledr(o_io_ledr),
        .o_io_ledg(o_io_ledg),
        .o_io_hex0(o_io_hex0),
        .o_io_hex1(o_io_hex1),
        .o_io_hex2(o_io_hex2),
        .o_io_hex3(o_io_hex3),
        .o_io_hex4(o_io_hex4),
        .o_io_hex5(o_io_hex5),
        .o_io_hex6(o_io_hex6),
        .o_io_hex7(o_io_hex7),
        .o_io_lcd(o_io_lcd),
        .SRAM_ADDR(SRAM_ADDR),
        .SRAM_Q(SRAM_Q),
        .SRAM_D(SRAM_D),
        .SRAM_CE_N(SRAM_CE_N),
        .SRAM_WE_N(SRAM_WE_N),
        .SRAM_LB_N(SRAM_LB_N),
        .SRAM_UB_N(SRAM_UB_N),
        .SRAM_OE_N(SRAM_OE_N),
        .dec(dec),
        .a(a),
        .b(b),
        .mispredict(mispredict),
        .sourceF(sourceF),
        .PCSrcE(PCSrcE),
        .PCTargetE(PCTargetE),
        .t(t),
        .ins_vld(ins_vld)
    );

    imem imem(
        PCF,
        InstrF
    );

    dmem dmem(
        .addr(SRAM_ADDR[11:0]), // Match address width
        .be({SRAM_UB_N, SRAM_LB_N}),
        .we(SRAM_WE_N),
        .clk(clk),
        .data_i(SRAM_D),
        .data_o(SRAM_Q)
    );

endmodule

module dmem
	#(
		parameter int
		BYTE_WIDTH = 8,
		ADDRESS_WIDTH = 12,
		BYTES = 2,
		DATA_WIDTH_R = BYTE_WIDTH * BYTES)
(
	input [ADDRESS_WIDTH-1:0] addr,
	input [BYTES-1:0] be,
	input we, clk,
	input [DATA_WIDTH_R-1:0] data_i,
	output [DATA_WIDTH_R-1:0] data_o
	);
	
	localparam RAM_DEPTH = 1 << ADDRESS_WIDTH;

	// model the RAM with two dimensional packed array
	logic [BYTES-1:0][BYTE_WIDTH-1:0] ram[0:RAM_DEPTH-1];

	reg [DATA_WIDTH_R-1:0] data_reg;
	logic [15:0] DIN, DOUT;

	// port A
	always@(posedge clk)
	begin
		if(!we) begin
		// edit this code if using other than four bytes per word
			if(!be[0]) ram[addr][0] <= DIN[7:0];
			if(!be[1]) ram[addr][1] <= DIN[15:8];
		end
	end
    assign DOUT = ram[addr];
	assign DIN  = data_i;
	assign data_o = DOUT;

endmodule