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