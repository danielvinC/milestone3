module testbench();
    logic clk;
    logic reset;
    logic [31:0] WriteData, DataAdr;
    logic MemLoad, MemWrite, PCstall;
	logic [31:0] PCF, InstrF, ReadDataM;
    logic [31:0] i_io_sw;
    logic [3:0]  i_io_btn;
    logic [31:0] o_io_ledr;
    logic [31:0] o_io_ledg;
    logic [6:0]  o_io_hex0;
    logic [6:0]  o_io_hex1;
    logic [6:0]  o_io_hex2;
    logic [6:0]  o_io_hex3;
    logic [6:0]  o_io_hex4;
    logic [6:0]  o_io_hex5;
    logic [6:0]  o_io_hex6;
    logic [6:0]  o_io_hex7;
    logic [31:0] o_io_lcd
	 , dec, a,b
	 ; logic mispredict, PCSrcE, sourceF
     ; logic [31:0] PCTargetE
	 ; logic [17:0] t
     ; logic ins_vld
     ;
// instantiate device to be tested
    rv_top dut(clk, reset, WriteData, DataAdr, {MemLoad, MemWrite}, PCstall, PCF, InstrF, ReadDataM
	 ,dec,a,b
     ,mispredict, sourceF, PCSrcE
     , PCTargetE
     ,  i_io_sw,
        i_io_btn,
        o_io_ledr,
        o_io_ledg,
        o_io_hex0,
        o_io_hex1,
        o_io_hex2,
        o_io_hex3,
        o_io_hex4,
        o_io_hex5,
        o_io_hex6,
        o_io_hex7,
        o_io_lcd
     , t
     , ins_vld
	 );
// initialize test
    initial
        begin
            reset <= 0; # 22; reset <= 1;
        end
    // generate clock to sequence tests
    always
        begin
            clk <= 1; # 5; clk <= 0; # 5;
        end

    //   check results
    always @(negedge clk)
    begin
        if(MemWrite) begin
            if(DataAdr === 96 & WriteData === 3) begin
                $display("Simulation succeeded");
                #10
                $stop;
            end 
        end
    end
endmodule
