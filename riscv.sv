module riscv(   input   logic        clk, reset,
                output  logic [31:0] PCF,
                input   logic [31:0] InstrF,
                output  logic [1:0]  MemReadWriteM, 
                output  logic        PCstall,
                output  logic [31:0] ALUResultM, WriteDataM,
                output  logic [31:0] ReadDataM,
                input  logic [31:0] i_io_sw,
                input  logic [3:0]  i_io_btn,
                output logic [31:0] o_io_ledr,
                output logic [31:0] o_io_ledg,
                output logic [6:0]  o_io_hex0,
                output logic [6:0]  o_io_hex1,
                output logic [6:0]  o_io_hex2,
                output logic [6:0]  o_io_hex3,
                output logic [6:0]  o_io_hex4,
                output logic [6:0]  o_io_hex5,
                output logic [6:0]  o_io_hex6,
                output logic [6:0]  o_io_hex7,
                output logic [31:0] o_io_lcd,
                output logic [17:0]   SRAM_ADDR,
                input  logic [15:0]   SRAM_Q   ,
                output logic [15:0]   SRAM_D   ,
                output logic          SRAM_CE_N,
                output logic          SRAM_WE_N,
                output logic          SRAM_LB_N,
                output logic          SRAM_UB_N,
                output logic          SRAM_OE_N,
                output  logic [31:0] dec,a,b,
                output  logic mispredict, sourceF , PCSrcE
                ,output logic [31:0] PCTargetE,
                output logic [18:0] t,
                output logic ins_vld
                     );
    logic [6:0] opD;
    logic [2:0] funct3D;
    logic       funct7b5D;
    logic [2:0] ImmSrcD;
    logic       takenE;
    // logic       PCSrcE;
    logic [3:0] ALUControlE;
    logic [1:0] ALUSrcAE;
    logic       ALUSrcBE;
    logic       ResultSrcEb0;
    logic       RegWriteM;
    logic [1:0] ResultSrcW;
    logic       RegWriteW;

    logic [1:0] ForwardAE, ForwardBE;
    logic       StallF, StallD, StallE, StallM, FlushD, FlushE;
    logic [4:0] Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW;


    logic [1:0] br_typeE, data_typeM;

    logic JumpE, BranchE, BranchD, unsignE, unsignM, waitM;

    controller c(clk, reset,
                opD, funct3D, funct7b5D, ImmSrcD, BranchD,
                ins_vld,
                FlushE, StallE, takenE, PCSrcE, JumpE, BranchE, br_typeE, unsignE,
                ALUControlE, ALUSrcAE, ALUSrcBE,
                ResultSrcEb0, 
                StallM,
                unsignM, data_typeM,
                MemReadWriteM, RegWriteM, RegWriteW, ResultSrcW
                ,t
                );

    datapath dp(clk, reset,
                StallF, PCF, InstrF, PCstall, mispredict, sourceF,
                opD, funct3D, funct7b5D, StallD, FlushD, ImmSrcD, 
                BranchD, 
                JumpE, BranchE, 
                FlushE, ForwardAE, ForwardBE, PCSrcE, ALUControlE,
                ALUSrcAE, ALUSrcBE, unsignE, br_typeE, takenE, PCTargetE, 
                StallE,
                unsignM,
                MemReadWriteM, data_typeM, WriteDataM, ALUResultM, ReadDataM,
                StallM, 
                i_io_sw,
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
                o_io_lcd,
                waitM,
                SRAM_ADDR,
                SRAM_Q   ,
                SRAM_D   ,
                SRAM_CE_N,
                SRAM_WE_N,
                SRAM_LB_N,
                SRAM_UB_N,
                SRAM_OE_N,
                RegWriteW, ResultSrcW,
                Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW
                ,dec,a,b
                );
                    
    hazard hu(Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW,
                mispredict, ResultSrcEb0, PCstall, RegWriteM,
                waitM,
                RegWriteW,
                ForwardAE, ForwardBE, StallF, StallD, StallE, StallM, FlushD, FlushE);
endmodule

module datapath(input logic clk, reset,
                // Fetch stage signals
                input   logic        StallF,
                output  logic [31:0] PCF,
                input   logic [31:0] InstrF_i,
                output  logic        PCstall,
                output  logic        mispredictF,
                output  logic        sourceF,
                // Decode stage signals
                output  logic [6:0] opD,
                output  logic [2:0] funct3D,
                output  logic       funct7b5D,
                input   logic       StallD, FlushD,
                input   logic [2:0] ImmSrcD,
                input   logic       BranchD,
                // Execute stage signals
                input   logic       JumpE, BranchE,
                input   logic       FlushE, 
                input   logic [1:0] ForwardAE, ForwardBE,
                input   logic       PCSrcE,
                input   logic [3:0] ALUControlE,
                input   logic [1:0] ALUSrcAE, // needed for lui
                input   logic       ALUSrcBE,
                input   logic       unsignE,
                input   logic [1:0] br_typeE,
                output  logic       takenE,
                output  logic [31:0]PCTargetE,
                input   logic       StallE,
                // Memory stage signals
                input   logic       unsignM,
                input   logic  [1:0]MemReadWriteM, data_typeM,
                output  logic [31:0]WriteDataM, ALUResultM,
                output  logic [31:0]ReadDataM,
                input   logic       StallM,
                input  logic [31:0] i_io_sw,
                input  logic [3:0]  i_io_btn,
                output logic [31:0] o_io_ledr,
                output logic [31:0] o_io_ledg,
                output logic [6:0]  o_io_hex0,
                output logic [6:0]  o_io_hex1,
                output logic [6:0]  o_io_hex2,
                output logic [6:0]  o_io_hex3,
                output logic [6:0]  o_io_hex4,
                output logic [6:0]  o_io_hex5,
                output logic [6:0]  o_io_hex6,
                output logic [6:0]  o_io_hex7,
                output logic [31:0] o_io_lcd,
                output logic          waitM,
                output logic [17:0]   SRAM_ADDR,
                input  logic [15:0]   SRAM_Q   ,
                output logic [15:0]   SRAM_D   ,
                output logic          SRAM_CE_N,
                output logic          SRAM_WE_N,
                output logic          SRAM_LB_N,
                output logic          SRAM_UB_N,
                output logic          SRAM_OE_N,
                // Writeback stage signals
                input   logic       RegWriteW,
                input   logic [1:0] ResultSrcW,
                // Hazard Unit signals
                output  logic [4:0] Rs1D, Rs2D, Rs1E, Rs2E,
                output  logic [4:0] RdE, RdM, RdW,
                     
                output  logic [31:0] dec,a,b);

    // Fetch stage signals
        logic [31:0] PCNextF, PCPlus4F, InstrF_o, PCTargetF;
        // logic sourceF;
    // Decode stage signals
        logic [31:0] InstrD_in, InstrD;
        logic [31:0] PCD, PCPlus4D;
        logic [31:0] RD1D, RD2D;
        logic [31:0] ImmExtD;
        logic [4:0]  RdD;
    // Execute stage signals
        logic [31:0] RD1E, RD2E;
        logic [31:0] PCE, ImmExtE;
        logic [31:0] SrcAE0, SrcAE, SrcBE;
        logic [31:0] SrcAEforward;
        logic [31:0] ALUResultE;
        logic [31:0] WriteDataE;
        logic [31:0] PCPlus4E;
        // logic [31:0] PCTargetE;
    // Memory stage signals
        logic [31:0] PCPlus4M;
    // Writeback stage signals
        logic [31:0] ALUResultW;
        logic [31:0] ReadDataW;
        logic [31:0] PCPlus4W;
        logic [31:0] ResultW;
    // Fetch stage pipeline register and logic
        BranchPredictor btb (
            .i_clk(clk),
            .i_rst(reset),
            // Fetch stage signals
            .i_PCF(PCF),           // Current Program Counter
            .o_SourceF(sourceF),          // Branch prediction (1: Taken, 0: Not Taken)
            .o_PCTargetF(PCTargetF),    // Predicted target PC (valid if prediction = 1)
            // Decode stage signals
            .i_branchD(BranchD),
            .i_indexD(PCD[3:2]),          // PCD[3:2]
            .i_tagD(PCD[31:4]),           // PCD[31:4]
            // Execute stage signals
            .i_indexE(PCE[3:2]),          // PCE[3:2]
            .i_tagE(PCE[31:4]),           // PCE[31:4]
            .i_TargetE(PCTargetE),        // target PC if branch is taken
            .i_PCPlus4E(PCPlus4E),       // target PC if branch is mispredicted
            .i_branchE(BranchE),               // Indicates if the current instruction is a branch
            .i_pc_selE(PCSrcE),
            .i_JumpE(JumpE),
            .o_mispredict(mispredictF)
        );
        mux2    #(32) pcmux(PCPlus4F, PCTargetF, sourceF, PCNextF);
        flopenr #(32) pcreg(clk, reset, ~StallF, PCNextF, PCF);
        instr_realign realign(clk, reset, FlushD, InstrF_i, InstrF_o, PCstall);
        adder         pcadd(PCF, 32'h4, PCPlus4F);
    // Decode stage pipeline register and logic
        flopenrc #(96) regD(clk, reset, FlushD, ~StallD,
                            {InstrF_o, PCF, PCPlus4F},
                            {InstrD_in, PCD, PCPlus4D});
                    

                    assign a = SrcAE;
                    assign b = SrcBE;
                    assign dec = ALUResultE;

        decompress     decom(InstrD_in, InstrD);
        assign opD       = InstrD[6:0];
        assign funct3D   = InstrD[14:12];
        assign funct7b5D = InstrD[30];
        assign Rs1D      = InstrD[19:15];
        assign Rs2D      = InstrD[24:20];
        assign RdD       = InstrD[11:7];

        regfile rf(clk, reset, RegWriteW, Rs1D, Rs2D, RdW, ResultW, RD1D, RD2D);
        extend ext(InstrD[31:7], ImmSrcD, ImmExtD);

    // Execute stage pipeline register and logic
        flopenrc #(175) regE(clk, reset, FlushE, ~StallE,
                            {RD1D, RD2D, PCD, Rs1D, Rs2D, RdD, ImmExtD, PCPlus4D},
                            {RD1E, RD2E, PCE, Rs1E, Rs2E, RdE, ImmExtE, PCPlus4E});
        mux3 #(32) faemux(RD1E, ResultW, ALUResultM, ForwardAE, SrcAEforward);
        mux2 #(32) srcamux0(32'b0, dPCE, ALUSrcAE[1], SrcAE0); // for auipc
        mux2 #(32) srcamux(SrcAEforward, SrcAE0, ALUSrcAE[0], SrcAE); // for lui
        mux3 #(32) fbemux(RD2E, ResultW, ALUResultM, ForwardBE, WriteDataE);
        mux2 #(32) srcbmux(WriteDataE, ImmExtE, ALUSrcBE, SrcBE);
        alu        alu(SrcAE, SrcBE, unsignE, br_typeE, ALUControlE, ALUResultE, takenE);
        adder      branchadd(ImmExtE, PCE, PCTargetE);
    // Memory stage pipeline register
        flopenr #(101) regM(clk, reset, ~StallM,
                        {ALUResultE, WriteDataE, RdE, PCPlus4E},
                        {ALUResultM, WriteDataM, RdM, PCPlus4M});
        lsu Lsu (
            .i_clk(clk),
            .i_rst_n(reset),
            .i_lsu_addr(ALUResultM),
            .i_st_data(WriteDataM),
            .i_lsu_wren(MemReadWriteM),
            .i_io_sw(i_io_sw),
            .i_data_type(data_typeM),
            .i_unsigned(unsignM),
            .i_io_btn(i_io_btn),
            .o_ld_data(ReadDataM),
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
            .o_pc_stall(waitM), // to hazard unit
            .SRAM_ADDR(SRAM_ADDR),
            .SRAM_Q(SRAM_Q),
            .SRAM_D(SRAM_D), 
            .SRAM_CE_N(SRAM_CE_N),
            .SRAM_WE_N(SRAM_WE_N),
            .SRAM_LB_N(SRAM_LB_N),
            .SRAM_UB_N(SRAM_UB_N),
            .SRAM_OE_N(SRAM_OE_N)
        );
    // Writeback stage pipeline register and logic
        flopr #(101) regW(clk, reset,
                        {ALUResultM, ReadDataM, RdM, PCPlus4M},
                        {ALUResultW, ReadDataW, RdW, PCPlus4W});
        mux3 #(32) resultmux(ALUResultW, ReadDataW, PCPlus4W, ResultSrcW,
        ResultW);
endmodule


module adder(input [31:0] a, b,
        output [31:0] y);
        assign y = a + b;
endmodule


module flopr #(parameter WIDTH = 8)
    (input logic clk, reset,
    input logic [WIDTH-1:0] d,
    output logic [WIDTH-1:0] q);
    always_ff @(posedge clk, negedge reset)
    if (!reset) q <= 0;
    else q <= d;
endmodule

module flopenr #(parameter WIDTH = 8)
    (input logic clk, reset, en,
    input logic [WIDTH-1:0] d,
    output logic [WIDTH-1:0] q);
    always_ff @(posedge clk, negedge reset)
    if (!reset) q <= 0;
    else if (en) q <= d;
endmodule

module flopenrc #(parameter WIDTH = 8)
    (input logic clk, reset, clear, en,
    input logic [WIDTH-1:0] d,
    output logic [WIDTH-1:0] q);
    always_ff @(posedge clk, negedge reset)
    if (!reset) q <= 0;
    else if (en)
    if (clear) q <= 0;
    else q <= d;
endmodule

module floprc #(parameter WIDTH = 8)
    (input logic clk,
    input logic reset,
    input logic clear,
    input logic [WIDTH-1:0] d,
    output logic [WIDTH-1:0] q);
    always_ff @(posedge clk, negedge reset)
    if (!reset) q <= 0;
    else
    if (clear) q <= 0;
    else q <= d;
endmodule

module mux2 #(parameter WIDTH = 8)
    (input logic [WIDTH-1:0] d0, d1,
    input logic s,
    output logic [WIDTH-1:0] y);
    assign y = s ? d1 : d0;
endmodule

module mux3 #(parameter WIDTH = 8)
    (input logic [WIDTH-1:0] d0, d1, d2,
    input logic [1:0] s,
    output logic [WIDTH-1:0] y);
    assign y = s[1] ? d2 : (s[0] ? d1 : d0);
endmodule





 
