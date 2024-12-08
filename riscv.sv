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

module controller(input logic clk, reset,
                // Decode stage control signals
                input logic [6:0] opD,
                input logic [2:0] funct3D,
                input logic funct7b5D,
                output logic [2:0] ImmSrcD,
                output logic BranchD, ins_vld,
                // Execute stage control signals
                input logic FlushE, StallE,
                input logic takenE, 
                output logic PCSrcE, // for datapath and Hazard Unit
                output logic JumpE, BranchE, 
                output logic [1:0] br_typeE,
                output logic unsignE,
                output logic [3:0] ALUControlE,
                output logic [1:0] ALUSrcAE,
                output logic ALUSrcBE, // for lui
                output logic ResultSrcEb0, // for Hazard Unit
                // Memory stage control signals
                input  logic StallM, 
                output logic unsignM,
                output logic [1:0] data_typeM,
                output logic [1:0] MemReadWriteM,
                output logic RegWriteM, // for Hazard Unit
                // Writeback stage control signals
                output logic RegWriteW, // for datapath and Hazard Unit
                output logic [1:0] ResultSrcW,
                output logic [18:0] t);
    // pipelined control signals

    logic       RegWriteD, RegWriteE;
    logic [1:0] ResultSrcD, ResultSrcE, ResultSrcM;
    logic [1:0] MemReadWriteD, MemReadWriteE;
    logic       JumpD;
    // logic       BranchD;
    logic [1:0] ALUOpD;
    logic [3:0] ALUControlD;
    logic [1:0] ALUSrcAD;
    logic [1:0] br_typeD;
    logic [1:0] data_typeD, data_typeE;
    logic       unsignD;
    logic       ALUSrcBD; // for lui
    assign t = {RegWriteD, ResultSrcD, MemReadWriteD, JumpD, BranchD,
                            ALUControlD, ALUSrcAD, ALUSrcBD, br_typeD, data_typeD, unsignD}; 
            // t = 19'b0_0_0_0_0_0_0000_00_0_00_00_0;
            
    // Decode stage logic
    maindec md( opD, ResultSrcD, MemReadWriteD, BranchD,
                ALUSrcAD, ALUSrcBD, RegWriteD, JumpD, ImmSrcD, ALUOpD,
                ins_vld);
    aludec  ad(opD[5], funct3D, funct7b5D, ALUOpD, ALUControlD, br_typeD, data_typeD, unsignD);

    // Execute stage pipeline control register and logic
    flopenrc #(19) controlregE(clk, reset, FlushE, ~StallE,
                            {RegWriteD, ResultSrcD, MemReadWriteD, JumpD, BranchD,
                            ALUControlD, ALUSrcAD, ALUSrcBD, br_typeD, data_typeD, unsignD},
                            {RegWriteE, ResultSrcE, MemReadWriteE, JumpE, BranchE,
                            ALUControlE, ALUSrcAE, ALUSrcBE, br_typeE, data_typeE, unsignE});
    assign PCSrcE = (BranchE & takenE) | JumpE;
    assign ResultSrcEb0 = ResultSrcE[0];
    // Memory stage pipeline control register
    flopenr #(8) controlregM(clk, reset, ~StallM,
                        {RegWriteE, ResultSrcE, MemReadWriteE, data_typeE, unsignE},
                        {RegWriteM, ResultSrcM, MemReadWriteM, data_typeM, unsignM});
    // Writeback stage pipeline control register
    flopr #(3) controlregW(clk, reset,
                        {RegWriteM, ResultSrcM},
                        {RegWriteW, ResultSrcW});
endmodule

module maindec( input  logic [6:0] op,
                output logic [1:0] ResultSrc,
                output logic [1:0] MemReadWrite,
                output logic       Branch,
                output logic [1:0] ALUSrcA, // for lui
                output logic       ALUSrcB,
                output logic       RegWrite, Jump,
                output logic [2:0] ImmSrc,
                output logic [1:0] ALUOp,
                output logic ins_vld
                );

        logic [14:0] controls;

        assign {RegWrite, ImmSrc, ALUSrcA, ALUSrcB, MemReadWrite,
                ResultSrc, Branch, ALUOp, Jump} = controls;
    always_comb
        case(op)
        // RegWrite_ImmSrc_ALUSrcA_ALUSrcB_MemReadWrite_ResultSrc_Branch_ALUOp_Jump
            7'b0000011: begin
                controls = 15'b1_000_00_1_10_01_0_11_0; // lw
                ins_vld = 1'b1;
            end
            7'b0100011: begin
                controls = 15'b0_001_00_1_01_00_0_11_0; // sw
                ins_vld = 1'b1;
            end
            7'b0110011: begin 
                controls = 15'b1_xxx_00_0_00_00_0_10_0; // R-type
                ins_vld = 1'b1;
            end
            7'b1100011: begin 
                controls = 15'b0_010_00_0_00_00_1_01_0; // branch
                ins_vld = 1'b1;
            end
            7'b0010011: begin 
                controls = 15'b1_000_00_1_00_00_0_10_0; // I-type ALU
                ins_vld = 1'b1;
            end
            7'b1101111: begin 
                controls = 15'b1_011_00_0_00_10_0_00_1; // jal
                ins_vld = 1'b1;
            end
            7'b1100111: begin 
                controls = 15'b1_000_00_0_00_00_0_00_1; // jalr
                ins_vld = 1'b1;
            end
            7'b0110111: begin 
                controls = 15'b1_100_01_1_00_00_0_00_0; // lui
                ins_vld = 1'b1;
            end
            7'b0010111: begin 
                controls = 15'b1_100_11_1_00_00_0_00_0; // auipc
                ins_vld = 1'b1;
            end
            7'b0000000: begin 
                controls = 15'b0_000_00_0_00_00_0_00_0; // need valid values at reset
                ins_vld = 1'b0;
            end
            default: begin
                controls = 15'bx_xxx_xx_x_xx_xx_x_xx_x; // non-implemented instruction
                ins_vld = 1'b0;
            end

        endcase
endmodule

module aludec(  input logic        opb5,
                input logic  [2:0] funct3,
                input logic        funct7b5,
                input logic  [1:0] ALUOp,
                output logic [3:0] ALUControl,
                output logic [1:0] br_type, data_type,
                output logic       unsign
                );

    logic RtypeSub, RtypeSra;
    assign RtypeSub = funct7b5 & opb5; // TRUE for R-type subtract instruction shift right arithmatic instruction
    assign RtypeSra = RtypeSub;

    always_comb begin
        unsign = 1'b0;
        br_type = 2'b00;
        data_type = 2'b00;
        case(ALUOp)
            2'b00: ALUControl = 4'b0000; // addition
            2'b01: begin // branch
                ALUControl = 4'b0001; // subtraction
                case(funct3)
                    3'b000:  {unsign, br_type} = 3'b0_00; // beq
                    3'b001:  {unsign, br_type} = 3'b0_01; // bne
                    3'b100:  {unsign, br_type} = 3'b0_10; // blt
                    3'b101:  {unsign, br_type} = 3'b0_11; // bge
                    3'b110:  {unsign, br_type} = 3'b1_10; // bltu
                    3'b111:  {unsign, br_type} = 3'b1_11; // bgeu
                    default: {unsign, br_type} = 3'bx_xx;
                endcase
            end
            2'b10: begin
                case(funct3) // R-type or I-type ALU
                    3'b000: if (RtypeSub)
                                ALUControl = 4'b0001; // sub
                            else
                                ALUControl = 4'b0000; // add, addi
                    3'b010:     ALUControl = 4'b0101; // slt, slti
                    3'b100:     ALUControl = 4'b0100; // xor, xori
                    3'b110:     ALUControl = 4'b0011; // or, ori
                    3'b111:     ALUControl = 4'b0010; // and, andi
                    3'b011: begin
                                ALUControl = 4'b0111; // sltu, sltiu
                                unsign = 1'b0;
                            end
                    3'b001:     ALUControl = 4'b1000; // sll, slli
                    3'b101: if (RtypeSra)
                                ALUControl = 4'b1001; // sra, srai
                            else
                                ALUControl = 4'b1010; // srl, srli

                    default:    ALUControl = 4'bxxxx; // ???
                endcase
            end
            2'b11: begin
                ALUControl = 4'b0000;
                case(funct3) // load or store
                    3'b000: {unsign, data_type} = 3'b0_10; // lb, sb
                    3'b001:  {unsign, data_type} = 3'b0_01; // lh, lh
                    3'b010:  {unsign, data_type} = 3'b0_00; // lw, sw
                    3'b100:  {unsign, data_type} = 3'b1_10; // lbu
                    3'b101:  {unsign, data_type} = 3'b1_01; // lhu
                    default: {unsign, data_type} = 3'bx_xx;
                endcase
            end
        endcase 
    end
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
            .i_indexD(PCD[3:2]),          // PCD[3:2]
            .i_tagD(PCD[31:4]),           // PCD[31:4]
            .i_branchD(BranchD),
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
                    assign dec = SRAM_Q;

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

// Hazard Unit: forward, stall, and flush
module hazard(input logic [4:0] Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW,
            input logic mispredict, ResultSrcEb0,
            input logic PCstall,
            input logic RegWriteM, waitM,
            input logic RegWriteW,
            output logic [1:0] ForwardAE, ForwardBE,
            output logic StallF, StallD, StallE, StallM, FlushD, FlushE);
    logic lwStallD;
    // forwarding logic
    always_comb begin
        ForwardAE = 2'b00;
        ForwardBE = 2'b00;
        if (Rs1E != 5'b0)
            if ((Rs1E == RdM) & RegWriteM) ForwardAE = 2'b10;
            else if ((Rs1E == RdW) & RegWriteW) ForwardAE = 2'b01;

        if (Rs2E != 5'b0)
            if ((Rs2E == RdM) & RegWriteM) ForwardBE = 2'b10;
            else if ((Rs2E == RdW) & RegWriteW) ForwardBE = 2'b01;
    end
    // stalls and flushes
    assign lwStallD = ResultSrcEb0 & ((Rs1D == RdE) | (Rs2D == RdE)) ;
    assign StallF = lwStallD | PCstall | waitM;
    assign StallD = lwStallD | waitM;
    assign StallE = waitM;
    assign StallM = waitM;
    assign FlushD = mispredict;
    assign FlushE = lwStallD | mispredict;
endmodule

// module regfile(input logic clk,
//             input logic we3,
//             input logic [ 4:0] a1, a2, a3,
//             input logic [31:0] wd3,
//             output logic [31:0] rd1, rd2);
//             logic [31:0] rf[31:0];
// // three ported register file
// // read two ports combinationally (A1/RD1, A2/RD2)
// // write third port on rising edge of clock (A3/WD3/WE3)
// // write occurs on falling edge of clock
// // register 0 hardwired to 0
// always_ff @(negedge clk)
//     if (we3) rf[a3] <= wd3;
//         assign rd1 = (a1 != 0) ? rf[a1] : 0;
//         assign rd2 = (a2 != 0) ? rf[a2] : 0;
// endmodule

module regfile (
    input logic clk,
    input logic reset,
    input logic we3,
    input logic [4:0] a1, a2, a3,
    input logic [31:0] wd3,
    output logic [31:0] rd1, rd2
);
    logic [31:0] rf[31:0];
    integer i;

    // three ported register file
    // read two ports combinationally (A1/RD1, A2/RD2)
    // write third port on rising edge of clock (A3/WD3/WE3)
    // write occurs on falling edge of clock
    // register 0 hardwired to 0

    // Initialize registers during reset
    always_ff @(negedge clk or negedge reset) begin
        if (!reset) begin
            for (i = 0; i < 32; i = i + 1) begin
                rf[i] <= 32'd0;
            end
        end else
            // Write operation on rising edge of clock
            if (we3) rf[a3] <= wd3;
    end
        
        // Read operations
        assign rd1 = (a1 != 0) ? rf[a1] : 0;
        assign rd2 = (a2 != 0) ? rf[a2] : 0;
    
endmodule

module adder(input [31:0] a, b,
        output [31:0] y);
        assign y = a + b;
endmodule

module extend(input logic [31:7] instr,
            input logic [2:0] immsrc, // extended to 3 bits for lui
            output logic [31:0] immext);
    always_comb
        case(immsrc)
        // I-type
        3'b000: immext = {{20{instr[31]}}, instr[31:20]};
        // S-type (stores)
        3'b001: immext = {{20{instr[31]}}, instr[31:25], instr[11:7]};
        // B-type (branches)
        3'b010: immext = {{20{instr[31]}}, instr[7], instr[30:25],
        instr[11:8], 1'b0};
        // J-type (jal)
        3'b011: immext = {{12{instr[31]}}, instr[19:12], instr[20],
        instr[30:21], 1'b0};
        // U-type (lui, auipc)
        3'b100: immext = {instr[31:12], 12'b0};
        default: immext = 32'bx; // undefined
        endcase
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

module imem(input logic [31:0] a,
    output logic [31:0] rd);
    logic [31:0] RAM[63:0];
    initial
    $readmemh("C:/Users/Dell/dan/intelFPGA_lite/18.1/2023-march/da/example.txt",RAM);
    assign rd = RAM[a[31:2]]; // word aligned
endmodule

// module dmem(input logic clk, we,
//     input logic [31:0] a, wd,
//     output logic [31:0] rd);
//     logic [31:0] RAM[63:0];
//     assign rd = RAM[a[31:2]]; // word aligned
//     always_ff @(posedge clk)
//     if (we) RAM[a[31:2]] <= wd;
// endmodule

module alu(input logic [31:0] a, b,
            input logic unsign, 
            input logic [1:0] br_type,
            input logic [3:0] alucontrol,
            output logic [31:0] result,
            output logic taken
        );
        logic [31:0]    condinvb;
        logic [32:0]    sum;
        logic           v; // overflow
        logic           isAddSub; // true when is add or subtract operation

        logic zero, less;
        assign condinvb = alucontrol[0] ? ~b : b;
        assign sum = {1'b0, a} + {1'b0, condinvb} + alucontrol[0];
        assign isAddSub = ~alucontrol[2] & ~alucontrol[1] |
                        ~alucontrol[1] & alucontrol[0];

        logic [31:0] shift_reg;
        logic [4:0] shift;
        assign shift = b[4:0];

        always_comb begin
            shift_reg = a;
            result = 32'b0;
            case (alucontrol)
                4'b0000: result = sum[31:0]; // add, addi
                4'b0001: result = sum[31:0]; // subtract
                4'b0010: result = a & b; // and, andi
                4'b0011: result = a | b; // or, ori
                4'b0100: result = a ^ b; // xor, xori
                4'b0101: result = sum[31] ^ v; // slt, slti
                4'b0111: result = !sum[32]; // sltu, sltiu
                4'b1000: begin
                    if (shift[0]) shift_reg = {shift_reg[30:0], 1'b0};   // Shift by 1 bit
                    if (shift[1]) shift_reg = {shift_reg[29:0], 2'b0};   // Shift by 2 bits
                    if (shift[2]) shift_reg = {shift_reg[27:0], 4'b0};   // Shift by 4 bits
                    if (shift[3]) shift_reg = {shift_reg[23:0], 8'b0};   // Shift by 8 bits
                    if (shift[4]) shift_reg = {shift_reg[15:0], 16'b0};  // Shift by 16 bits
                    result = shift_reg; // sll, slli
                end
                4'b1001: begin
                    // Arithmetic right shift
                    if (shift[0]) shift_reg = {shift_reg[31], shift_reg[31:1]};        // Shift by 1 bit
                    if (shift[1]) shift_reg = {{2{shift_reg[31]}}, shift_reg[31:2]};   // Shift by 2 bits
                    if (shift[2]) shift_reg = {{4{shift_reg[31]}}, shift_reg[31:4]};   // Shift by 4 bits
                    if (shift[3]) shift_reg = {{8{shift_reg[31]}}, shift_reg[31:8]};   // Shift by 8 bits
                    if (shift[4]) shift_reg = {{16{shift_reg[31]}}, shift_reg[31:16]}; // Shift by 16 bits
                    result = shift_reg; // sra, srai
                end
                4'b1010: begin
                    if (shift[0]) shift_reg = {1'b0, shift_reg[31:1]};   // Shift by 1 bit
                    if (shift[1]) shift_reg = {2'b0, shift_reg[31:2]};   // Shift by 2 bits
                    if (shift[2]) shift_reg = {4'b0, shift_reg[31:4]};   // Shift by 4 bits
                    if (shift[3]) shift_reg = {8'b0, shift_reg[31:8]};   // Shift by 8 bits
                    if (shift[4]) shift_reg = {16'b0, shift_reg[31:16]}; // Shift by 16 bits
                    result = shift_reg; // srl, srli
                end

                default: result = 32'bx;
            endcase
        end

        assign zero = (result == 32'b0);
        assign less = unsign ? sum[32] : sum[31] ^ v;
        assign v = ~(alucontrol[0] ^ a[31] ^ b[31]) & (a[31] ^ sum[31]) & isAddSub;

        always_comb begin
            case(br_type)
                2'b00: taken = zero; // beq
                2'b01: taken = ~zero; // bne
                2'b10: taken = less; // blt
                2'b11: taken = zero | ~less; // bge
            endcase
        end

endmodule

module BranchPredictor (
    input logic i_clk,
    input logic i_rst,
    // Fetch stage input signals
    input logic [31:0] i_PCF,           // Current Program Counter
    output logic o_SourceF,             // Branch prediction (1: Taken, 0: Not Taken)
    output logic [31:0] o_PCTargetF,    // Predicted target PC (valid if prediction = 1)
    // Decode stage input signals
    input logic i_branchD,
    input logic [1:0] i_indexD,          // PCD[3:2]
    input logic [27:0] i_tagD,           // PCD[31:4]
    // Execute stage input signals
    input logic [1:0] i_indexE,          // PCE[3:2]
    input logic [27:0] i_tagE,           // PCE[31:4]
    input logic [31:0] i_TargetE,        // target PC if branch is taken
    input logic [31:0] i_PCPlus4E,       // target PC if branch is mispredicted
    input logic i_branchE,               // Indicates if the current instruction is a branch
    input logic i_pc_selE,                // Actual branch outcome from execute stage
    input logic i_JumpE,                 // Jump Instruction: always mispredict
    output logic o_mispredict

    );

    // Parameters
    localparam INDEX_BITS = 2;          // Number of bits for indexing (2 bits for 4 entries)
    localparam TAG_BITS = 28;           // Remaining bits for the tag (32-bit PC minus 2 index bits)

    // Define the 2-bit FSM states
    typedef enum logic [1:0] {
        STRONG_NOT_TAKEN = 2'b00,
        WEAK_NOT_TAKEN   = 2'b01,
        WEAK_TAKEN       = 2'b10,
        STRONG_TAKEN     = 2'b11
    } state_t;

    // BTB Entry Structure
    typedef struct packed {
        logic [TAG_BITS-1:0] tag;       // Higher bits of PC for correctness check
        logic [31:0] target;            // Predicted target address
        state_t state;                  // FSM state for branch prediction
    } btb_entry_t;

    // Define the BTB as an array of 4 entries
    localparam BTB_SIZE = 1 << INDEX_BITS; // 2^2 = 4 entries
    btb_entry_t BTB [0:BTB_SIZE-1];

    // Extract index and tag from PC
    logic [INDEX_BITS-1:0] index;
    logic [TAG_BITS-1:0] tag;

    assign index = i_PCF[3:2];                 // Use bits [3:2] of the PC for indexing
    assign tag = i_PCF[31:4];                  // Use the remaining bits as the tag

    integer i;

    assign o_mispredict = (i_tagE == BTB[i_indexE][tag] && i_pc_selE != BTB[i_indexE].state[1]) | i_JumpE ? 1'b1 : 1'b0;

    // Fetch Stage Logic
    always_comb begin
        o_SourceF = 1'b0;           // Default prediction: Not Taken
        o_PCTargetF = 32'h0;        // Default target: 0
        if (o_mispredict) begin
            o_SourceF = 1'b1;   
            if (i_pc_selE)      
                o_PCTargetF = i_TargetE; 
            else    
                o_PCTargetF = i_PCPlus4E;
        end else if (BTB[index].tag == tag) begin
            // Tag matches, predict based on FSM state
            case (BTB[index].state)
                STRONG_NOT_TAKEN, WEAK_NOT_TAKEN: 
                    o_SourceF = 1'b0;
                WEAK_TAKEN, STRONG_TAKEN: begin
                    o_SourceF = 1'b1;
                    o_PCTargetF = BTB[index].target;
                end
            endcase
        end
    end

    
    always_ff @(posedge i_clk or negedge i_rst) begin
        if (!i_rst) begin
            // Reset logic: Initialize BTB entries
            for (i = 0; i < BTB_SIZE; i = i + 1) begin
                BTB[i].tag <= {TAG_BITS{1'b0}};
                BTB[i].target <= 32'h0;
                BTB[i].state <= STRONG_NOT_TAKEN;
            end
        end 
        // Decode Stage Logic
        else if (i_branchD) begin
            if (BTB[i_indexD].tag != i_tagD) begin
                BTB[i_indexD].tag <= i_tagD; // Update tag
                BTB[i_indexD].state <= STRONG_NOT_TAKEN;
            end
        end
        // Execute Stage Logic
        else if (i_branchE) begin
            // Update BTB on branch instruction
            if (i_pc_selE) begin
                // Branch was taken: Update target and FSM state
                BTB[i_indexE].target <= i_TargetE;
                if (BTB[i_indexE].state != STRONG_TAKEN)
                    BTB[i_indexE].state <= state_t'(BTB[i_indexE].state + 1);
            end else begin
                // Branch not taken: Decrement FSM state
                if (BTB[i_indexE].state != STRONG_NOT_TAKEN)
                    BTB[i_indexE].state <= state_t'(BTB[i_indexE].state - 1);
            end
        end
    end
endmodule

 
