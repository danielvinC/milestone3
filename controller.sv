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