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