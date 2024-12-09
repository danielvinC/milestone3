
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

    assign o_mispredict = ((i_tagE == BTB[i_indexE].tag) && (i_pc_selE!= BTB[i_indexE].state[1])) | i_JumpE;
    
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