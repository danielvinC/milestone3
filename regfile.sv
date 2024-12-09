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