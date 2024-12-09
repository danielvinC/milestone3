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