// This is an asynchronous control unit, generating and passing control signals to other stages

// Although separatedly written, this control unit
// is not one of the five pipeline stages.

module ControlUnit (
    opcode, func  // memWrite, memRead, memToReg, aluSecondSrc, regDst, regWrite
);
    input [5:0] opcode, func;

    // Declare control signals
    reg memWrite;   // If asserted, write to data memory
    reg memRead;    // If asserted, read from data memory
    reg memToReg;   // If asserted, The value to reg write comes from memory; else from ALU
    reg aluSecondSrc;     // if 1, input from immediate field
    reg regDst;     // If asserted, the register "write data" is rd(15:11); else rt(20:16)
    reg regWrite;   // If asserted, the register will write data on the falling edge signal
    reg branch;     // Is branch instruction (not including jump)
    reg jump;       // Is jump instruction



    always @(opcode or func) begin
    case (opcode)
        6'b000000: // R-type instructions
        begin
            case (func)
                // add
                6'b100000: 
                begin 
                    memWrite <= 0;
                    memRead <= 0;
                    memToReg <= 0;
                    aluSecondSrc <= 0;
                    regDst <= 1;
                    regWrite <= 1;
                    branch <= 0;
                    jump <= 0;
                end
                6'b100001: begin // addu
                    memWrite <= 0;
                    memRead <= 0;
                    memToReg <= 0;
                    aluSecondSrc <= 0;
                    regDst <= 1;
                    regWrite <= 1;
                    branch <= 0;
                    jump <= 0;
                end
                // sub
                6'b100010: begin
                    memWrite <= 0;
                    memRead <= 0;
                    memToReg <= 0;
                    aluSecondSrc <= 0;
                    regDst <= 1;
                    regWrite <= 1;
                    branch <= 0;
                    jump <= 0;
                end
                // subu 
                6'b100011: begin
                    memWrite <= 0;
                    memRead <= 0;
                    memToReg <= 0;
                    aluSecondSrc <= 0;
                    regDst <= 1;
                    regWrite <= 1;
                    branch <= 0;
                    jump <= 0;
                end
                // and
                6'b100100: begin
                    memWrite <= 0;
                    memRead <= 0;
                    memToReg <= 0;
                    aluSecondSrc <= 0;
                    regDst <= 1;
                    regWrite <= 1;
                    branch <= 0;
                    jump <= 0;
                end
                // nor
                6'b100111: begin
                    memWrite <= 0;
                    memRead <= 0;
                    memToReg <= 0;
                    aluSecondSrc <= 0;
                    regDst <= 1;
                    regWrite <= 1;
                    branch <= 0;
                    jump <= 0;
                end
                // or
                6'b100101: begin
                    memWrite <= 0;
                    memRead <= 0;
                    memToReg <= 0;
                    aluSecondSrc <= 0;
                    regDst <= 1;
                    regWrite <= 1;
                    branch <= 0;
                    jump <= 0;
                end
                // xor
                6'b100110: begin
                    memWrite <= 0;
                    memRead <= 0;
                    memToReg <= 0;
                    aluSecondSrc <= 0;
                    regDst <= 1;
                    regWrite <= 1;
                    branch <= 0;
                    jump <= 0;
                end
                // slt
                6'b101010: begin
                    memWrite <= 0;
                    memRead <= 0;
                    memToReg <= 0;
                    aluSecondSrc <= 0;
                    regDst <= 1;
                    regWrite <= 1;
                    branch <= 0;
                    jump <= 0;
                end
                // srl
                6'b000010: begin
                    memWrite <= 0;
                    memRead <= 0;
                    memToReg <= 0;
                    aluSecondSrc <= 0;
                    regDst <= 1;
                    regWrite <= 1;
                    branch <= 0;
                    jump <= 0;
                end
                // srlv
                6'b000110: begin
                    memWrite <= 0;
                    memRead <= 0;
                    memToReg <= 0;
                    aluSecondSrc <= 0;
                    regDst <= 1;
                    regWrite <= 1;
                    branch <= 0;
                    jump <= 0;
                end
                // sra
                6'b000011: begin
                    memWrite <= 0;
                    memRead <= 0;
                    memToReg <= 0;
                    aluSecondSrc <= 0;
                    regDst <= 1;
                    regWrite <= 1;
                    branch <= 0;
                    jump <= 0;
                end
                // srav
                6'b000111: begin
                    memWrite <= 0;
                    memRead <= 0;
                    memToReg <= 0;
                    aluSecondSrc <= 0;
                    regDst <= 1;
                    regWrite <= 1;
                    branch <= 0;
                    jump <= 0;
                end
                // sll
                6'b000000: begin
                    memWrite <= 0;
                    memRead <= 0;
                    memToReg <= 0;
                    aluSecondSrc <= 0;
                    regDst <= 1;
                    regWrite <= 1;
                    branch <= 0;
                    jump <= 0;
                end
                // sllv
                6'b000100: begin
                    memWrite <= 0;
                    memRead <= 0;
                    memToReg <= 0;
                    aluSecondSrc <= 0;
                    regDst <= 1;
                    regWrite <= 1;
                    branch <= 0;
                    jump <= 0;
                end
                // jr
                6'b001000: begin
                    memWrite <= 0;
                    memRead <= 0;
                    memToReg <= 0;
                    aluSecondSrc <= 0;
                    regDst <= 0;
                    regWrite <= 0;
                    branch <= 0;
                    jump <= 1;
                end

                // default: // should not occur
                // $display("Error: Unknown R-type instruction format.");
            endcase
        end

        6'b000010:      // j
        begin
            memWrite <= 0;
            memRead <= 0;
            memToReg <= 0;
            aluSecondSrc <= 0;
            regDst <= 0;
            regWrite <= 0;
            branch <= 0;
            jump <= 1;
        end
        6'b000011:      // jal
        // For jal, one additional operation is performed
        // That is to write pc to register number 31
        begin
            memWrite <= 0;
            memRead <= 0;
            memToReg <= 0;
            aluSecondSrc <= 0;
            regDst <= 0;
            regWrite <= 1;
            branch <= 0;
            jump <= 1;
        end

            
        default: // I-type instructions
        begin
            case (opcode)
                // addi
                6'b001000: begin
                    memWrite <= 0;
                    memRead <= 0;
                    memToReg <= 0;
                    aluSecondSrc <= 1;
                    regDst <= 0;
                    regWrite <= 1;
                    branch <= 0;
                    jump <= 0;
                end
                // addiu
                6'b001001: begin
                    memWrite <= 0;
                    memRead <= 0;
                    memToReg <= 0;
                    aluSecondSrc <= 1;
                    regDst <= 0;
                    regWrite <= 1;
                    branch <= 0;
                    jump <= 0;
                end
                // andi
                6'b001100: begin
                    memWrite <= 0;
                    memRead <= 0;
                    memToReg <= 0;
                    aluSecondSrc <= 1;
                    regDst <= 0;
                    regWrite <= 1;
                    branch <= 0;
                    jump <= 0;
                end
                // ori
                6'b001101: begin
                    memWrite <= 0;
                    memRead <= 0;
                    memToReg <= 0;
                    aluSecondSrc <= 1;
                    regDst <= 0;
                    regWrite <= 1;
                    branch <= 0;
                    jump <= 0;
                end
                // xori
                6'b001110: begin
                    memWrite <= 0;
                    memRead <= 0;
                    memToReg <= 0;
                    aluSecondSrc <= 1;
                    regDst <= 0;
                    regWrite <= 1;
                    branch <= 0;
                    jump <= 0;
                end
                // beq
                6'b000100: begin
                    memWrite <= 0;
                    memRead <= 0;
                    memToReg <= 0;
                    aluSecondSrc <= 0;
                    regDst <= 0;
                    regWrite <= 0;
                    branch <= 1;
                    jump <= 0;
                end
                // bne
                6'b000101: begin
                    memWrite <= 0;
                    memRead <= 0;
                    memToReg <= 0;
                    aluSecondSrc <= 0;
                    regDst <= 0;
                    regWrite <= 0;
                    branch <= 1;
                    jump <= 0;
                end
                // lw
                6'b100011: begin
                    memWrite <= 0;
                    memRead <= 1;
                    memToReg <= 1;
                    aluSecondSrc <= 1;
                    regDst <= 0;        // rt
                    regWrite <= 1;
                    branch <= 0;
                    jump <= 0;
                end
                // sw
                6'b101011: begin
                    memWrite <= 1;
                    memRead <= 0;
                    memToReg <= 0;
                    aluSecondSrc <= 1;
                    regDst <= 0;
                    regWrite <= 0;
                    branch <= 0;
                    jump <= 0;
                end
                default: // 11111111111111111111 terminating signal 
                begin
                    memWrite <= 0;
                    memRead <= 0;
                    memToReg <= 0;
                    aluSecondSrc <= 0;
                    regDst <= 0;
                    regWrite <= 0;
                    branch <= 0;
                    jump <= 0;
                end
                //     $display("Error: Unknown I-type instruction format.");
            endcase
        end
    endcase



    end

endmodule