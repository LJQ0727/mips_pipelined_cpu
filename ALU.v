// The ALU part


module ALU (
    clk, firstVal, secondVal, func, opcode, sa
);


    // input output declaration
    input clk;
    input [31:0] firstVal;  // One of the two operands
    input [31:0] secondVal; // One of the two operands
    input [5:0] func;       // The func field of the instrcution
    input [5:0] opcode;     // The opcode field; togather to determine the operation type to perform
    input [4:0] sa;         // Shift amount, for R-type instructions
    reg [31:0] result;
    reg zeroFlag;
    //output reg overflowFlag;
    //output reg negativeFlag;
    

    wire signed [31:0] firstValSigned;
    assign firstValSigned = firstVal;
    wire signed [31:0] secondValSigned;
    assign secondValSigned = secondVal;



    // The arithmetic logics
    always @(posedge clk) begin
        #1
        //$display("alu received secondVal: %b",secondVal);
        //$display("opcode: %b",opcode);
        //$display("func: %b",func);
        // set default flags to 0
        //overflowFlag = 0;
        zeroFlag = 0;
        //negativeFlag = 0;

        // assign sa (if exists), = secondVal
        // use mux for that selection

        // The logics
        case (opcode)
            6'b000000: // R-type instructions
            begin
                case (func)
                    // add
                    6'b100000: 
                    begin 
                        result = firstVal + secondVal;
                        //$display("add called, %d, %d", firstVal, secondVal);
                        // detect and set overflow
                        //overflowFlag = (firstVal[31] ^ secondVal[31]) ? 0 : (firstVal[31] ^ result[31]);
                        //$display("overflowFlag: ", overflowFlag);
                    end
                    6'b100001: begin // addu
                        result = firstVal + secondVal;
                    end
                    // sub
                    6'b100010: begin
                        result = firstVal - secondVal;
                        $display("sub %d - %d", firstVal,secondVal);
                        //secondVal = ~secondVal + 1;
                        // detect overflow
                        //$display("%b",secondVal);
                        //overflowFlag = (firstVal[31] ^ secondVal[31]) ? 0 : (firstVal[31] ^ result[31]);
                    end
                    // subu 
                    6'b100011: result = firstVal - secondVal;
                    // and
                    6'b100100: result = firstVal & secondVal;
                    // nor
                    6'b100111: result = ~(firstVal | secondVal);
                    // or
                    6'b100101: result = firstVal | secondVal;
                    // xor
                    6'b100110: result = firstVal ^ secondVal;
                    // slt
                    6'b101010: begin
                        result = (firstVal < secondVal);        // Directly set the result to indicate less than; removes negative flag
                        $display("slt called: %d %d; result: %d", firstVal, secondVal, result);
                        // set negative flag
                        //negativeFlag = (firstValSigned < secondValSigned);
                    end
                    // srl
                    6'b000010: result = secondVal >> sa;
                    // srlv
                    6'b000110: result = secondVal >> firstVal;
                    // sra
                    6'b000011: result = secondValSigned >>> sa;
                    // srav
                    6'b000111: result = secondValSigned >>> firstVal;
                    // sll
                    6'b000000: result = secondVal << sa;
                    // sllv
                    6'b000100: result = secondVal << firstVal;
                    // jr
                    6'b001000: begin 
                        result = firstVal;
                        $display("jr detected in ALU");
                    end

                    // default: // should not occur
                    // $display("Error: Unknown R-type instruction format.");
                endcase
            end

            6'b000011:      // jal's opcode
            begin
                $display("jal detected in ALU");
                result = firstVal;
            end

                
            // addi
            6'b001000: begin
                result = firstVal + secondVal;       // second val is immediate
                $display("addi called in alu, %d, %d", firstVal, secondVal);
                // detect overflow
                //overflowFlag = (firstVal[31] ^ immediate[15]) ? 0 : (firstVal[31] ^ result[31]);
            end
            // addiu
            6'b001001:begin
                $display("addiu called, %d, %d", firstVal, secondVal);
                result = firstVal + secondVal;       // second val is immediate
            end
            // andi
            6'b001100: result = firstVal & (secondVal & 32'h0000ffff);       // second val is immediate
            // ori
            6'b001101: result = firstVal | (secondVal & 32'h0000ffff);        // second val is immediate
            // xori
            // Caution: zero-extend immediate
            6'b001110: result = firstVal ^ (secondVal & 32'h0000ffff);       // second val is immediate
            // beq
            6'b000100: begin
                result = ((firstVal - secondVal) == 0);
                if (result == 0) zeroFlag = 1;
                $display("beq called: comaparing %d %d", firstVal, secondVal);
            end
            // bne
            6'b000101: begin
                //result = immediate << 2;
                result = ((firstVal - secondVal) != 0);
                if (result == 0) zeroFlag = 1;      // Should use result to indicate branch or not; zeroFLag is depreciated
                $display("bne called: comaparing %d %d; result %d", firstVal, secondVal, result);
            end
            // lw
            6'b100011: begin 
                result = firstVal + secondVal/4;        // added to the sign-extended immediate, no shifting
                    $display("lw called: %d %d", firstVal, secondVal);
                // $display("result: %d", result);

            end
            // sw
            6'b101011: begin 
                result = firstVal + secondVal/4;        // added to the sign-extended immediate, no shifting
                    $display("sw called: %d %d", firstVal, secondVal);
                // $display("result: %d", result);
            end

            // default: // should not occur
            //     $display("Error: Unknown I-type instruction format.");
        endcase
    end
endmodule



