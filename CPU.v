`timescale 1ns/1ns

`include "InstructionRAM.v"
`include "RegisterFile.v"
`include "ALU.v"
`include "MainMemory.v"
`include "ControlUnit.v"


module CPU (
    input clk
);

    // Stall signal should be controlled by hazard detection unit
    // ???
    reg stallSignal;    // If asserted, stall the pc register and the fetch decode interface registers
    reg unstallSignal;  // Unstall the first two stages
    reg finish;     // Indicate whether the terminate signal is on

    // Maintain program counter
    reg[31:0] pc, pcIncrement;       // Maintaining the program counter register
    integer clkcount = 0;
    
    always @(posedge clk) begin
        //$display("posclk");
        pcIncrement = pc + 1;   // word-based
        clkcount = clkcount + 1;
        //$display("Clock: %d", clkcount);
    end

    always @(negedge clk) begin
        //$display("negclk");
        // Used blocking assignment here
        if (!stallSignal) begin
            //pc = exmem.pcBranched;
            case (exmem.pcSrc)
                2'b00: pc = pcIncrement;
                2'b01: pc = exmem.pcBranched;
                2'b10: pc = exmem.jumpAddr;
                default: begin
                    //$display("%b, Error: invalid pcSrc signal.", exmem.pcSrc);
                    pc = pcIncrement;
                end
            endcase
        end
    end

    // Initialize control values
    initial begin
        //exmem.pcSrc <= 2'b00;
    end

    wire [31:0] instruction;
    
    
    // Put together components
    InstructionRAM insMem(clk, 1'b0, 1'b1, pc, instruction);
    FetchDecodeInterface ifid(clk, instruction, stallSignal, pcIncrement);
    ControlUnit control(ifid.opcode, ifid.func);
    RegisterFile regfile(clk, ifid.rs, ifid.rt, 
        memwb.destRegField, memwb.writeDataReg, memwb.regWrite);
    DecodeExecuteInterface idex(clk, control.memWrite, 
        control.memRead, control.memToReg, hazard.aluSecondSrc, control.regDst,
        control.regWrite, control.branch, control.jump,
        ifid.pcIncrement, regfile.firstVal, regfile.secondVal,
        ifid.immediateSignExt, ifid.rt, ifid.rd, ifid.rs, ifid.func, ifid.opcode, ifid.sa,
        ifid.jumpAddr, ifid.finish, hazard.aluFirstSrc,
        exmem.aluResult, memwb.aluResult, control.aluSecondSrc);
    HazardDetectionUnit hazard(clk, exmem.destRegField, idex.rsField,
        idex.rtField, memwb.destRegField, idex.aluSecondSrc
        );
    ALU alu(clk, idex.aluFirstVal, idex.aluSecondVal, idex.func,
        idex.opcode, idex.sa);
    ExecuteMemoryInterface exmem(clk, idex.branch, idex.jump, idex.pcBranched,
        alu.zeroFlag, alu.result, idex.secondVal, idex.regWrite, idex.destRegField,
        idex.memRead, idex.memWrite, idex.memToReg, idex.jumpAddr, idex.finish);

always @(posedge clk) begin
    //$display("idex.aluSecondVal: %b", idex.aluSecondVal);
end

    // Prepare inputs to data memory
    wire [64:0] EDIT_SERIAL;
    //wire [31:0] DATA_TEMP; // unused
    assign EDIT_SERIAL[64] = exmem.memWrite;        // If want to write, assign this bit
    assign EDIT_SERIAL[63:32] = exmem.aluResult;
    assign EDIT_SERIAL[31:0] = exmem.secondVal;



    MainMemory dataMem(clk, 1'b0, exmem.memRead, exmem.aluResult, EDIT_SERIAL, exmem.memWrite);
    MemoryWriteBackInterface memwb(clk, exmem.regWrite, exmem.memToReg,
    dataMem.DATA, exmem.aluResult, exmem.destRegField, exmem.finish);

    

endmodule

module FetchDecodeInterface (
    clk, instruction, stallSignal, pcIncrementTemp
);
    // Input output declaration
    input clk;
    input [31:0] instruction;
    input stallSignal;
    input [31:0] pcIncrementTemp;

    // Split instruction into fields
    reg[5:0] opcode, func;
    reg[4:0] rs, rt, rd, sa;
    reg [15:0] immediate;
    reg signed [31:0] immediateSignExt;
    reg [31:0] jumpAddr;
    reg [31:0] pcIncrement;
    reg finish;

    initial begin
        finish <= 0;
    end

    always @(negedge clk) begin     // negedge so only write after finishing previous stage
            // We want this to happen after the above
            if (instruction == 32'b11111111111111111111111111111111) begin
                finish = 1'b1;
                               
            end

        if (!stallSignal) begin
            func <= instruction[5:0];
            opcode <= instruction[31:26];
            sa <= instruction[10:6];
            pcIncrement <= pcIncrementTemp;

            immediate <= instruction[15:0];       // I-type special
            immediateSignExt[31:16] <= {16{instruction[15]}};
            immediateSignExt[15:0] <= instruction[15:0];
            //$display("immediateSignExt: %d", immediateSignExt);

            rd <= instruction[15:11];
            rt <= instruction[20:16];
            rs <= instruction[25:21];
            

            // For J-type instructions
            jumpAddr[27:2] <= instruction[25:0];
            jumpAddr[1:0] <= 2'b00;
            jumpAddr[31:28] <= pcIncrement[31:28];


        end
    end
endmodule



module DecodeExecuteInterface (clk,
    memWriteTemp, memReadTemp, memToRegTemp,
    HAZARDaluSecondSrc, regDstTemp, regWriteTemp, 
    branchTemp, jumpTemp,
    pcIncrementTemp, firstValTemp,
    secondValTemp, immediateSignExtTemp,
    rtFieldTemp, rdFieldTemp, rsFieldTemp,
    funcTemp, opcodeTemp, saTemp, jumpAddrTemp, finishTemp, 
    HAZARDaluFirstSrc, EXMEMaluResult, MEMWBaluResult, aluSecondSrcTemp
);

    input clk, memWriteTemp, memReadTemp, memToRegTemp,
     regDstTemp, regWriteTemp, branchTemp, jumpTemp, finishTemp;     // Control signals
    input [31:0] pcIncrementTemp, firstValTemp,
     secondValTemp, immediateSignExtTemp, jumpAddrTemp, 
     EXMEMaluResult, MEMWBaluResult;        // IMPORTANT: passed from hazard detector

    //first operand
    // 00: rs
    // 01: EXMEM's aluResult
    // 10: MEMWB's aluResult

    // second operand
    // 00: rt
    // 01: immediate
    // 10: EXMEM's aluResult
    // 11: MEMWB's aluResult

    input [4:0] rtFieldTemp, rdFieldTemp, rsFieldTemp;
    input [5:0] funcTemp, opcodeTemp;
    input [4:0] saTemp;
    input [1:0] HAZARDaluSecondSrc, HAZARDaluFirstSrc, aluSecondSrcTemp;       // Control the source of the first and second ALU operand

    reg memWrite, memRead, memToReg, regDst, regWrite, branch, jump;     // Control signals
    reg [31:0] pcBranched, firstVal, secondVal, immediateSignExt, jumpAddr;
    reg [4:0] rtField, rdField, rsField;
    reg [5:0] func, opcode;
    reg [4:0] sa, destRegField;
    reg finish;
    reg [1:0] aluSecondSrc;

    reg [31:0] aluSecondVal, aluFirstVal;

    always @(negedge clk) begin
        //$display("secondValTemp received in idex: %b", secondValTemp);
        memWrite <= memWriteTemp;
        branch = branchTemp;
        jump = jumpTemp;
        memRead <= memReadTemp;
        memToReg <= memToRegTemp;
        aluSecondSrc <= aluSecondSrcTemp;
        //aluSecondSrc <= HAZARDaluSecondSrc;
        regDst <= regDstTemp;
        regWrite <= regWriteTemp;
        pcBranched <= pcIncrementTemp + immediateSignExtTemp;
        immediateSignExt <= immediateSignExtTemp;
        rtField <= rtFieldTemp;
        rdField <= rdFieldTemp;
        rsField <= rsFieldTemp;
        firstVal <= firstValTemp;
        secondVal <= secondValTemp;
        func <= funcTemp;
        opcode <= opcodeTemp;
        sa <= saTemp;
        jumpAddr <= jumpAddrTemp;
        destRegField <= regDstTemp ? rdFieldTemp : rtFieldTemp;
        finish <= finishTemp;

    end

    // RULE:
    // first operand
    // 00: rs
    // 01: EXMEM's aluResult
    // 10: MEMWB's aluResult

    // second operand
    // 00: rt
    // 01: immediate
    // 10: EXMEM's aluResult
    // 11: MEMWB's aluResult


    always @(negedge clk) begin
        #4
        //$display("setting alufirst and second val");
         $display("HAZARDaluFirstSrc: %b", HAZARDaluFirstSrc);
         $display("HAZARDaluSecondSrc: %b", HAZARDaluSecondSrc);
        case (HAZARDaluFirstSrc)
            2'b00: aluFirstVal = firstVal;
            2'b01: aluFirstVal = EXMEMaluResult;
            2'b10: aluFirstVal = MEMWBaluResult;
            //default: aluFirstVal = firstVal; //$display("Error with aluFirstSrc");
        endcase
        case (HAZARDaluSecondSrc)
            2'b00: aluSecondVal = secondVal;
            2'b01: aluSecondVal = immediateSignExt;
            2'b10: aluSecondVal = EXMEMaluResult;
            2'b11: aluSecondVal = MEMWBaluResult;
            //default: $display("error with HAZARDaluSecondSrc");
        endcase

        // $display("after hazard unit set aluSecondVal: %b", aluSecondVal);

        // $display("destRegField in deex: %d", destRegField);
        // $display("regDstTemp: %d", regDstTemp);
        // $display("rtfieldTemp: %d", rtFieldTemp);
    end

endmodule

module ExecuteMemoryInterface (
    clk,
     branchTemp, // if asserted, will branch
     jumpTemp,
     pcBranchedTemp,  // The branched pc address; to be selected
     zeroFlagTemp,  // For branch
    ALUresultTemp, 
    secondValTemp, // To be forwarded to write data to data mem
    regWriteTemp, // If asserted, will perform register write
    destRegFieldTemp,   // The register number to be written to
    memReadTemp,     // Controls whether to read memory
    memWriteTemp,    // Whether to write to memory
    memToRegTemp,    // Whether pass data from data mem or not
    jumpAddrTemp,
    finishTemp
);
    input clk, branchTemp, zeroFlagTemp, regWriteTemp, memReadTemp, 
        memWriteTemp, memToRegTemp, jumpTemp, finishTemp;
    input [31:0] pcBranchedTemp, ALUresultTemp, secondValTemp,
        jumpAddrTemp;
    input [4:0] destRegFieldTemp;

    reg regWrite, memRead, memWrite, memToReg;
    reg [31:0] pcBranched, aluResult, secondVal, jumpAddr;
    reg [4:0] destRegField;
    reg [1:0] pcSrc;
    reg finish;


    always @(negedge clk) begin
        regWrite <= regWriteTemp;
        memRead <= memReadTemp;
        memWrite <= memWriteTemp;
        memToReg <= memToRegTemp;
        pcBranched <= pcBranchedTemp;
        aluResult <= ALUresultTemp;
        secondVal <= secondValTemp;     // To data memory's "write data"
        jumpAddr <= jumpAddrTemp;
        destRegField <= destRegFieldTemp;
        finish <= finishTemp;
        //$display("destRegField in exmem: %d", destRegField);

        pcSrc[1] <= (jumpTemp == 1) ? 1 : 0;
        pcSrc[0] <= ((branchTemp == 1) && (zeroFlagTemp == 1)) ? 1 : 0;

        //$display("secondVal: %d", secondVal);


    end
endmodule

module MemoryWriteBackInterface (
    clk, regWriteTemp, memToRegTemp, readDataMemTemp, ALUresultTemp, 
    destRegFieldTemp, finishTemp
);
    input clk, regWriteTemp, memToRegTemp, finishTemp;
    input [31:0] readDataMemTemp, ALUresultTemp;
    input [4:0] destRegFieldTemp;
    reg regWrite, memToReg;
    reg [31:0] writeDataReg, aluResult;
    reg [4:0] destRegField;
    reg finish;

    always @(negedge clk) begin
        regWrite <= regWriteTemp;
        memToReg <= memToRegTemp;
        finish <= finishTemp;
        aluResult <= ALUresultTemp;

        // These two combined to write to register file
        destRegField <= destRegFieldTemp;
        //$display("destRegField in memwb: %d", destRegField);
        writeDataReg <= memToRegTemp ? readDataMemTemp : ALUresultTemp;

    end
endmodule

module ForwardingUnit (
    stallSignal
);
    output stallSignal;

    // TODO





endmodule

// Does forwarding jobs
module HazardDetectionUnit (clk,
    EXMEMdestRegField, IDEXrs, IDEXrt, MEMWBdestRegField, 
    aluSecondSrcTemp    // passed from control unit
    //EXMEMaluResult, MEMWBaluResult

);
    input [4:0] EXMEMdestRegField, IDEXrs, IDEXrt, MEMWBdestRegField;
    input clk;
    reg [1:0] aluFirstSrc;        // If asserted, will change input to the first ALU operand
    input [1:0] aluSecondSrcTemp;       // We should only set the MSB here
    reg [1:0] aluSecondSrc;
    //first operand
    // 00: rs
    // 01: EXMEM's aluResult
    // 10: MEMWB's aluResult

    // second operand
    // 00: rt
    // 01: immediate
    // 10: EXMEM's aluResult
    // 11: MEMWB's aluResult


    // No need the CLK?????
    // Asynchronous?????
    
    always @(negedge clk) begin
        #1      // After the interfaces are set
        //$display("setting hazard");
        aluFirstSrc = 2'b00;        // Defaults to 0
        aluSecondSrc = aluSecondSrcTemp;        // Defaults

        // $display("%b",EXMEMdestRegField);
        // $display("%b",IDEXrs);
        // $display("%b",IDEXrt);


        // Case 1: EX/MEM.destination register = ID/EX.register rs
        if (EXMEMdestRegField == IDEXrs)begin
            // Supply the ALU first operand to be the EXMEM's aluResult
            aluFirstSrc = 2'b01;
            $display("hazard: case 1");
        end

        // Case 2: EX/MEM.destination register = ID/EX.register rt
        if (EXMEMdestRegField == IDEXrt)begin
            // Supply the ALU second operand to be the EXMEM's aluResult
            if (aluSecondSrcTemp[0] != 1) begin aluSecondSrc = 2'b10;
            $display("hazard: case 2");
            end
        end

        // Case 3: MEM/WB.destination register = ID/EX.register rs
        if (MEMWBdestRegField == IDEXrs)begin
            // Supply the ALU first operand to be the MEMWB's aluResult
            aluFirstSrc = 2'b10;
            $display("hazard: case 3");
        end

        // Case 4: MEM/WB.destination register = ID/EX.register rt
        if (MEMWBdestRegField == IDEXrt)begin
            // Supply the ALU second operand to be the MEMWB's aluResult
            if (aluSecondSrcTemp[0] != 1) begin aluSecondSrc = 2'b11;
            $display("hazard: case 4");
            end
        end

        // $display("xxxxxxxxxxxxxxxx");
         //$display("in hazard unit firstsrc: %b", aluFirstSrc);
        // $display("in hazard unit secondsrc: %b", aluSecondSrc);

        // $display("%b",aluFirstSrc);
        // $display("%b",aluSecondSrc);



    end



endmodule