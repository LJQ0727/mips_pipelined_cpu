`timescale 1ns/1ns

`include "InstructionRAM.v"
`include "RegisterFile.v"
`include "ALU.v"
`include "MainMemory.v"
`include "ControlUnit.v"

// Note: 1 clock is 10 ns for this cpu

module CPU (
    input clk
);

    // Stall signal should be controlled by hazard detection unit
    reg stallSignal;    // If asserted, stall the pc register and the fetch decode interface registers
    reg finish;     // Indicate whether the terminate signal is on
    wire [31:0] instruction;

    // Maintain program counter
    reg[31:0] pc, pcIncrement;       // Maintaining the program counter register
    integer clkcount = 0;
    
    always @(posedge clk) begin
        $display("posclk");
        pcIncrement = pc + 1;   // word-based
        clkcount = clkcount + 1;
        //$display("Clock: %d", clkcount);
        //$display("stallSignal: %b", stallSignal);
    end

    always @(negedge clk) begin
        $display("negclk");
        //$display("instruction: %b", instruction);
    end

    // Set program counter
    always @(negedge clk) begin
        #4
        // Used blocking assignment here
        if (!stallSignal) begin
            case (exmem.pcSrc)
                2'b00: pc = pcIncrement;
                2'b01: begin 
                    pc = exmem.pcBranched;
                    $display("branching to %d", pc);
                end
                2'b10: pc = exmem.jumpAddr;
                default: begin
                    //$display("%b, Error: invalid pcSrc signal.", exmem.pcSrc);
                    pc = pcIncrement;
                end
            endcase
        end
    end

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
        exmem.aluResult, memwb.aluResult, control.aluSecondSrc,
        memwb.readDataMem);
    HazardDetectionUnit hazard(clk, exmem.destRegField, idex.rsField,
        idex.rtField, memwb.destRegField, memwb.memToReg
        );
    ALU alu(clk, idex.aluFirstVal, idex.aluSecondVal, idex.func,
        idex.opcode, idex.sa);
    ExecuteMemoryInterface exmem(clk, idex.branch, idex.jump, idex.pcBranched,
        alu.zeroFlag, alu.result, idex.aluSecondValTemp, idex.regWrite, idex.destRegField,
        idex.memRead, idex.memWrite, idex.memToReg, idex.jumpAddr, idex.finish);

    // Prepare inputs to data memory
    wire [64:0] EDIT_SERIAL;
    //wire [31:0] DATA_TEMP; // unused
    assign EDIT_SERIAL[64] = exmem.memWrite;        // If want to write, assign this bit
    assign EDIT_SERIAL[63:32] = exmem.aluResult;
    assign EDIT_SERIAL[31:0] = exmem.secondVal;

    MainMemory dataMem(clk, 1'b0, exmem.memRead, exmem.aluResult, EDIT_SERIAL, exmem.memWrite);
    MemoryWriteBackInterface memwb(clk, exmem.regWrite, exmem.memToReg,
    dataMem.DATA, exmem.aluResult, exmem.destRegField, exmem.finish);

    // 1. For lw stall hazard
    always @(posedge idex.memRead) begin       // at certain negedge clk
        #1  // After the interface values are set
        if (idex.rtField == ifid.rs || idex.rtField == ifid.rt) begin 
            stallSignal <= 1'b1;
            $display("lw stall");
            
            #13 // after next cycle's negedge
            // Insert nop to the next instruction
            idex.memWrite <= 1'b0;
            idex.memRead <= 1'b0;
            idex.memToReg <= 1'b0;
            idex.aluSecondSrc <= 1'b0;
            idex.regDst <= 1'b0;
            idex.regWrite <= 1'b0;
            idex.branch <= 1'b0;
            idex.jump <= 1'b0;
            idex.rtField <= 5'b00000;
            idex.rdField <= 5'b00000;
            // idex.aluSecondVal <= 0;
            idex.aluFirstVal <= 12345;
            idex.aluSecondVal <= 0;
            $display("cleared the control");

            // Unstall the next next posedge
            stallSignal <= 1'b0;
            $display("unstalled the signal");
        end
        
    end

    // 2. For branch hazard
    always @(negedge clk) begin     
        // We assume branch not taken and continue execution
        // However, when the branch is taken, should eliminate the executed ones
        #1
        if (exmem.pcSrc == 2'b01) begin      // branch taken
            // Eliminate information of 3 interfaces
            
            // Overwrite to do nothing on these instructions
            ifid.opcode <= 0;
            ifid.func <= 0;
            ifid.rs <= 0;
            ifid.rt <= 0;
            ifid.rd <= 0;
            ifid.sa <= 0;
            ifid.immediate <= 0;
            ifid.immediateSignExt <= 0;
            ifid.jumpAddr <= 0;
            ifid.pcIncrement <= 0;
            ifid.finish <= 0;


            idex.memWrite <= 0;
            idex.memRead <= 0;
            idex.memToReg <= 0;
            idex.aluSecondSrc <= 0;
            idex.regDst <= 0;
            idex.regWrite <= 0;
            idex.branch <= 0;
            idex.jump <= 0;

            $display("Branch hazard countered");
        end

    end

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

        if (!stallSignal) begin     // If is stalled, everything in FetchDecodeInterface will remain unchanged
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
    HAZARDaluFirstSrc, EXMEMaluResult, MEMWBaluResult, aluSecondSrcTemp,
    MEMWBreadDataMem
);

    input clk, memWriteTemp, memReadTemp, memToRegTemp,
     regDstTemp, regWriteTemp, branchTemp, jumpTemp, finishTemp;     // Control signals
    input [31:0] pcIncrementTemp, firstValTemp,
     secondValTemp, immediateSignExtTemp, jumpAddrTemp, 
     EXMEMaluResult, MEMWBaluResult, MEMWBreadDataMem;        // IMPORTANT: passed from hazard detector

    //first operand
    // 00: rs
    // 01: EXMEM's aluResult
    // 10: MEMWB's aluResult

    // second operand
    // 00: rt
    // 01: EXMEM's aluResult
    // 10: MEMWB's aluResult

    // second operand extended filter
    // 0: from last one
    // 1: immediate sign extend

    input [4:0] rtFieldTemp, rdFieldTemp, rsFieldTemp;
    input [5:0] funcTemp, opcodeTemp;
    input [4:0] saTemp;
    input [1:0] HAZARDaluSecondSrc, HAZARDaluFirstSrc;       // Control the source of the first and second ALU operand
    input aluSecondSrcTemp;

    reg memWrite, memRead, memToReg, regDst, regWrite, branch, jump;     // Control signals
    reg [31:0] pcBranched, firstVal, secondVal, aluSecondValTemp, immediateSignExt, jumpAddr;
    reg [4:0] rtField, rdField, rsField;
    reg [5:0] func, opcode;
    reg [4:0] sa, destRegField;
    reg finish;
    reg [1:0] aluSecondSrc;

    reg [31:0] aluSecondVal, aluFirstVal;

    always @(negedge clk) begin
        //$display("secondValTemp received in idex: %b", secondValTemp);
        memWrite <= memWriteTemp;
        branch <= branchTemp;
        jump <= jumpTemp;
        memRead <= memReadTemp;
        memToReg <= memToRegTemp;
        aluSecondSrc <= aluSecondSrcTemp;
        //aluSecondSrc <= HAZARDaluSecondSrc;
        regDst <= regDstTemp;
        regWrite <= regWriteTemp;
        pcBranched <= pcIncrementTemp + immediateSignExtTemp;   // The branch destination addr
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
    //first operand
    // 00: rs
    // 01: EXMEM's aluResult
    // 10: MEMWB's aluResult

    // second operand
    // 00: rt
    // 01: EXMEM's aluResult
    // 10: MEMWB's aluResult

    // second operand extended filter
    // 0: from last one
    // 1: immediate sign extend


    always @(negedge clk) begin
        #2      // After the hazard signals are set

        //$display("setting alufirst and second val");
        // $display("branch in idex: %b", branch);
        //   $display("HAZARDaluSecondSrc: %b", HAZARDaluSecondSrc);
        //   $display("MEMWBreadDataMem: %d", MEMWBreadDataMem);
        //  $display("aluSecondSrc: %b", aluSecondSrc);
        // $display("immediateSignExt: %b", immediateSignExt);

        case (HAZARDaluFirstSrc)
            2'b00: aluFirstVal = firstVal;
            2'b01: aluFirstVal = EXMEMaluResult;
            2'b10: aluFirstVal = MEMWBaluResult;
            //default: aluFirstVal = firstVal; //$display("Error with aluFirstSrc");
        endcase

        case (HAZARDaluSecondSrc)
            2'b00: aluSecondValTemp = secondVal;
            2'b01: aluSecondValTemp = EXMEMaluResult;
            2'b10: aluSecondValTemp = MEMWBaluResult;
            2'b11: aluSecondValTemp = MEMWBreadDataMem;
            //default: $display("error with HAZARDaluSecondSrc");
        endcase

        case (aluSecondSrc)
            1'b0: aluSecondVal = aluSecondValTemp;
            1'b1: aluSecondVal = immediateSignExt;
            //default: 
        endcase


        // $display("after hazard unit set aluSecondVal: %b", aluSecondVal);

        //  $display("aluSecondSrc in idex: %d", aluSecondSrc);
        //  $display("immediateSignExt in idex: %d", immediateSignExt);
        //  $display("aluSecondVal in idex: %d", aluSecondVal);
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
        // branch <= branchTemp;
        // jump <= jumpTemp;
        //$display("destRegField in exmem: %d", destRegField);

        pcSrc[1] <= (jumpTemp == 1) ? 1 : 0;
        pcSrc[0] <= ((branchTemp == 1) && (ALUresultTemp == 1)) ? 1 : 0;
        $display("branch: %b", branchTemp);
        //$display("aluResult: %d", aluResult);
        if ((branchTemp == 1) && (ALUresultTemp == 1)) $display("branching...");

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
    reg [31:0] readDataMem;

    always @(negedge clk) begin
        regWrite <= regWriteTemp;
        memToReg <= memToRegTemp;
        finish <= finishTemp;
        aluResult <= ALUresultTemp;
        readDataMem <= readDataMemTemp;
        // These two combined to write to register file
        destRegField <= destRegFieldTemp;
        //$display("destRegField in memwb: %d", destRegField);
        writeDataReg <= memToRegTemp ? readDataMemTemp : ALUresultTemp;

    end
endmodule


// Does forwarding jobs
module HazardDetectionUnit (clk,
    EXMEMdestRegField, IDEXrs, IDEXrt, MEMWBdestRegField, MEMWBmemToReg
        // passed from control unit
    //EXMEMaluResult, MEMWBaluResult

);
    input [4:0] EXMEMdestRegField, IDEXrs, IDEXrt, MEMWBdestRegField;
    input clk, MEMWBmemToReg;
    reg [1:0] aluFirstSrc;        // If asserted, will change input to the first ALU operand
    reg [1:0] aluSecondSrc;

    // RULE:
    //first operand
    // 00: rs
    // 01: EXMEM's aluResult
    // 10: MEMWB's aluResult

    // second operand
    // 00: rt
    // 01: EXMEM's aluResult
    // 10: MEMWB's aluResult

    
    always @(negedge clk) begin
        #1      // After the interfaces are set
        //$display("setting hazard");
        aluFirstSrc = 2'b00;        // Defaults to 0
        aluSecondSrc = 2'b00;        // Defaults to 0

        // $display("%b",EXMEMdestRegField);
        // $display("%b",IDEXrs);


        // Case 1: EX/MEM.destination register = ID/EX.register rs
        if (EXMEMdestRegField == IDEXrs)begin
            // Supply the ALU first operand to be the EXMEM's aluResult
            aluFirstSrc = 2'b01;
            $display("hazard: case 1");
        end

        // $display("!EXMEMdestRegField: %b", EXMEMdestRegField);
        // $display("!IDExrt: %b",IDEXrt);
        // Case 2: EX/MEM.destination register = ID/EX.register rt
        if (EXMEMdestRegField == IDEXrt)begin
            // Supply the ALU second operand to be the EXMEM's aluResult
            aluSecondSrc = 2'b01;
            $display("hazard: case 2");
        end

        // Case 3: MEM/WB.destination register = ID/EX.register rs
        if (MEMWBdestRegField == IDEXrs)begin
            // Supply the ALU first operand to be the MEMWB's aluResult
            if (MEMWBmemToReg == 1) begin 
                aluSecondSrc = 2'b11;
                $display("hazard: case 3 for lw");
            end
            else begin aluSecondSrc = 2'b10;
                $display("hazard: case 3 for register");
            end
        end

        // Case 4: MEM/WB.destination register = ID/EX.register rt
        if (MEMWBdestRegField == IDEXrt)begin
            // Supply the ALU second operand to be the MEMWB's aluResult
            if (MEMWBmemToReg == 1) begin 
                aluSecondSrc = 2'b11;
                $display("hazard: case 4 for lw");
            end
            else begin aluSecondSrc = 2'b10;
                $display("hazard: case 4 for register");
            end
        end


        // $display("xxxxxxxxxxxxxxxx");
         //$display("in hazard unit firstsrc: %b", aluFirstSrc);
        // $display("in hazard unit secondsrc: %b", aluSecondSrc);

    end

endmodule
