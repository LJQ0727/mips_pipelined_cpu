

// Initialize CPU instance
// output memory state to console


`include "CPU.v"
`timescale 1ns/1ns


module test_CPU();
    
    reg clk;
    // Set up clock
    initial begin
        clk = 0;
    end
    always #5 clk = ~clk;   // One clock cycle is 10 ns

    reg [4:0] firstRegNumber, secondRegNumber;
    reg [31:0] writeRegNumber, writeData;
    reg regWriteSignal;
    wire [31:0] firstVal, secondVal;      // The retrieved register values

    
    CPU cpu(clk);

    integer i;
    
    always @(posedge cpu.memwb.finish) begin
          // After executing the last memory writeback instruction
        // Print out memory info
        
        $display("The data memory after execution is:");
        for (i = 0; i < 512; i++)begin
            //$display("%b",cpu.dataMem.DATA_RAM[i]);
        end
        $finish;    // Terminate program
    end

    initial begin
        #0
        //$display("hello world");
        cpu.pc <= 0;
        cpu.stallSignal <= 0;
        cpu.pcIncrement <= 0;
        //$monitor(cpu.pc);

        //$monitor("stall: %b",cpu.stallSignal);

        #3000      // Default finish
        $display("Error: No finish signal detected. Process aborted.");
        $finish;

    end

endmodule

