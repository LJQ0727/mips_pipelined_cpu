

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

    CPU cpu(clk);

    integer i;
    integer clkcount = 0;
    integer file;
    initial begin
        // Create file for memory dump
        // Filename: mem.txt
        file = $fopen("memdump.txt", "w");
    end

    always @(posedge clk) begin
        clkcount = clkcount + 1;
    end
    
    always @(posedge cpu.memwb.finish) begin
        // After executing the last memory writeback instruction

        // Print out total clock used
        $display("The total clock cycles used: %d", clkcount);
        
        // Print out memory info
        $display("The data memory after execution is:");
        for (i = 0; i < 512; i++)begin
            $display("%b",cpu.dataMem.DATA_RAM[i]);
            $fwrite(file, "%b\n", cpu.dataMem.DATA_RAM[i]);
        end
        $display("File: memdump.txt also stores the memory dump.");
        $finish;    // Terminate program
    end

    initial begin
        #0
        //$display("hello world");
        cpu.pc <= 0;
        cpu.stallSignal <= 0;
        cpu.pcIncrement <= 0;


        #3000      // Default finish
        $display("Error: No finish signal detected. Process aborted.");
        $finish;

    end

endmodule

