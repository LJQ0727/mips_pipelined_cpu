// The register file stage
// Including both the read and write pipe stage

module RegisterFile (
    clk, firstRegNumber, secondRegNumber, writeRegNumber, writeData, regWriteSignal
);
    // Containing 32 32-bit registers
    
    input clk;
    input [4:0] firstRegNumber, secondRegNumber, writeRegNumber;
    input [31:0] writeData;
    input regWriteSignal;
    reg [31:0] firstVal, secondVal;      // The retrieved register values


    // Declare registers
    reg [31:0] registers [31:0];


    integer i;
    initial begin
        // Initialize all registers to 0
        for (i = 0; i < 31; i = i + 1) begin
            registers[i] = 0;
        end
    end


    always @(posedge clk) begin
        
        // read
        firstVal <= registers[firstRegNumber];
        secondVal <= registers[secondRegNumber];
        //   $display("readRegNumber: %d", firstRegNumber);
        //  $display("readData: %d", registers[firstRegNumber]);
    end

    always @(posedge clk) begin
        //$display("writeData: %d writeRegNum: %d writeSignal: %d", writeData, writeRegNumber, regWriteSignal);
        // write, if necessary
        if (regWriteSignal) begin
            // $display("prepare to write to regfile");
            // $display("writeRegNumber: %d", writeRegNumber);
            // $display("writeData: %d", writeData);
            if (writeRegNumber != 5'b00000)begin
                registers[writeRegNumber] = writeData;  // Register 0 is always kept 0
                //$display(registers[writeRegNumber]);
                $display("write %d to register number %d", writeData, writeRegNumber);
            end
        end
    end
endmodule