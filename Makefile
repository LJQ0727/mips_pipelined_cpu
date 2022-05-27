test_CPU.out: test_CPU.v
	iverilog -o test_CPU.out test_CPU.v
	vvp test_CPU.out

.PHONY: clean
clean: 
	rm test_CPU.out memdump.txt