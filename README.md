# Register-transfer Level Programming: A Single-cycle CPU

## Overview

This project contains assembly and Verilog code for implementing a single-cycle CPU, including factorial calculation and leaf function. The project is organized into two main directories: `Assembly` and `Verilog`.

## Directory Structure
Assembly/
    fact.s
    hw1.s
    leaf.s
Verilog/
    CHIP.v
    fact/
        fact_data_ans.txt
        fact_data.txt
        fact_gen.py
        fact_text.txt
    Final_tb.v
    hw1/
        hw1_data_ans.txt
        hw1_data.txt
        hw1_gen.py
        hw1_text.txt
    leaf/
        leaf_data_ans.txt
        leaf_data.txt
        leaf_gen.py
        leaf_text.txt
    license.cshrc
    memory.v


### Assembly Directory

- `Assembly/fact.s`: Assembly code for calculating the factorial of a number.
- `Assembly/hw1.s`: Assembly code for an assignment.
- `Assembly/leaf.s`: Assembly code for the leaf function.

### Verilog Directory

- `Verilog/CHIP.v`: Verilog code for the main chip module.
- `Verilog/Final_tb.v`: Testbench for the implemented CPU.
- `Verilog/memory.v`: Verilog code for the memory module.
- `Verilog/license.cshrc`: License configuration file.

#### Fact Directory

- `Verilog/fact/fact_data_ans.txt`: Expected output data for the factorial calculation.
- `Verilog/fact/fact_data.txt`: Input data for the factorial calculation.
- `Verilog/fact/fact_gen.py`: Python script to generate input and expected output data for the factorial calculation.
- `Verilog/fact/fact_text.txt`: Assembly instructions for the factorial calculation.

#### HW1 Directory

- `Verilog/hw1/hw1_data_ans.txt`: Expected output data for the homework assignment.
- `Verilog/hw1/hw1_data.txt`: Input data for the assignment.
- `Verilog/hw1/hw1_gen.py`: Python script to generate input and expected output data for the homework assignment.
- `Verilog/hw1/hw1_text.txt`: Assembly instructions for the homework assignment.

#### Leaf Directory

- `Verilog/leaf/leaf_data_ans.txt`: Expected output data for the leaf function.
- `Verilog/leaf/leaf_data.txt`: Input data for the leaf function.
- `Verilog/leaf/leaf_gen.py`: Python script to generate input and expected output data for the leaf function.
- `Verilog/leaf/leaf_text.txt`: Assembly instructions for the leaf function.

## How to Run

### Assembly Code

1. Open the assembly file you want to run (e.g., `Assembly/fact.s`, `Assembly/hw1.s`, `Assembly/leaf.s`).
2. Assemble and run the code using your preferred assembler and simulator.

### Verilog Code

1. Open the Verilog testbench file `Verilog/Final_tb.v`.
2. Ensure the correct `MEM_TEXT`, `MEM_DATA`, and `MEM_DATA_ANS` macros are defined for the project you want to test.
3. Run the simulation using your preferred Verilog simulator.

### Python Scripts

1. Navigate to the directory containing the Python script (e.g., `Verilog/fact`, `Verilog/hw1`, `Verilog/leaf`).
2. Run the script to generate input and expected output data:
   ```sh
   python fact_gen.py
   python hw1_gen.py
   python leaf_gen.py

## License
This project uses the license configurations specified in Verilog/license.cshrc.

## Contact
For any questions or issues, please contact the project maintainer. 