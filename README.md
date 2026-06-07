# SystemVerilog Projects Repository

This repository hosts **SystemVerilog-based design and verification projects** focused on RTL modeling, state machines, assertions, and bus protocols. These projects serve as foundational exercises in digital logic and functional verification using SystemVerilog.

---

## Projects Overview

- **MESI Cache Coherence Protocol:**  
  Models a bus-based MESI coherence mechanism for a dual-core cache system. Includes RTL design, interface assertions, and functional coverage.  
  → See the [`MESI/`](./MESI) folder for full documentation and files.

---

## Tools & Technologies

- **Languages:** SystemVerilog  
- **Simulation Tools:** ModelSim / QuestaSim / Vivado / Verilator / Synopsys VCS  

---

## Future Additions

This repository will be expanded with:

- RTL design examples (ALU, priority encoders, arbiters)
- Assertion-based checkers for protocol rules
- SystemVerilog testbenches for common digital blocks

> UVM-based projects will be maintained separately in the [UVM Repository](https://github.com/sridurgaraju/UVM)

---

## Setup & Running Projects

To run a project:

1. Clone the repository and navigate to the project folder:
   ```bash
   git clone https://github.com/sridurgaraju/SystemVerilog.git
   cd SystemVerilog/<project-folder>
2. Compile and run using your preferred simulator:
   ```bash
    vlog *.sv
    vsim -c -do "run -all; quit" testbench
3. To view waveforms using GTKWave:
   ```bash
   gtkwave dump.vcd

---

## Contact
For questions, reach out via:
Email: sridurgaraju07@gmail.com
LinkedIn: https://www.linkedin.com/in/sri-durga-raju/

Happy verifying!

