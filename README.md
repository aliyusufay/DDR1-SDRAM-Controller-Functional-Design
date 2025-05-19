# 🧠 DDR1 SDRAM Functional Controller – SystemVerilog

A fully synthesizable, JEDEC JESD79F-compliant functional RTL controller for DDR1 SDRAM. This design integrates the control path, datapath, and auto-precharge execution logic. It is capable of issuing, sequencing, and validating DDR1 commands with full timing enforcement and a cycle-accurate burst datapath.

---

## ✨ Features

- **JEDEC DDR1-compliant controller** with command decoding:
  - ACT, READ, WRITE, PRECHARGE, REFRESH, MRS, NOP
- **Auto-precharge**: Triggered on A10=1 with burst-end logic
- **Timing constraints enforced**:  
  `tRCD`, `tRP`, `tRAS`, `tRFC`, `tCCD`, `tRTP`, `tRTW`, `tWR`, `tWTR`, `tRRD`, `tMRD`, `tXSRD`, `tXSNR`, `tXARD`, `tCKE`
- **Mode Register decode**: Burst length, CAS latency, burst type
- **Inter-bank support**: Concurrent timing-safe operations
- **Datapath**: Integrated burst counter, `burst_done`, and transfer flow
- **Self-refresh and power-down FSM** via CKE
- **Connectable to external functional DDR1 SDRAM model**

---

## 📂 File Structure

| File / Folder | Description |
|---------------|-------------|
| `rtl/ddr_controller.sv` | Top-level controller with FSM, timing, datapath, AP logic |
| `tb/ddr_controller_tb.sv` | Functional testbench to validate command sequences and timings |

---

## 🔄 Project Status

- ✅ Control path & FSM: Complete  
- ✅ Datapath & auto-precharge: Integrated  
- ✅ Functional testbench: Done  
- 🔄 Memory model connection: Pending (modular support in place)  
- 🔜 Optional: Add assertions for formal timing coverage
- 🔜 Optional: Comprehensive testbench

---

## 🧑‍💻 Author

**Ali Yusuf Askari Husain**  
M.Tech Microelectronics | RTL & Verification | Emulation (Intel Intern)  
📧 ali.yusuf.ay.110@gmail.com  
🔗 [LinkedIn](https://www.linkedin.com/in/ali-yusuf-73746a13a/)

---

## 🔗 Project Link

[GitHub Repo](https://github.com/aliyusufay/DDR1-SDRAM-Controller-Functional-Design)
