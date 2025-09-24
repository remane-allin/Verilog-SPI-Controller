# Verilog SPI Master & Slave Controller

**SPI 主机和从机（Master & Slave）的全功能 Verilog HDL 实现。适用于 FPGA 或 ASIC 设计。**

---

## 🚀 概览与特性

这是一个简单、可综合的串行外设接口（SPI）控制器的 Verilog 实现。它包含两个独立的模块：

* **`spi_master.v`**: SPI 主机模块，负责生成时钟（SCLK）、控制片选（CS）并启动数据传输。
* **`spi_slave.v`**: SPI 从机模块，负责接收主机命令并响应数据。

**主要特性:**
* 支持 Mode 0 (CPOL=0, CPHA=0)。
* 可配置的数据位宽（默认 8-bit）。
* 清晰的控制信号和状态机。

## 📁 文件说明

| 文件名 | 描述 |
| :--- | :--- |
| `spi_master.v` | SPI 协议的主机（Master）端逻辑实现。 |
| `spi_slave.v` | SPI 协议的从机（Slave）端逻辑实现。 |
| `README.md` | 本文件。 |

## 💻 如何使用

要将此 IP 模块集成到您的设计中，您只需要例化 `spi_master` 和 `spi_slave` 模块，并连接您的顶层信号。

**[这里可以添加一个简单的 Verilog 例化代码片段]**

## 📜 许可证 (License)

本项目使用 **MIT 许可证** 开源。详情请参见 `LICENSE` 文件。
