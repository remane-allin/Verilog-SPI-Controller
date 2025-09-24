# Verilog SPI Master & Slave Controller

**SPI 主机和从机（Master & Slave）的全功能 Verilog HDL 实现。适用于 FPGA 或 ASIC 设计。**

---

## 🚀 概览与特性

这是一个简单、可综合的串行外设接口（SPI）控制器的 Verilog 实现。它包含两个独立的模块：

* **`spi_master.v`**: SPI 主机模块，负责生成时钟（SCLK）、控制片选（CS）并启动数据传输。
* **`spi_slave.v`**: SPI 从机模块，负责接收主机命令并响应数据。
* **`tb_spi_phy_loopback.v`**: 仿真代码，包含对 Master 写、Master 读和全双工的验证。

**主要特性:**
*  Mode 0 (CPOL=0, CPHA=0)。

## 📁 文件说明

| 文件名 | 描述 |
| :--- | :--- |
| `spi_master.v` | SPI 协议的主机（Master）端逻辑实现。 |
| `spi_slave.v` | SPI 协议的从机（Slave）端逻辑实现。 |
| `tb_spi_phy_loopback.v` | SPI 仿真。 |
| `README.md` | 本文件。 |

## 💻 如何使用

需要添加 `spi_master` 、 `spi_slave` 模块 `tb` 即可。

## 💻 博客链接
https://blog.csdn.net/guogaia/article/details/152015473?spm=1001.2014.3001.5501

## 📜 许可证 (License)

本项目使用 **MIT 许可证** 开源。详情请参见 `LICENSE` 文件。
