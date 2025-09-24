//==============================================================================
// File Name    : spi_slave.v
// Description  : SPI从机物理接口模块 - 请求/应答架构
//                实现SPI Mode 0 (CPOL=0, CPHA=0) 协议时序
//                采用请求/应答模式，作为纯粹的比特流转换器
// Author       :
// Date         :
// Version      : 2.0
//==============================================================================
//
// 功能说明:
// --------
// 本模块实现了SPI从机物理层接口，专注于比特流转换功能。
// 采用请求/应答模式与协议层通信，确保可靠的握手时序。
//
// 核心设计理念:
// --------
// 1. 纯粹的比特流转换器:
//    - 物理层只负责串行/并行转换
//    - 不参与复杂的状态判断和控制逻辑
//    - 专注于可靠的时序处理
//
// 2. 请求/应答通信模式:
//    - 协议层通过phy_tx_req请求发送字节
//    - 物理层通过phy_tx_ack应答发送完成
//    - 单向命令模式，杜绝时序竞争
//
// 3. 输出驱动保证:
//    - tx_enable由协议层全程控制
//    - 物理层在tx_enable有效期间始终驱动so
//    - 从机制上杜绝高阻态问题
//
// 主要特性:
// --------
// 1. 协议兼容性:
//    - 严格遵循SPI Mode 0时序 (CPOL=0, CPHA=0)
//    - 数据在SCK上升沿采样，下降沿更新
//    - 支持标准的CS片选控制
//
// 2. 数据接收功能:
//    - 8位串行到并行转换
//    - 自动字节边界检测
//    - 单周期接收完成脉冲通知
//
// 3. 数据发送功能:
//    - 8位并行到串行转换
//    - 请求/应答式发送控制
//    - 可靠的输出驱动机制
//
// 4. 同步设计:
//    - 所有逻辑同步到系统时钟
//    - 异步SPI信号的同步处理
//    - 边沿检测和防亚稳态设计
//
// 接口说明:
// --------
// 系统接口:
//   clk        - 系统时钟
//   op_reset   - 操作复位(高电平有效)
//
// SPI物理接口:
//   cs_n       - 片选信号(低电平有效)
//   sck        - SPI时钟
//   si         - 串行输入
//   so         - 串行输出(三态)
//
// 协议层接口:
//   tx_data    - 发送数据(8位)
//   tx_enable  - 发送使能(协议层控制)
//   phy_tx_req - 发送请求脉冲(协议层->物理层)
//   phy_tx_ack - 发送应答脉冲(物理层->协议层)
//   rx_data    - 接收数据(8位)
//   rx_valid   - 接收完成脉冲
//
// 工作时序:
// --------
// 1. 接收时序:
//    CS下降沿 -> 开始接收 -> SCK上升沿采样SI -> 8位后产生rx_valid脉冲
//
// 2. 发送时序:
//    协议层拉高tx_enable -> 准备tx_data -> 发出phy_tx_req脉冲 ->
//    物理层锁存数据开始发送 -> 8位发送完成后产生phy_tx_ack脉冲
//
// 3. 复位时序:
//    CS上升沿或op_reset高电平 -> 所有内部状态复位 -> 准备下次传输
//
//==============================================================================

module spi_slave (
    // 系统时钟和复位
    input  wire         clk,            // 系统时钟
    input  wire         op_reset,       // 操作复位信号，高电平有效

    // SPI物理接口信号
    input  wire         cs_n,           // SPI片选信号，低电平有效
    input  wire         sck,            // SPI时钟信号
    input  wire         si,             // SPI串行输入信号
    output wire         so,             // SPI串行输出信号（三态）

    // 与协议层的接口 - 请求/应答模式
    input  wire [7:0]   tx_data,        // 协议层提供的发送数据
    input  wire         tx_enable,      // 发送使能信号（协议层控制）
    input  wire         phy_tx_req,     // 发送请求脉冲（协议层->物理层）
    output reg          phy_tx_ack,     // 发送应答脉冲（物理层->协议层）

    output reg  [7:0]   rx_data,        // 接收到的数据
    output wire         rx_valid        // 接收数据有效脉冲（单周期）
);

//==============================================================================
// 内部信号定义
//==============================================================================
reg  [7:0]              rx_shift_reg;      // 接收移位寄存器
reg  [7:0]              tx_shift_reg;      // 发送移位寄存器
reg  [2:0]              bit_counter;       // 比特计数器 (0-7)
reg                     tx_in_progress;    // 发送进行中标志
reg                     tx_byte_complete;  // 字节发送完成中间信号（用于延迟phy_tx_ack）
reg                     tx_byte_complete_d1; // 延迟1个时钟周期
reg                     tx_byte_complete_d2; // 延迟2个时钟周期
// so_reg 已删除 - 现在直接由 tx_shift_reg[7] 驱动 so 输出

// SCK同步和边沿检测
reg  [2:0]              sck_sync;          // SCK同步寄存器
wire                    sck_rising_edge;   // SCK上升沿
wire                    sck_falling_edge;  // SCK下降沿

// phy_tx_req同步和边沿检测
reg                     phy_tx_req_prev;
wire                    phy_tx_req_rising_edge;

// CS同步和边沿检测
reg  [2:0]              cs_sync;           // CS同步寄存器
wire                    cs_active;         // CS有效信号
wire                    cs_falling_edge;   // CS下降沿

// 接收完成检测
reg                     byte_received;     // 字节接收完成标志
reg                     rx_valid_reg;      // 接收有效寄存器

//==============================================================================
// SCK和CS信号同步处理
//==============================================================================
// 同步SCK到系统时钟域
always @(posedge clk or posedge op_reset) begin
    if (op_reset) begin
        sck_sync <= 3'b000;
    end
    else begin
        sck_sync <= {sck_sync[1:0], sck};
    end
end

// 同步CS到系统时钟域
always @(posedge clk or posedge op_reset) begin
    if (op_reset) begin
        cs_sync <= 3'b111;
    end
    else begin
        cs_sync <= {cs_sync[1:0], cs_n};
    end
end

//==============================================================================
// 边沿检测和状态信号
//==============================================================================
assign sck_rising_edge  = (sck_sync[2:1] == 2'b01);
assign sck_falling_edge = (sck_sync[2:1] == 2'b10);
assign cs_active        = !cs_sync[1];
assign cs_falling_edge  = (cs_sync[2:1] == 2'b10);

//==============================================================================
// 数据接收处理逻辑
//==============================================================================
// 比特计数器管理 - 纯粹的SCK时钟计数器
always @(posedge clk or posedge op_reset) begin
    if (op_reset) begin
        bit_counter <= 3'b000;
    end
    else if (!cs_active) begin
        bit_counter <= 3'b000;
    end
    // 仅在SPI时钟上升沿时递增计数器
    else if (cs_active && sck_rising_edge) begin
        bit_counter <= bit_counter + 1'b1;
    end
end

// 接收移位寄存器
always @(posedge clk or posedge op_reset) begin
    if (op_reset) begin
        rx_shift_reg <= 8'h00;
    end
    else if (!cs_active) begin
        rx_shift_reg <= 8'h00;
    end
    else if (sck_rising_edge) begin
        rx_shift_reg <= {rx_shift_reg[6:0], si};
    end
end

// 字节接收完成检测
always @(posedge clk or posedge op_reset) begin
    if (op_reset) begin
        byte_received <= 1'b0;
    end
    else begin
        byte_received <= cs_active && sck_rising_edge && (bit_counter == 3'b111);
    end
end

// 接收数据输出
always @(posedge clk or posedge op_reset) begin
    if (op_reset) begin
        rx_data <= 8'h00;
    end
    else if (cs_active && sck_rising_edge && (bit_counter == 3'b111)) begin
        rx_data <= {rx_shift_reg[6:0], si};
    end
    else if (cs_falling_edge) begin
        rx_data <= 8'h00;
    end
end

// 接收有效信号生成
always @(posedge clk or posedge op_reset) begin
    if (op_reset) begin
        rx_valid_reg <= 1'b0;
    end
    else begin
        rx_valid_reg <= cs_active && sck_rising_edge && (bit_counter == 3'b111);
    end
end

assign rx_valid = rx_valid_reg;

//==============================================================================
// 数据发送处理逻辑 - 请求/应答模式
//==============================================================================
// 发送移位寄存器逻辑 - 从tx_data加载数据并执行移位操作
always @(posedge clk or posedge op_reset) begin
    if (op_reset) begin
        tx_shift_reg <= 8'h00;
    end
    else if (!cs_active) begin
        tx_shift_reg <= 8'h00;
    end
    else if (cs_active && tx_enable && phy_tx_req_rising_edge) begin
        tx_shift_reg <= tx_data;
    end
    // 在sck下降沿执行移位操作（从第二个bit开始，即bit_counter > 0时）
    else if (cs_active && sck_falling_edge && tx_in_progress && (bit_counter != 3'b000)) begin
        tx_shift_reg <= tx_shift_reg << 1;
    end
end

// 发送进行中标志 - 规范化单字节发送控制
always @(posedge clk or posedge op_reset) begin
    if (op_reset) begin
        tx_in_progress <= 1'b0;
    end
    else if (!cs_active) begin
        tx_in_progress <= 1'b0;
    end
    // 在phy_tx_req上升沿开始发送
    else if (cs_active && tx_enable && phy_tx_req_rising_edge) begin
        tx_in_progress <= 1'b1;
    end
    // 在该字节的第8位发送完成时复位（与phy_tx_ack产生条件相同）
    else if (sck_falling_edge && tx_in_progress && (bit_counter == 3'b111)) begin
        tx_in_progress <= 1'b0;
    end
end

// 字节发送完成中间信号生成 - 第一级流水线
always @(posedge clk or posedge op_reset) begin
    if (op_reset) begin
        tx_byte_complete <= 1'b0;
    end
    else if (!cs_active) begin
        tx_byte_complete <= 1'b0;
    end
    // 在发送完8位后产生中间完成信号
    else if (sck_falling_edge && tx_in_progress && (bit_counter == 3'b111)) begin
        tx_byte_complete <= 1'b1;
    end
    else begin
        tx_byte_complete <= 1'b0;
    end
end

// 延迟流水线寄存器 - 第二级和第三级流水线
always @(posedge clk or posedge op_reset) begin
    if (op_reset) begin
        tx_byte_complete_d1 <= 1'b0;
        tx_byte_complete_d2 <= 1'b0;
    end
    else if (!cs_active) begin
        tx_byte_complete_d1 <= 1'b0;
        tx_byte_complete_d2 <= 1'b0;
    end
    else begin
        tx_byte_complete_d1 <= tx_byte_complete;
        tx_byte_complete_d2 <= tx_byte_complete_d1;
    end
end

// 发送应答信号生成 - 第四级流水线，总共延迟3个时钟周期
always @(posedge clk or posedge op_reset) begin
    if (op_reset) begin
        phy_tx_ack <= 1'b0;
    end
    else if (!cs_active) begin
        phy_tx_ack <= 1'b0;
    end
    // 延迟3个时钟周期的应答信号
    else if (tx_byte_complete_d2) begin
        phy_tx_ack <= 1'b1;
    end
    else begin
        phy_tx_ack <= 1'b0;
    end
end

//==============================================================================
// 输出信号分配
//==============================================================================
// SO三态输出控制 - 直接由tx_shift_reg[7]驱动，由tx_enable控制三态
assign so = (tx_enable && cs_active) ? tx_shift_reg[7] : 1'bZ;

// phy_tx_req 边沿检测逻辑
always @(posedge clk or posedge op_reset) begin
    if (op_reset) begin
        phy_tx_req_prev <= 1'b0;
    end
    else begin
        phy_tx_req_prev <= phy_tx_req;
    end
end
assign phy_tx_req_rising_edge = phy_tx_req & ~phy_tx_req_prev;

endmodule

