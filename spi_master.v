//==============================================================================
// File Name    : spi_master.v
// Description  : SPI主机物理接口模块 - 基于tb_spi_arinc429.v的spi_write_read_byte任务
//                实现SPI Mode 0 (CPOL=0, CPHA=0) 协议时序
//                采用状态机控制的可综合设计
// Author       :
// Date         :
// Version      : 1.0
//==============================================================================
//
// 功能说明:
// --------
// 本模块实现了SPI主机物理层接口，专注于比特流转换功能。
// 采用状态机控制SPI传输时序，确保严格遵循SPI Mode 0协议。
//
// 核心设计理念:
// --------
// 1. 纯粹的比特流转换器:
//    - 物理层只负责串行/并行转换
//    - 不参与复杂的协议判断和控制逻辑
//    - 专注于可靠的时序处理
//
// 2. 状态机控制模式:
//    - 应用层通过start_transfer启动传输
//    - 物理层通过transfer_done通知传输完成
//    - 清晰的握手时序，避免竞争冒险
//
// 3. 时钟生成保证:
//    - 内部生成SPI时钟，确保时序精确
//    - 支持可配置的时钟分频
//    - 严格的CS和SCK时序关系
//
// 主要特性:
// --------
// 1. 协议兼容性:
//    - 严格遵循SPI Mode 0时序 (CPOL=0, CPHA=0)
//    - 数据在SCK上升沿采样，下降沿更新
//    - 支持标准的CS片选控制
//
// 2. 数据发送功能:
//    - 8位并行到串行转换
//    - MSB先行传输模式
//    - 可靠的输出驱动机制
//
// 3. 数据接收功能:
//    - 8位串行到并行转换
//    - 自动字节边界检测
//    - 传输完成时数据有效
//
// 4. 同步设计:
//    - 所有逻辑同步到系统时钟
//    - 内部时钟分频生成SPI时钟
//    - 状态机控制传输流程
//
// 接口说明:
// --------
// 系统接口:
//   clk            - 系统时钟
//   rst_n          - 系统复位(低电平有效)
//
// SPI物理接口:
//   cs_n           - 片选信号(低电平有效)
//   sck            - SPI时钟
//   si (MOSI)      - 主机输出，从机输入
//   so (MISO)      - 主机输入，从机输出
//
// 应用层接口:
//   start_transfer - 启动传输脉冲
//   data_to_send   - 发送数据(8位)
//   data_received  - 接收数据(8位)
//   transfer_done  - 传输完成脉冲
//
// 工作时序:
// --------
// 1. 传输启动:
//    应用层准备data_to_send -> 发出start_transfer脉冲 -> 
//    主机开始传输 -> CS拉低，开始SCK时钟
//
// 2. 数据传输:
//    8个SCK周期 -> 每个下降沿更新MOSI -> 每个上升沿采样MISO ->
//    传输完成后CS拉高 -> 产生transfer_done脉冲
//
// 3. 复位时序:
//    rst_n低电平 -> 所有内部状态复位 -> CS和SCK保持高电平
//
//==============================================================================

module spi_master (
    // 系统时钟和复位
    input  wire         clk,                // 系统时钟
    input  wire         rst_n,              // 系统复位信号，低电平有效

    // SPI物理接口信号
    output reg          cs_n,               // SPI片选信号，低电平有效
    output reg          sck,                // SPI时钟信号
    output reg          si,                 // SPI串行输出信号 (MOSI)
    input  wire         so,                 // SPI串行输入信号 (MISO)

    // 与应用层的接口
    input  wire         start_transfer,     // 启动传输脉冲
    input  wire [7:0]   data_to_send,       // 发送数据
    output reg  [7:0]   data_received,      // 接收到的数据
    output reg          transfer_done       // 传输完成脉冲
);

//==============================================================================
// 参数定义
//==============================================================================
// SPI时钟分频参数 - 基于testbench中的SPI_CLK_PERIOD = 200ns
// 假设系统时钟为50MHz (20ns周期)，需要分频10得到100ns半周期
parameter CLK_DIV = 5;  // 时钟分频系数，产生100ns半周期

// 状态机状态定义
localparam [2:0] ST_IDLE     = 3'b000,  // 空闲状态
                 ST_START    = 3'b001,  // 开始传输
                 ST_TRANSFER = 3'b010,  // 数据传输
                 ST_STOP     = 3'b011;  // 结束传输

//==============================================================================
// 内部信号定义
//==============================================================================
reg  [2:0]              state, next_state;     // 状态机
reg  [7:0]              tx_shift_reg;          // 发送移位寄存器
reg  [7:0]              rx_shift_reg;          // 接收移位寄存器
reg  [2:0]              bit_counter;           // 比特计数器 (0-7)
reg  [3:0]              clk_counter;           // 时钟分频计数器
reg                     sck_enable;            // SCK使能信号
reg                     sck_edge;              // SCK边沿指示
reg                     start_transfer_prev;   // start_transfer边沿检测
wire                    start_transfer_edge;   // start_transfer上升沿

//==============================================================================
// start_transfer边沿检测
//==============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        start_transfer_prev <= 1'b0;
    end
    else begin
        start_transfer_prev <= start_transfer;
    end
end

assign start_transfer_edge = start_transfer & ~start_transfer_prev;

//==============================================================================
// 时钟分频逻辑 - 生成SPI时钟
//==============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        clk_counter <= 4'b0000;
        sck_edge <= 1'b0;
    end
    else if (sck_enable) begin
        if (clk_counter == CLK_DIV - 1) begin
            clk_counter <= 4'b0000;
            sck_edge <= 1'b1;
        end
        else begin
            clk_counter <= clk_counter + 1'b1;
            sck_edge <= 1'b0;
        end
    end
    else begin
        clk_counter <= 4'b0000;
        sck_edge <= 1'b0;
    end
end

//==============================================================================
// 状态机 - 当前状态寄存器
//==============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= ST_IDLE;
    end
    else begin
        state <= next_state;
    end
end

//==============================================================================
// 状态机 - 下一状态逻辑
//==============================================================================
always @(*) begin
    next_state = state;

    case (state)
        ST_IDLE: begin
            if (start_transfer_edge) begin
                next_state = ST_START;
            end
        end

        ST_START: begin
            next_state = ST_TRANSFER;
        end

        ST_TRANSFER: begin
            if (sck_edge && sck && (bit_counter == 3'b111)) begin
                next_state = ST_STOP;
            end
        end

        ST_STOP: begin
            next_state = ST_IDLE;
        end

        default: begin
            next_state = ST_IDLE;
        end
    endcase
end

//==============================================================================
// 状态机 - 输出逻辑
//==============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        cs_n <= 1'b1;
        sck <= 1'b0;
        si <= 1'b0;
        sck_enable <= 1'b0;
        bit_counter <= 3'b000;
        tx_shift_reg <= 8'h00;
        rx_shift_reg <= 8'h00;
        data_received <= 8'h00;
        transfer_done <= 1'b0;
    end
    else begin
        // 默认值
        transfer_done <= 1'b0;

        case (state)
            ST_IDLE: begin
                cs_n <= 1'b1;
                sck <= 1'b0;
                si <= 1'b0;
                sck_enable <= 1'b0;
                bit_counter <= 3'b000;

                // 在启动传输时锁存发送数据
                if (start_transfer_edge) begin
                    tx_shift_reg <= data_to_send;
                end
            end

            ST_START: begin
                cs_n <= 1'b0;  // 拉低CS开始传输
                sck <= 1'b0;
                sck_enable <= 1'b1;
                si <= tx_shift_reg[7];  // 输出第一位数据
                bit_counter <= 3'b000;
            end

            ST_TRANSFER: begin
                cs_n <= 1'b0;
                sck_enable <= 1'b1;

                if (sck_edge) begin
                    sck <= ~sck;  // 翻转SCK

                    if (!sck) begin
                        // SCK上升沿：采样输入数据
                        rx_shift_reg <= {rx_shift_reg[6:0], so};
                    end
                    else begin
                        // SCK下降沿：更新输出数据，移位发送寄存器
                        bit_counter <= bit_counter + 1'b1;
                        if (bit_counter < 3'b111) begin
                            tx_shift_reg <= tx_shift_reg << 1;
                            si <= tx_shift_reg[6];  // 下一位数据
                        end
                    end
                end
            end

            ST_STOP: begin
                cs_n <= 1'b1;  // 拉高CS结束传输
                sck <= 1'b0;
                si <= 1'b0;
                sck_enable <= 1'b0;

                // 传输完成，输出接收到的数据
                data_received <= rx_shift_reg;
                transfer_done <= 1'b1;
            end

            default: begin
                cs_n <= 1'b1;
                sck <= 1'b0;
                si <= 1'b0;
                sck_enable <= 1'b0;
            end
        endcase
    end
end

endmodule
