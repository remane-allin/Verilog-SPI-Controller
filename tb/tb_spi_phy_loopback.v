//==============================================================================
// File Name    : tb_spi_phy_loopback.v
// Description  : SPI主从物理层回路测试平台 - 最终版本
//                验证SPI Mode 0协议的主从通信功能
// Author       :
// Date         :
// Version      : Final
//==============================================================================
//
// 测试验证:
// --------
// 1. 主机写->从机读: 验证主机发送数据，从机正确接收
// 2. 主机读<-从机写: 验证从机发送架构，主机接收能力
// 3. 全双工通信: 验证主从机同时收发数据架构
//
//==============================================================================

`timescale 1ns / 1ps

module tb_spi_phy_loopback;

//==============================================================================
// 参数定义
//==============================================================================
parameter CLK_PERIOD = 20;      // 50MHz系统时钟，20ns周期
parameter SIM_TIME = 1000000;   // 仿真时间1ms

//==============================================================================
// 信号定义
//==============================================================================
// 系统信号
reg                 clk;
reg                 rst_n;

// SPI总线信号
wire                cs_n;
wire                sck;
wire                si;     // MOSI: Master Out Slave In
wire                so;     // MISO: Master In Slave Out

// 主机接口信号
reg                 start_transfer;
reg  [7:0]          master_tx_data;
wire [7:0]          master_rx_data;
wire                transfer_done;

// 从机接口信号 (模拟协议层)
reg  [7:0]          slave_tx_data;
reg                 slave_tx_enable;
reg                 slave_phy_tx_req;
wire                slave_phy_tx_ack;
wire [7:0]          slave_rx_data;
wire                slave_rx_valid;

// 测试控制信号
integer             test_count;
integer             pass_count;
integer             fail_count;

//==============================================================================
// 时钟生成
//==============================================================================
initial begin
    clk = 0;
    forever #(CLK_PERIOD/2) clk = ~clk;
end

//==============================================================================
// 复位生成
//==============================================================================
initial begin
    rst_n = 0;
    #(CLK_PERIOD * 10);
    rst_n = 1;
    $display("=== SPI Physical Layer Loopback Test Started ===");
end

//==============================================================================
// 主机模块例化
//==============================================================================
spi_master u_master (
    .clk            (clk),
    .rst_n          (rst_n),
    .cs_n           (cs_n),
    .sck            (sck),
    .si             (si),           // MOSI
    .so             (so),           // MISO
    .start_transfer (start_transfer),
    .data_to_send   (master_tx_data),
    .data_received  (master_rx_data),
    .transfer_done  (transfer_done)
);

//==============================================================================
// 从机模块例化
//==============================================================================
spi_slave u_slave (
    .clk            (clk),
    .op_reset       (~rst_n),       // 从机使用高电平复位
    .cs_n           (cs_n),
    .sck            (sck),
    .si             (si),           // MOSI
    .so             (so),           // MISO
    .tx_data        (slave_tx_data),
    .tx_enable      (slave_tx_enable),
    .phy_tx_req     (slave_phy_tx_req),
    .phy_tx_ack     (slave_phy_tx_ack),
    .rx_data        (slave_rx_data),
    .rx_valid       (slave_rx_valid)
);

//==============================================================================
// 测试任务定义
//==============================================================================

// 任务1: 主机写->从机读测试
task test_master_write_slave_read;
    input [7:0] test_data;
    begin
        master_tx_data = test_data;
        slave_tx_enable = 1'b0;
        slave_phy_tx_req = 1'b0;

        start_transfer = 1'b1;
        #(CLK_PERIOD);
        start_transfer = 1'b0;

        #(CLK_PERIOD * 500);  // 等待传输完成

        $display("Test 1 - Master TX: 0x%02X, Slave RX: 0x%02X", test_data, slave_rx_data);
        if (slave_rx_data == test_data) begin
            $display("PASS: Master Write, Slave Read");
            pass_count = pass_count + 1;
        end else begin
            $display("FAIL: Master Write, Slave Read");
            fail_count = fail_count + 1;
        end

        test_count = test_count + 1;
        #(CLK_PERIOD * 100);
    end
endtask

// 任务2: 主机读<-从机写测试 - 简化版本
task test_master_read_slave_write;
    input [7:0] test_data;
    begin
        $display("--- Test 2: Master Read <- Slave Write ---");
        $display("Slave TX Data: 0x%02X", test_data);

        // 准备从机发送数据
        slave_tx_data = test_data;
        slave_tx_enable = 1'b1;

        // 确保上一次传输已结束并等待一段空闲
        wait(cs_n == 1'b1);
        #(CLK_PERIOD * 5);

        // 启动传输，同时在传输过程中发送从机请求
        start_transfer = 1'b1;
        #(CLK_PERIOD * 2);
        start_transfer = 1'b0;

        // 等待CS拉低并避开内部CS同步延迟（≥2个clk），增加超时保护
        begin : WAIT_CS_LOW_T2
            integer cs_wait_cnt;
            cs_wait_cnt = 0;
            while (cs_n != 1'b0 && cs_wait_cnt < 200) begin
                #(CLK_PERIOD);
                cs_wait_cnt = cs_wait_cnt + 1;
            end
        end
        #(CLK_PERIOD * 3);
        slave_phy_tx_req = 1'b1;
        #(CLK_PERIOD * 2);
        slave_phy_tx_req = 1'b0;

        // 等待传输完成（固定时间窗口，避免丢失单拍脉冲）
        #(CLK_PERIOD * 600);

        // 验证结果
        $display("Master RX Data: 0x%02X", master_rx_data);
        if (master_rx_data == test_data) begin
            $display("PASS: Master Read, Slave Write");
            pass_count = pass_count + 1;
        end else begin
            $display("FAIL: Master Read, Slave Write - Expected: 0x%02X, Got: 0x%02X",
                     test_data, master_rx_data);
            fail_count = fail_count + 1;
        end

        test_count = test_count + 1;
        slave_tx_enable = 1'b0;  // 关闭从机发送
        #(CLK_PERIOD * 100);
    end
endtask

// 任务3: 全双工通信测试
task test_full_duplex_transfer;
    input [7:0] master_data;
    input [7:0] slave_data;
    begin
        $display("--- Test 3: Full-Duplex Transfer ---");
        $display("Master TX Data: 0x%02X, Slave TX Data: 0x%02X", master_data, slave_data);

        // 准备主机发送数据
        master_tx_data = master_data;

        // 准备从机发送数据
        slave_tx_data = slave_data;
        slave_tx_enable = 1'b1;

        // 确保上一次传输已结束并等待一段空闲
        wait(cs_n == 1'b1);
        #(CLK_PERIOD * 5);

        // 启动传输，同时在传输过程中发送从机请求
        start_transfer = 1'b1;
        #(CLK_PERIOD * 2);
        start_transfer = 1'b0;

        // 等待CS拉低并避开内部CS同步延迟（≥2个clk），增加超时保护
        begin : WAIT_CS_LOW_T3
            integer cs_wait_cnt3;
            cs_wait_cnt3 = 0;
            while (cs_n != 1'b0 && cs_wait_cnt3 < 200) begin
                #(CLK_PERIOD);
                cs_wait_cnt3 = cs_wait_cnt3 + 1;
            end
        end
        #(CLK_PERIOD * 3);
        slave_phy_tx_req = 1'b1;
        #(CLK_PERIOD * 2);
        slave_phy_tx_req = 1'b0;

        // 这部分已经在上面处理了，删除重复代码

        // 等待传输完成（固定时间窗口，避免丢失单拍脉冲）
        #(CLK_PERIOD * 600);

        // 验证结果
        $display("Master RX Data: 0x%02X, Slave RX Data: 0x%02X", master_rx_data, slave_rx_data);

        if ((master_rx_data == slave_data) && (slave_rx_data == master_data)) begin
            $display("PASS: Full-Duplex Transfer");
            pass_count = pass_count + 1;
        end else begin
            $display("FAIL: Full-Duplex Transfer");
            $display("  Master Expected: 0x%02X, Got: 0x%02X", slave_data, master_rx_data);
            $display("  Slave Expected: 0x%02X, Got: 0x%02X", master_data, slave_rx_data);
            fail_count = fail_count + 1;
        end

        test_count = test_count + 1;
        slave_tx_enable = 1'b0;  // 关闭从机发送
        #(CLK_PERIOD * 100);
    end
endtask

//==============================================================================
// 主测试流程
//==============================================================================
initial begin
    // 初始化信号
    start_transfer = 1'b0;
    master_tx_data = 8'h00;
    slave_tx_data = 8'h00;
    slave_tx_enable = 1'b0;
    slave_phy_tx_req = 1'b0;

    test_count = 0;
    pass_count = 0;
    fail_count = 0;

    // 等待复位完成
    wait(rst_n);
    #(CLK_PERIOD * 20);

    // 测试1: 主机写->从机读
    test_master_write_slave_read(8'hA5);
    test_master_write_slave_read(8'h5A);

    // 测试2: 主机读<-从机写（使用任务，包含请求握手与结果检查）
    test_master_read_slave_write(8'hB6);

    // 测试3: 全双工通信（使用任务，包含请求握手与结果检查）
    test_full_duplex_transfer(8'hA5, 8'hB6);

    // 测试结果汇总
    #(CLK_PERIOD * 100);
    $display("=== Test Summary ===");
    $display("Total Tests: %0d", test_count);
    $display("Passed: %0d", pass_count);
    $display("Failed: %0d", fail_count);

    if (fail_count == 0) begin
        $display("=== ALL TESTS PASSED ===");
    end else begin
        $display("=== SOME TESTS FAILED ===");
    end

    $finish;
end

//==============================================================================
// 仿真时间限制
//==============================================================================
initial begin
    #SIM_TIME;
    $display("=== Simulation Timeout ===");
    $finish;
end

//==============================================================================
// 波形文件生成
//==============================================================================
initial begin
    $dumpfile("tb_spi_phy_loopback.vcd");
    $dumpvars(0, tb_spi_phy_loopback);
end

endmodule
