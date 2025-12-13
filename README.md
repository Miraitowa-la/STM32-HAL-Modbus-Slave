# STM32-HAL-Modbus-Slave

A lightweight Modbus RTU slave protocol stack based on STM32 HAL library. Supports automatic RS485/TTL switching, Flash-based configuration storage, and is compatible with all STM32 series.

## Version Selection

This project provides two major versions. Choose the one that best fits your needs:

| Version | Status | Description |
|---------|--------|-------------|
| [**V1.X**](V1_0/README.md) | Stable | Classic architecture, production-ready, single instance |
| [**V2.X**](V2_0/README.md) | Stable | Multi-instance support, runtime configuration, data source decoupling |

### V1.X - Stable Version

A mature and stable Modbus RTU slave implementation with the following features:

- Standard Modbus function codes (0x01-0x06, 0x0F, 0x10)
- Custom function code 0x64 for online configuration
- Flash-based non-volatile storage for slave address and baud rate
- RS485/RS232/TTL interface support
- Broadcast recovery mode (address 0xFF)
- Ping-pong buffer mechanism for reliable communication
- Dynamic transmission timeout calculation
- CRC16 algorithm selection (bit-shift or lookup table)
- Blocking or DMA transmission mode

**Documentation**: [English](V1_0/README.md) | [简体中文](V1_0/README-zh_CN.md)

### V2.X - Multi-Instance Version

A completely redesigned Modbus slave library with OOP-C style architecture:

- **Multi-Instance Support**: Run multiple independent Modbus slaves on different UARTs
- **Runtime Configuration**: All settings passed via initialization, no static macros
- **Data Source Decoupling**: Multiple instances can share one data source
- **Callback Mechanism**: Custom function code 0x64 via application callbacks
- All V1.X features preserved (ping-pong buffer, dynamic timeout, CRC options, DMA/blocking)

**Status**: Stable

**Documentation**: [English](V2_0/README.md) | [简体中文](V2_0/README-zh_CN.md)

## License

MIT License - Suitable for any commercial or personal project

## Contributing

Welcome to submit Issues and Pull Requests!

---

一个轻量级的 STM32 HAL 库 Modbus RTU 从机协议栈。支持 RS485/TTL 自动切换、Flash 配置存储，兼容 STM32 全系列。

## 版本选择

本项目提供两个主要版本，请根据需求选择：

| 版本 | 状态 | 说明 |
|------|------|------|
| [**V1.X**](V1_0/README-zh_CN.md) | 稳定版 | 经典架构，可用于生产环境，单实例 |
| [**V2.X**](V2_0/README-zh_CN.md) | 稳定版 | 多实例支持，运行时配置，数据源解耦 |

### V1.X - 稳定版本

成熟稳定的 Modbus RTU 从机实现，具有以下特性：

- 标准 Modbus 功能码支持 (0x01-0x06, 0x0F, 0x10)
- 自定义功能码 0x64 支持在线配置修改
- Flash 掉电保存从机地址和波特率
- RS485/RS232/TTL 接口支持
- 广播救砖模式 (地址 0xFF)
- 乒乓缓冲机制保证通信可靠性
- 动态发送超时计算
- CRC16 算法可选（移位法或查表法）
- 阻塞或 DMA 发送模式

**文档**: [English](V1_0/README.md) | [简体中文](V1_0/README-zh_CN.md)

### V2.X - 多实例版本

采用 OOP-C 风格全新架构设计的 Modbus 从机库：

- **多实例支持**: 可在不同 UART 上同时运行多个独立的 Modbus 从站
- **运行时配置**: 所有设置通过初始化传入，无需静态宏定义
- **数据源解耦**: 多个实例可共享同一份数据源
- **回调机制**: 自定义功能码 0x64 通过应用层回调实现
- 保留 V1.X 所有特性（乒乓缓冲、动态超时、CRC可选、DMA/阻塞）

**状态**: 实验版

**文档**: [English](V2_0/README.md) | [简体中文](V2_0/README-zh_CN.md)

## 许可证

MIT License - 适用于任何商业或个人项目

## 贡献

欢迎提交 Issue 和 Pull Request！

---

**⭐ 如果这个项目对你有帮助，请给一个 Star 支持一下！**
