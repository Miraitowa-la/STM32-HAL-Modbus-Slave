# STM32-HAL-Modbus-Slave

A lightweight Modbus RTU slave protocol stack based on STM32 HAL library. Supports automatic RS485/TTL switching, Flash-based configuration storage, and is compatible with all STM32 series.

## Version Selection

This project provides two major versions. Choose the one that best fits your needs:

| Version | Status | Description |
|---------|--------|-------------|
| [**V1.X**](V1_0/README.md) | Stable | Classic architecture, production-ready, actively maintained |
| [**V2.X**](V2_0/README.md) | Development | New architecture, enhanced extensibility (Coming Soon) |

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

### V2.X - Development Version

A completely redesigned Modbus slave library with a new architecture for better extensibility and more advanced features.

**Status**: Under Development

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
| [**V1.X**](V1_0/README-zh_CN.md) | 稳定版 | 经典架构，可用于生产环境，持续维护中 |
| [**V2.X**](V2_0/README-zh_CN.md) | 开发中 | 全新架构，增强可扩展性（敬请期待） |

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

### V2.X - 开发版本

全新架构设计的 Modbus 从机库，提供更好的可扩展性和更多高级功能。

**状态**: 开发中

**文档**: [English](V2_0/README.md) | [简体中文](V2_0/README-zh_CN.md)

## 许可证

MIT License - 适用于任何商业或个人项目

## 贡献

欢迎提交 Issue 和 Pull Request！

---

**⭐ 如果这个项目对你有帮助，请给一个 Star 支持一下！**
