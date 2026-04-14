# RK3588 IMX415 Porting Archive

这个仓库用于整理 RK3588 平台下与 IMX415 传感器移植相关的资料、驱动代码、DTS 文件和参考文档，方便后续持续维护、回顾和二次开发。

## Repository Layout

```text
docs/
  sensor/IMX415/       IMX415 相关手册与规格书
  isp/XC7160/          XC7160 ISP 参考资料
  soc/                 RK3588 / RV1126 / AR9341 等 SoC 文档
driver/
  imx415/              IMX415 驱动、Kconfig、Makefile、defconfig
dts/
  rk3588/              RK3588 平台 DTS / DTSI / 内核相关参考代码
notes/
  PORTING_NOTES.md     移植记录、问题和验证结论
```

## Current Contents

- `driver/imx415/`
  - `imx415带注释文件.c`
  - `Kconfig`
  - `Makefile`
  - `rockchip_defconfig`
- `dts/rk3588/`
  - Rockchip 平台相关 DTS / DTSI 文件
  - camera / cif / i2c / isp / gpio 等目录
- `docs/`
  - IMX415、XC7160、RK3588 以及其他参考手册
