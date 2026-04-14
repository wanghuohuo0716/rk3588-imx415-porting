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

## Recommended Workflow

1. 把你真正修改过的驱动和 DTS 文件继续集中维护在当前目录结构下。
2. 每次移植或调试后，把关键结论记录到 `notes/PORTING_NOTES.md`。
3. 如果后面增加编译脚本、补丁或测试记录，建议新增以下目录：
   - `patches/`
   - `scripts/`
   - `logs/`
4. 如果仓库准备上传到 GitHub，建议优先使用私有仓库。

## Suggested GitHub Repo Type

- Visibility: `Private`
- Name example: `rk3588-imx415-porting`
- Main purpose:
  - 保存移植过程中的源码修改
  - 保存关键参考资料
  - 记录编译、烧录和验证步骤

## Suggested First Upload Steps

在本机安装并配置 Git 后，可以按下面的顺序操作：

```powershell
git init
git add .
git commit -m "Initial import: RK3588 IMX415 porting archive"
git branch -M main
git remote add origin <your-github-repo-url>
git push -u origin main
```

## Notes

- 当前仓库中包含 PDF 和网页保存资料，上传到 GitHub 前请再次确认是否适合共享。
- 如果后续加入超过 100MB 的大文件，建议使用 Git LFS。
- 如果某些资料仅用于参考而不需要版本管理，可以后续移出仓库，避免仓库变得过大。
