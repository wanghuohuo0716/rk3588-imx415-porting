// SPDX-License-Identifier: GPL-2.0
// SPDX 是标准化的许可证标识格式，便于自动化工具识别许可证类型。
/*
 * imx415 driver
 *
 * Copyright (C) 2020 Rockchip Electronics Co., Ltd.
 * 版权归瑞芯微电子（Rockchip）所有，开发年份为 2020 年
 *
 * V0.0X01.0X00 first version. 版本控制，版本号中的 "X" 是十六进制
 * V0.0X01.0X01 版本号
 *  1. fix hdr ae ratio error,修复 HDR 自动曝光（AE）比例错误
 *     0x3260 should be set 0x01 in normal mode, 寄存器 0x3260 控制 HDR 模式开关，普通模式：必须设为 0x01
 *     should be 0x00 in hdr mode.HDR 模式：必须设为 0x00（早期版本设反导致功能异常）
 *  2. rhs1 should be 4n+1 when set hdr ae.修复 RHS1 时序约束：RHS1 是短曝光结束行位置，必须满足 4n+1 约束（如 25, 29, 33...），否则传感器时序混乱

 * V0.0X01.0X02 版本号
 *  1. shr0 should be greater than (rsh1 + 9).修复长曝光起始行（SHR0）约束：必须满足 SHR0 ≥ RHS1 + 9，确保长短曝光时序不重叠
 *  2. rhs1 should be ceil to 4n + 1.优化 RHS1 计算：从简单取整改为向上取整（ceil）到 4n+1，避免时序违规

 * V0.0X01.0X03 版本号
 *  1. support 12bit HDR DOL3 支持 12-bit 位深的 HDR DOL3 模式：DOL3 = 三曝光合成（Long + Medium + Short）
 *  2. support HDR/Linear quick switch 支持 HDR 与 Linear 模式快速切换：无需重启传感器，通过寄存器动态切换工作模式

 * V0.0X01.0X04 版本号
 * 1. support enum format info by aiq 支持 AIQ（Auto Image Quality）框架枚举格式信息，便于上层 ISP 算法自动获取传感器能力（分辨率/帧率/位深等）

 * V0.0X01.0X05 版本号
 * 1. fixed 10bit hdr2/hdr3 frame rate issue 修复 10-bit HDR2/HDR3 模式下帧率计算错误

 * V0.0X01.0X06 版本号
 * 1. support DOL3 10bit 20fps 1485Mbps 新增 10-bit DOL3 20fps 模式，MIPI 速率 1485Mbps
 * 2. fixed linkfreq error 修复 link_freq（链路频率）控制错误，确保 V4L2 框架正确上报带宽参数

 * V0.0X01.0X07 版本号
 * 1. fix set_fmt & ioctl get mode unmatched issue. 修复 set_fmt（设置格式）与 ioctl 获取模式不一致问题
 * 2. need to set default vblank when change format. 切换分辨率时自动设置默认 VBLANK（垂直消隐），避免帧率异常
 * 3. enum all supported mode mbus_code, not just cur_mode. 枚举所有支持的媒体总线格式（mbus_code），而非仅当前模式

 
 * V0.0X01.0X08 版本号
 * 1. add dcphy param for hdrx2 mode. 为 HDR X2 模式添加 DC-PHY 参数配置
 */

#define DEBUG // 启用内核调试模式，启用后，驱动中 dev_dbg() 等调试日志会输出到内核日志（dmesg 可查看）
#include <linux/clk.h> // 时钟控制：管理传感器外部时钟（xvclk，通常 27MHz/37.125MHz）
#include <linux/device.h> // Linux 设备驱动核心框架，用于设备注册/注销
#include <linux/delay.h> // 提供 usleep_range()/msleep() 等，用于满足传感器上电/复位时序要求
#include <linux/gpio/consumer.h> // 操作复位引脚（reset GPIO）
#include <linux/i2c.h> // 通过 I2C 总线读写传感器寄存器
#include <linux/module.h> // 所有 Linux 驱动必备，提供 module_init()/module_exit() 宏
#include <linux/pm_runtime.h> // 支持运行时自动休眠/唤醒（如摄像头不用时自动断电省电）
#include <linux/regulator/consumer.h> // 电源控制：管理三路供电：
                                                            //dvdd：数字电路电源（1.2V）
                                                            //avdd：模拟电路电源（2.8V）
                                                            //dovdd：I/O 接口电源（1.8V）

#include <linux/sysfs.h> // 在 /sys 目录下创建可读写文件，方便用户空间调试
#include <linux/slab.h> // 提供 kmalloc()/kfree()，用于动态申请驱动私有数据结构内存
#include <linux/version.h> // 用于条件编译，适配不同版本内核的 API 差异
#include <linux/rk-camera-module.h> // Rockchip 专有：瑞芯微摄像头模块框架，提供统一的电源/复位管理接口

// V4L2 核心组件：
#include <media/media-entity.h> // 媒体控制器拓扑（描述 sensor→isp→display 数据流）
#include <media/v4l2-async.h> // 异步绑定（设备树中 sensor 与 isp 的自动匹配）
#include <media/v4l2-ctrls.h> // 控制接口（曝光/增益等参数）
#include <media/v4l2-subdev.h> // 子设备框架（传感器作为 ISP 的子设备）

#include <linux/pinctrl/consumer.h> // 配置 GPIO 引脚功能（如将某个引脚设为 I2C 功能而非普通 GPIO
#include <linux/rk-preisp.h> // PreISP 支持：Rockchip 特有功能，用于在 ISP 前做预处理（如 HDR 合成）

// 从设备树（DTS）中读取端口连接关系（如 sensor 连接到 mipi_csi0）
#include <media/v4l2-fwnode.h> // 提供了 将设备树中的端点（endpoint）信息解析为 V4L2 标准结构体 的函数。
#include <linux/of_graph.h> //  Linux 设备树图形（graph）解析 API，用于遍历设备树中的 port/endpoint 连接拓扑。

#include "../platform/rockchip/isp/rkisp_tb_helper.h" //Thunderboot 支持：瑞芯微快速启动技术，跳过部分初始化加速开机（用于安防/车载场景）

// 作用：定义驱动版本为 0.01.08（对应前面注释中的 V0.0X01.0X08）
// KERNEL_VERSION 宏：将版本号编码为 32 位整数（主版本<<16 | 次版本<<8 | 修订号）
// 用途：用户可通过 modinfo imx415.ko 查看驱动版本
#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x08) // 驱动版本号


// 问题背景：旧版内核（<4.19）没有 V4L2_CID_DIGITAL_GAIN 控制项
// 解决方案：如果不存在该定义，就用通用的 V4L2_CID_GAIN 代替
// 新手理解：相当于"如果新遥控器没这个按钮，就用旧遥控器的按钮代替"
#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

// MIPI CSI-2 接口的时钟频率
// 这些值单位是 Hz（赫兹），不是 Mbps。实际数据速率 = 频率 × 2（DDR 双倍数据率）
#define MIPI_FREQ_1188M			1188000000

// 常见于 4K@30fps 模式
#define MIPI_FREQ_891M			891000000

#define MIPI_FREQ_446M			446000000
#define MIPI_FREQ_743M			743000000
#define MIPI_FREQ_297M			297000000

// 数据通道数（MIPI Lane 数）
#define IMX415_4LANES			4
#define IMX415_2LANES			2

// 最大像素率（用于 V4L2 控制项）
// MIPI_FREQ_891M / 10 → 假设 10-bit 输出（每像素 10 bit）
// * 2 → DDR（双倍数据率）
// * 4 → 4 条 lane
// 结果：最大像素吞吐率 ≈ 712.8 MP/s
// 用途：上报给 V4L2 的 V4L2_CID_PIXEL_RATE 控制项上限
#define IMX415_MAX_PIXEL_RATE		(MIPI_FREQ_891M / 10 * 2 * IMX415_4LANES)

// 设备树属性名（从 DTS 中读取 HDR 模式）
// 在设备树（DTS）中这样写：
// rockchip,camera-hdr-mode = <2>; // 表示 HDR_X2
// 驱动通过 of_property_read_u32() 读取该值，决定默认启动哪种 HDR 模式
#define OF_CAMERA_HDR_MODE		"rockchip,camera-hdr-mode"

// 外部时钟频率，37.125 MHz，标准视频时钟（用于 4K/30fps 等高分辨率模式）
#define IMX415_XVCLK_FREQ_37M		37125000

// 27 MHz，用于 2-lane 或低功耗模式（如 1284×720@90fps）
#define IMX415_XVCLK_FREQ_27M		27000000

/* TODO: Get the real chip id from reg */
// Sony IMX415 的芯片 ID 应为 0xE0（但注释说“TODO”，说明可能未验证或需从寄存器读取）
// 寄存器地址 0x311A 存储芯片 ID，驱动会读此寄存器验证是否真的是 IMX415
#define CHIP_ID				0xE0
#define IMX415_REG_CHIP_ID		0x311A

// 控制模式寄存器（启动/停止视频流），控制传感器工作状态的寄存器
// 写 0x01 到 0x3000 → 进入 软件待机模式（停止输出）？？？？？???这里没有1？是streaming这里打错了吗？
// 写 0x00 到 0x3000 → 开始输出视频流
#define IMX415_REG_CTRL_MODE		0x3000
#define IMX415_MODE_SW_STANDBY		BIT(0)
#define IMX415_MODE_STREAMING		0x0

// 增益寄存器（控制图像亮度）
// IMX415 支持多曝光（HDR），所以有多个增益寄存器：增益 = 模拟放大倍数，值越大图像越亮（但也可能更噪）
// Long Frame（长曝光）增益
#define IMX415_LF_GAIN_REG_H		0x3091  // 高位
#define IMX415_LF_GAIN_REG_L		0x3090  // 低位

// Short Frame 1（短曝光1）增益
#define IMX415_SF1_GAIN_REG_H		0x3093
#define IMX415_SF1_GAIN_REG_L		0x3092

// Short Frame 2（短曝光2，仅 HDR X3 有）增益
#define IMX415_SF2_GAIN_REG_H		0x3095
#define IMX415_SF2_GAIN_REG_L		0x3094

// 曝光时间寄存器（控制感光时间）：同样分三路（LF/SF1/SF2）
// 长曝光起始行（SHR0）
#define IMX415_LF_EXPO_REG_H		0x3052  // 高8位
#define IMX415_LF_EXPO_REG_M		0x3051  // 中8位
#define IMX415_LF_EXPO_REG_L		0x3050  // 低8位

// 短曝光1 起始行（SHR1）
#define IMX415_SF1_EXPO_REG_H		0x3056
#define IMX415_SF1_EXPO_REG_M		0x3055
#define IMX415_SF1_EXPO_REG_L		0x3054

// 短曝光2 起始行（SHR2）
#define IMX415_SF2_EXPO_REG_H		0x305A
#define IMX415_SF2_EXPO_REG_M		0x3059
#define IMX415_SF2_EXPO_REG_L		0x3058

// RHS1 / RHS2 寄存器（HDR 关键时序）
// RHS1 必须满足 4n+1（DOL2）或 6n+1（DOL3），否则图像异常！这是 IMX415 HDR 的核心难点。
// RHS1：Short Exposure 1 结束行（DOL2/DOL3 共用）
#define IMX415_RHS1_REG_H		0x3062
#define IMX415_RHS1_REG_M		0x3061
#define IMX415_RHS1_REG_L		0x3060
#define IMX415_RHS1_DEFAULT		0x004D  // 默认值 77 行

// RHS2：Short Exposure 2 结束行（仅 DOL3 有）
#define IMX415_RHS2_REG_H		0x3066
#define IMX415_RHS2_REG_M		0x3065
#define IMX415_RHS2_REG_L		0x3064
#define IMX415_RHS2_DEFAULT		0x004D

// 曝光与增益范围（V4L2 控制项参数）
#define IMX415_EXPOSURE_MIN		4      // 最小曝光值
#define IMX415_EXPOSURE_STEP	1      // 步长
#define IMX415_VTS_MAX			0x7fff // 最大垂直总行数（帧周期上限）

#define IMX415_GAIN_MIN			0x00   // 最小增益
#define IMX415_GAIN_MAX			0xf0   // 最大增益（约 15x）
#define IMX415_GAIN_STEP		1
#define IMX415_GAIN_DEFAULT		0x00

// 位操作宏（拆分寄存器值）：由于寄存器是 8/16/24 位，而曝光/增益是整数，需要用这些宏拆分：
// 例如 gain = 0x123 → H=0x01, L=0x23
#define IMX415_FETCH_GAIN_H(VAL)	(((VAL) >> 8) & 0x07)
#define IMX415_FETCH_GAIN_L(VAL)	((VAL) & 0xFF)

// 曝光是 24 位值，拆成 H/M/L
#define IMX415_FETCH_EXP_H(VAL)		(((VAL) >> 16) & 0x0F)
#define IMX415_FETCH_EXP_M(VAL)		(((VAL) >> 8) & 0xFF)
#define IMX415_FETCH_EXP_L(VAL)		((VAL) & 0xFF)

#define IMX415_FETCH_RHS1_H(VAL)	(((VAL) >> 16) & 0x0F)
#define IMX415_FETCH_RHS1_M(VAL)	(((VAL) >> 8) & 0xFF)
#define IMX415_FETCH_RHS1_L(VAL)	((VAL) & 0xFF)

#define IMX415_FETCH_VTS_H(VAL)		(((VAL) >> 16) & 0x0F)
#define IMX415_FETCH_VTS_M(VAL)		(((VAL) >> 8) & 0xFF)
#define IMX415_FETCH_VTS_L(VAL)		((VAL) & 0xFF)

// VTS 寄存器（控制帧率）: 垂直总行数（Vertical Total Size）,24位，拆分为3个寄存器
// 帧率 = xvclk / (HTS × VTS)
// 增大 VTS → 降低帧率；减小 VTS → 提高帧率（但VTS不能小于图像高度 + 46）
#define IMX415_VTS_REG_L		0x3024 
#define IMX415_VTS_REG_M		0x3025
#define IMX415_VTS_REG_H		0x3026

// 图像翻转控制
#define IMX415_MIRROR_BIT_MASK		BIT(0)  // 水平翻转（镜像）
#define IMX415_FLIP_BIT_MASK		BIT(1)  // 垂直翻转
#define IMX415_FLIP_REG			0x3030  // 翻转控制寄存器

// 寄存器表特殊标记:表示寄存器配置表结束
#define REG_NULL			0xFFFF

// 表示延时（单位 ms），例如 {REG_DELAY, 30} 表示延时 30ms
#define REG_DELAY			0xFFFE

// 寄存器值长度（用于 I2C 写入）
#define IMX415_REG_VALUE_08BIT		1  // 8 位寄存器
#define IMX415_REG_VALUE_16BIT		2  // 16 位
#define IMX415_REG_VALUE_24BIT		3  // 24 位

// Group Hold（原子更新）
// 在修改多个相关寄存器时，先 START 锁定，改完再 END 一次性生效，避免中间状态导致图像异常特别用于 HDR 曝光切换
#define IMX415_GROUP_HOLD_REG		0x3001
#define IMX415_GROUP_HOLD_START		0x01  // 开始锁定寄存器
#define IMX415_GROUP_HOLD_END		0x00  // 解锁并生效

/* Basic Readout Lines. Number of necessary readout lines in sensor */
//  BRL（Basic Readout Lines，基础读出行数）:BRL 是传感器内部固定参数，用于计算 RHS1/RHS2 上限
#define BRL_ALL				2228u   // 全分辨率（3864×2192）下的 BRL
#define BRL_BINNING			1115u   // Binning 模式（1944×1097）下的 BRL

// 这些宏确保 RHS1/RHS2 满足 Sony 官方 datasheet 的数学约束
/* Readout timing setting of SEF1(DOL2): RHS1 < 2 * BRL and should be 4n + 1 */
// DOL2 模式下 RHS1 最大值（必须 ≤ 2×BRL 且 = 4n+1）
#define RHS1_MAX_X2(VAL)		(((VAL) * 2 - 1) / 4 * 4 + 1)
// DOL2 最小 SHR1
#define SHR1_MIN_X2			9u

/* Readout timing setting of SEF1(DOL3): RHS1 < 3 * BRL and should be 6n + 1 */
// DOL3 模式下 RHS1 最大值（必须 ≤ 3×BRL 且 = 6n+1）
#define RHS1_MAX_X3(VAL)		(((VAL) * 3 - 1) / 6 * 6 + 1)
// DOL3 最小 SHR1
#define SHR1_MIN_X3			13u

// 设备树引脚状态（pinctrl）
// 在 DTS 中定义 GPIO 引脚状态：
// pinctrl-names = "default", "sleep";
// pinctrl-0 = <&camera_default>;
// pinctrl-1 = <&camera_sleep>;
#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"

//  快速启动（Thunderboot）,设备树中启用快速启动（跳过部分初始化，用于安防/车载快速开机）
#define RKMODULE_CAMERA_FASTBOOT_ENABLE "rockchip,camera_fastboot"

// 驱动名称,用于日志、设备名等
#define IMX415_NAME			"imx415"

// 电源名称数组
// 列出 IMX415 传感器所需的三路电源名称。
// dvdd：数字核心电压（通常 1.2V）
// dovdd：数字 I/O 接口电压（通常 1.8V）
// avdd：模拟电路电压（通常 2.8V）
// Linux 内核通过 regulator_bulk_get() 一次性申请多路电源，需要按名字匹配设备树中的 regulator。
static const char * const imx415_supply_names[] = {
	"dvdd",		/* Digital core power */
	"dovdd",	/* Digital I/O power */
	"avdd",		/* Analog power */
};

// 定义电源数量（3），方便后续数组操作，避免硬编码。
#define IMX415_NUM_SUPPLIES ARRAY_SIZE(imx415_supply_names)

// 寄存器值结构体，用于构建寄存器配置表
// addr：寄存器地址（如 0x3000）
// val：要写入的值（如 0x00 启动流）
struct regval {
	u16 addr;
	u8 val;
};

// 工作模式结构体
// 核心作用：描述一种“预设工作模式”（分辨率+帧率+HDR+寄存器配置）。
// 举例：4K@30fps Linear 模式 vs 4K@30fps HDR DOL2 模式是两个不同的 imx415_mode。
struct imx415_mode {
	u32 bus_fmt;            // 媒体总线格式（如 MEDIA_BUS_FMT_SGBRG10_1X10）
	u32 width;              // 图像宽度（如 3864）
	u32 height;             // 图像高度（如 2192）
	struct v4l2_fract max_fps; // 最大帧率（numerator/denominator 表示分数）
	u32 hts_def;            // 水平总周期（Horizontal Total Size）
	u32 vts_def;            // 垂直总周期（Vertical Total Size）
	u32 exp_def;            // 默认曝光值
	u32 mipi_freq_idx;      // MIPI 频率索引（指向 link_freq_items[]）
	u32 bpp;                // 每像素位数（10 或 12）
	const struct regval *global_reg_list; // 全局寄存器配置（所有模式共用部分）
	const struct regval *reg_list;        // 模式专属寄存器配置
	u32 hdr_mode;           // HDR 模式（NO_HDR / HDR_X2 / HDR_X3）
	u32 vc[PAD_MAX];        // Virtual Channel 分配（HDR 多路输出用）
	u32 xvclk;              // 外部时钟频率（37.125MHz 或 27MHz）
};

// 驱动主结构体（驱动的“状态机”）
// 这是整个驱动的核心数据结构，保存了传感器的所有状态和资源。
// 每个 IMX415 实例（每个摄像头）都有一个 struct imx415 对象。
struct imx415 {
	struct i2c_client	*client;        // I2C 设备句柄
	struct clk		*xvclk;         // 外部时钟（xvclk）
	struct gpio_desc	*reset_gpio;    // 复位 GPIO
	struct gpio_desc	*power_gpio;    // 电源使能 GPIO（可选）

	// 三路电源管理
	struct regulator_bulk_data supplies[IMX415_NUM_SUPPLIES];

	// 引脚状态控制（pinctrl）
	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pins_default; // 正常工作状态
	struct pinctrl_state	*pins_sleep;   // 休眠状态

	// V4L2 子设备框架
	struct v4l2_subdev	subdev;
	struct media_pad	pad;

	// 控制接口（曝光/增益等）
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl	*exposure;
	struct v4l2_ctrl	*anal_a_gain;
	struct v4l2_ctrl	*digi_gain;
	struct v4l2_ctrl	*hblank;
	struct v4l2_ctrl	*vblank;
	struct v4l2_ctrl	*pixel_rate;
	struct v4l2_ctrl	*link_freq;

	struct mutex		mutex;          // 保护并发访问
	bool			streaming;      // 是否正在输出视频流
	bool			power_on;       // 电源是否开启
	
    // 快速启动（Thunderboot）相关
	u32			    is_thunderboot;
	bool			is_thunderboot_ng;
	bool			is_first_streamoff;

	// 当前支持的模式列表
	const struct imx415_mode *supported_modes; // 指向 supported_modes[] 或 supported_modes_2lane[]
	const struct imx415_mode *cur_mode;        // 当前激活的模式
	u32			 cfg_num;                      // 支持的模式总数

	// 模块信息（来自设备树）
	u32			     module_index;
	const char		*module_facing; // "front" / "back"
	const char		*module_name;
	const char		*len_name;      // 镜头型号

	// 运行时状态
	u32	   cur_vts;        // 当前 VTS 值（用于动态调整帧率）
	bool   has_init_exp;   // 是否已设置初始曝光
	struct preisp_hdrae_exp_s init_hdrae_exp; // 初始 HDR 曝光参数

	// 总线配置（从设备树解析）
	struct v4l2_fwnode_endpoint bus_cfg;
};

//  D-PHY 物理层参数（Samsung PHY 专用）
// 为 HDR X2 模式提供 Samsung D-PHY 的低功耗（LP）和高速（HS）时序参数。
static struct rkmodule_csi_dphy_param dcphy_param = {
	.vendor = PHY_VENDOR_SAMSUNG,
	.lp_vol_ref = 6,
	.lp_hys_sw = {3, 0, 0, 0},
	.lp_escclk_pol_sel = {1, 1, 1, 1},
	.skew_data_cal_clk = {0, 3, 3, 3},
	.clk_hs_term_sel = 2,
	.data_hs_term_sel = {2, 2, 2, 2},
	.reserved = {0},
};

// 容器转换宏
// 作用：从 v4l2_subdev *sd 指针反向获取所属的 struct imx415 对象。
#define to_imx415(sd) container_of(sd, struct imx415, subdev)



// 下面这些代码是 Sony IMX415 图像传感器驱动中用于配置不同工作模式的寄存器初始化表（register initialization tables）。每个数组对应一种特定的“工作模式”，包括：
// 分辨率（如 3864×2192、1932×1096、1284×720）
// 输出位深（10-bit 或 12-bit）
// 工作模式（Linear 线性 / HDR DOL2 双曝光 / HDR DOL3 三曝光）
// MIPI 带宽（如 891Mbps、1485Mbps、1782Mbps、2376Mbps）
// 数据通道数（4-lane 或 2-lane）
// 外部时钟（xvclk = 37.125MHz 或 27MHz）

// 函数命名规则：imx415_{linear/hdr2/hdr3/global}_{10/12bit}_{分辨率}_({速率}M)_regs
// 这些表会被 imx415_mode 结构体引用，并在 __imx415_start_stream() 中通过 imx415_write_array() 写入传感器，从而切换到目标工作模式。
// 这些寄存器值直接来自 Sony 官方 datasheet 和 tuning guide，驱动开发者不能随意修改，否则会导致图像异常、时序错误或传感器无响应。

// 每个数组元素是 struct regval { u16 addr; u8 val; }，表示：
// addr：寄存器地址（如 0x3024）
// val：要写入的值（如 0xCA）
// 数组以 {REG_NULL, 0x00} 结尾，表示结束。
// 有些数组包含 {REG_DELAY, N}，表示延时 N 毫秒（如 {REG_DELAY, 0x1E} = 延时 30ms）。

/*
 * Xclk 37.125Mhz
 函数名：imx415_global_12bit_3864x2192_regs
 作用：全局寄存器配置（所有 12-bit 4K 模式共用）
 特点：
  配置模拟前端、黑电平、时序基础参数
  不包含帧率、曝光、VTS 等动态参数
  在启动流之前先写入此表
 */
static __maybe_unused const struct regval imx415_global_12bit_3864x2192_regs[] = {
	{0x3002, 0x00},
	{0x3008, 0x7F},
	{0x300A, 0x5B},
	{0x30C1, 0x00},
	{0x3031, 0x01},
	{0x3032, 0x01},
	{0x30D9, 0x06},
	{0x3116, 0x24},
	{0x3118, 0xC0},
	{0x311E, 0x24},
	{0x32D4, 0x21},
	{0x32EC, 0xA1},
	{0x3452, 0x7F},
	{0x3453, 0x03},
	{0x358A, 0x04},
	{0x35A1, 0x02},
	{0x36BC, 0x0C},
	{0x36CC, 0x53},
	{0x36CD, 0x00},
	{0x36CE, 0x3C},
	{0x36D0, 0x8C},
	{0x36D1, 0x00},
	{0x36D2, 0x71},
	{0x36D4, 0x3C},
	{0x36D6, 0x53},
	{0x36D7, 0x00},
	{0x36D8, 0x71},
	{0x36DA, 0x8C},
	{0x36DB, 0x00},
	{0x3701, 0x03},
	{0x3724, 0x02},
	{0x3726, 0x02},
	{0x3732, 0x02},
	{0x3734, 0x03},
	{0x3736, 0x03},
	{0x3742, 0x03},
	{0x3862, 0xE0},
	{0x38CC, 0x30},
	{0x38CD, 0x2F},
	{0x395C, 0x0C},
	{0x3A42, 0xD1},
	{0x3A4C, 0x77},
	{0x3AE0, 0x02},
	{0x3AEC, 0x0C},
	{0x3B00, 0x2E},
	{0x3B06, 0x29},
	{0x3B98, 0x25},
	{0x3B99, 0x21},
	{0x3B9B, 0x13},
	{0x3B9C, 0x13},
	{0x3B9D, 0x13},
	{0x3B9E, 0x13},
	{0x3BA1, 0x00},
	{0x3BA2, 0x06},
	{0x3BA3, 0x0B},
	{0x3BA4, 0x10},
	{0x3BA5, 0x14},
	{0x3BA6, 0x18},
	{0x3BA7, 0x1A},
	{0x3BA8, 0x1A},
	{0x3BA9, 0x1A},
	{0x3BAC, 0xED},
	{0x3BAD, 0x01},
	{0x3BAE, 0xF6},
	{0x3BAF, 0x02},
	{0x3BB0, 0xA2},
	{0x3BB1, 0x03},
	{0x3BB2, 0xE0},
	{0x3BB3, 0x03},
	{0x3BB4, 0xE0},
	{0x3BB5, 0x03},
	{0x3BB6, 0xE0},
	{0x3BB7, 0x03},
	{0x3BB8, 0xE0},
	{0x3BBA, 0xE0},
	{0x3BBC, 0xDA},
	{0x3BBE, 0x88},
	{0x3BC0, 0x44},
	{0x3BC2, 0x7B},
	{0x3BC4, 0xA2},
	{0x3BC8, 0xBD},
	{0x3BCA, 0xBD},
	{0x4004, 0x48},
	{0x4005, 0x09},
	{REG_NULL, 0x00},
};

// 模式：线性（非 HDR），12-bit，4K（3864×2192），4-lane，891Mbps
// 关键寄存器：
// 0x3024/0x3025 → VTS = 0x08CA（控制帧率 ≈30fps）
// 0x3050 → SHR0 = 0x08（长曝光起始行）
// 0x3260 = 0x01 → 关闭 HDR（Linear 模式）
// 0x30CF = 0x00 → 单路输出（VC=0）
static __maybe_unused const struct regval imx415_linear_12bit_3864x2192_891M_regs[] = {
	{0x3020, 0x00},
	{0x3021, 0x00},
	{0x3022, 0x00},
	{0x3024, 0xCA},
	{0x3025, 0x08},
	{0x3028, 0x4C},
	{0x3029, 0x04},
	{0x302C, 0x00},
	{0x302D, 0x00},
	{0x3033, 0x05},
	{0x3050, 0x08},
	{0x3051, 0x00},
	{0x3054, 0x19},
	{0x3058, 0x3E},
	{0x3060, 0x25},
	{0x3064, 0x4A},
	{0x30CF, 0x00},
	{0x3260, 0x01},
	{0x400C, 0x00},
	{0x4018, 0x7F},
	{0x401A, 0x37},
	{0x401C, 0x37},
	{0x401E, 0xF7},
	{0x401F, 0x00},
	{0x4020, 0x3F},
	{0x4022, 0x6F},
	{0x4024, 0x3F},
	{0x4026, 0x5F},
	{0x4028, 0x2F},
	{0x4074, 0x01},
	{REG_NULL, 0x00},
};

// 模式：HDR DOL2（双曝光），12-bit，4K，4-lane，1782Mbps
// 关键区别：
// 0x3260 = 0x00 → 启用 HDR
// 0x3050 = 0x90, 0x3054 = 0x09 → 设置长短曝光起始行
// 0x3060 = 0x4D → RHS1 = 77（短曝光结束行，满足 4n+1）
// 0x30CF = 0x01 → 启用双 Virtual Channel（VC0=长曝光，VC1=短曝光）
static __maybe_unused const struct regval imx415_hdr2_12bit_3864x2192_1782M_regs[] = {
	{0x3020, 0x00},
	{0x3021, 0x00},
	{0x3022, 0x00},
	{0x3024, 0xCA},
	{0x3025, 0x08},
	{0x3028, 0x26},
	{0x3029, 0x02},
	{0x302C, 0x01},
	{0x302D, 0x01},
	{0x3033, 0x04},
	{0x3050, 0x90},
	{0x3051, 0x0D},
	{0x3054, 0x09},
	{0x3058, 0x3E},
	{0x3060, 0x4D},
	{0x3064, 0x4A},
	{0x30CF, 0x01},
	{0x3260, 0x00},
	{0x400C, 0x01},
	{0x4018, 0xB7},
	{0x401A, 0x67},
	{0x401C, 0x6F},
	{0x401E, 0xDF},
	{0x401F, 0x01},
	{0x4020, 0x6F},
	{0x4022, 0xCF},
	{0x4024, 0x6F},
	{0x4026, 0xB7},
	{0x4028, 0x5F},
	{0x4074, 0x00},
	{REG_NULL, 0x00},
};

// 模式：HDR DOL3（三曝光），12-bit，4K，4-lane，1782Mbps
// 关键区别：
// 0x30CF = 0x03 → 三路 VC 输出
// 0x3060 = 0x19, 0x3064 = 0x32 → RHS1=25, RHS2=50（满足 6n+1 / 6n+2）
// 三个曝光起始行：0x3050（L）、0x3054（M）、0x3058（S）
static __maybe_unused const struct regval imx415_hdr3_12bit_3864x2192_1782M_regs[] = {
	{0x3020, 0x00},
	{0x3021, 0x00},
	{0x3022, 0x00},
	{0x3024, 0x96},
	{0x3025, 0x06},
	{0x3028, 0x26},
	{0x3029, 0x02},
	{0x302C, 0x01},
	{0x302D, 0x02},
	{0x3033, 0x04},
	{0x3050, 0x14},
	{0x3051, 0x01},
	{0x3054, 0x0D},
	{0x3058, 0x26},
	{0x3060, 0x19},
	{0x3064, 0x32},
	{0x30CF, 0x03},
	{0x3260, 0x00},
	{0x400C, 0x01},
	{0x4018, 0xB7},
	{0x401A, 0x67},
	{0x401C, 0x6F},
	{0x401E, 0xDF},
	{0x401F, 0x01},
	{0x4020, 0x6F},
	{0x4022, 0xCF},
	{0x4024, 0x6F},
	{0x4026, 0xB7},
	{0x4028, 0x5F},
	{0x4074, 0x00},
	{REG_NULL, 0x00},
};

// 作用：10-bit 模式的全局寄存器配置
// 与 12-bit 区别：
// 0x3031/0x3032 = 0x00（12-bit 是 0x01）→ 选择 10-bit ADC
// 其他参数微调以适配 10-bit 数据路径
static __maybe_unused const struct regval imx415_global_10bit_3864x2192_regs[] = {
	{0x3002, 0x00},
	{0x3008, 0x7F},
	{0x300A, 0x5B},
	{0x3031, 0x00},
	{0x3032, 0x00},
	{0x30C1, 0x00},
	{0x30D9, 0x06},
	{0x3116, 0x24},
	{0x311E, 0x24},
	{0x32D4, 0x21},
	{0x32EC, 0xA1},
	{0x3452, 0x7F},
	{0x3453, 0x03},
	{0x358A, 0x04},
	{0x35A1, 0x02},
	{0x36BC, 0x0C},
	{0x36CC, 0x53},
	{0x36CD, 0x00},
	{0x36CE, 0x3C},
	{0x36D0, 0x8C},
	{0x36D1, 0x00},
	{0x36D2, 0x71},
	{0x36D4, 0x3C},
	{0x36D6, 0x53},
	{0x36D7, 0x00},
	{0x36D8, 0x71},
	{0x36DA, 0x8C},
	{0x36DB, 0x00},
	{0x3701, 0x00},
	{0x3724, 0x02},
	{0x3726, 0x02},
	{0x3732, 0x02},
	{0x3734, 0x03},
	{0x3736, 0x03},
	{0x3742, 0x03},
	{0x3862, 0xE0},
	{0x38CC, 0x30},
	{0x38CD, 0x2F},
	{0x395C, 0x0C},
	{0x3A42, 0xD1},
	{0x3A4C, 0x77},
	{0x3AE0, 0x02},
	{0x3AEC, 0x0C},
	{0x3B00, 0x2E},
	{0x3B06, 0x29},
	{0x3B98, 0x25},
	{0x3B99, 0x21},
	{0x3B9B, 0x13},
	{0x3B9C, 0x13},
	{0x3B9D, 0x13},
	{0x3B9E, 0x13},
	{0x3BA1, 0x00},
	{0x3BA2, 0x06},
	{0x3BA3, 0x0B},
	{0x3BA4, 0x10},
	{0x3BA5, 0x14},
	{0x3BA6, 0x18},
	{0x3BA7, 0x1A},
	{0x3BA8, 0x1A},
	{0x3BA9, 0x1A},
	{0x3BAC, 0xED},
	{0x3BAD, 0x01},
	{0x3BAE, 0xF6},
	{0x3BAF, 0x02},
	{0x3BB0, 0xA2},
	{0x3BB1, 0x03},
	{0x3BB2, 0xE0},
	{0x3BB3, 0x03},
	{0x3BB4, 0xE0},
	{0x3BB5, 0x03},
	{0x3BB6, 0xE0},
	{0x3BB7, 0x03},
	{0x3BB8, 0xE0},
	{0x3BBA, 0xE0},
	{0x3BBC, 0xDA},
	{0x3BBE, 0x88},
	{0x3BC0, 0x44},
	{0x3BC2, 0x7B},
	{0x3BC4, 0xA2},
	{0x3BC8, 0xBD},
	{0x3BCA, 0xBD},
	{0x4004, 0x48},
	{0x4005, 0x09},
	{REG_NULL, 0x00},
};

// 其他 HDR/Linear 10-bit 模式
// 命名规则：imx415_{linear/hdr2/hdr3}_{10bit}_{分辨率}_{速率}M_regs
// 使用 10-bit 寄存器值
// VTS/HTS 根据帧率调整（如 1485M vs 1782M）
// HDR 模式下 RHS1/RHS2 仍满足 4n+1 / 6n+1 约束
static __maybe_unused const struct regval imx415_hdr3_10bit_3864x2192_1485M_regs[] = {
	{0x3020, 0x00},
	{0x3021, 0x00},
	{0x3022, 0x00},
	{0x3024, 0xBD},
	{0x3025, 0x06},
	{0x3028, 0x1A},
	{0x3029, 0x02},
	{0x302C, 0x01},
	{0x302D, 0x02},
	{0x3033, 0x08},
	{0x3050, 0x90},
	{0x3051, 0x15},
	{0x3054, 0x0D},
	{0x3058, 0xA4},
	{0x3060, 0x97},
	{0x3064, 0xB6},
	{0x30CF, 0x03},
	{0x3118, 0xA0},
	{0x3260, 0x00},
	{0x400C, 0x01},
	{0x4018, 0xA7},
	{0x401A, 0x57},
	{0x401C, 0x5F},
	{0x401E, 0x97},
	{0x401F, 0x01},
	{0x4020, 0x5F},
	{0x4022, 0xAF},
	{0x4024, 0x5F},
	{0x4026, 0x9F},
	{0x4028, 0x4F},
	{0x4074, 0x00},
	{REG_NULL, 0x00},
};

// 其他 HDR/Linear 10-bit 模式
// 命名规则：imx415_{linear/hdr2/hdr3}_{10bit}_{分辨率}_{速率}M_regs
// 使用 10-bit 寄存器值
// VTS/HTS 根据帧率调整（如 1485M vs 1782M）
// HDR 模式下 RHS1/RHS2 仍满足 4n+1 / 6n+1 约束
static __maybe_unused const struct regval imx415_hdr3_10bit_3864x2192_1782M_regs[] = {
	{0x3020, 0x00},
	{0x3021, 0x00},
	{0x3022, 0x00},
	{0x3024, 0xEA},
	{0x3025, 0x07},
	{0x3028, 0xCA},
	{0x3029, 0x01},
	{0x302C, 0x01},
	{0x302D, 0x02},
	{0x3033, 0x04},
	{0x3050, 0x3E},
	{0x3051, 0x01},
	{0x3054, 0x0D},
	{0x3058, 0x9E},
	{0x3060, 0x91},
	{0x3064, 0xC2},
	{0x30CF, 0x03},
	{0x3118, 0xC0},
	{0x3260, 0x00},
	{0x400C, 0x01},
	{0x4018, 0xB7},
	{0x401A, 0x67},
	{0x401C, 0x6F},
	{0x401E, 0xDF},
	{0x401F, 0x01},
	{0x4020, 0x6F},
	{0x4022, 0xCF},
	{0x4024, 0x6F},
	{0x4026, 0xB7},
	{0x4028, 0x5F},
	{0x4074, 0x00},
	{REG_NULL, 0x00},
};

// 其他 HDR/Linear 10-bit 模式
// 命名规则：imx415_{linear/hdr2/hdr3}_{10bit}_{分辨率}_{速率}M_regs
// 使用 10-bit 寄存器值
// VTS/HTS 根据帧率调整（如 1485M vs 1782M）
// HDR 模式下 RHS1/RHS2 仍满足 4n+1 / 6n+1 约束
static __maybe_unused const struct regval imx415_hdr2_10bit_3864x2192_1485M_regs[] = {
	{0x3020, 0x00},
	{0x3021, 0x00},
	{0x3022, 0x00},
	{0x3024, 0xFC},
	{0x3025, 0x08},
	{0x3028, 0x1A},
	{0x3029, 0x02},
	{0x302C, 0x01},
	{0x302D, 0x01},
	{0x3033, 0x08},
	{0x3050, 0xA8},
	{0x3051, 0x0D},
	{0x3054, 0x09},
	{0x3058, 0x3E},
	{0x3060, 0x4D},
	{0x3064, 0x4a},
	{0x30CF, 0x01},
	{0x3118, 0xA0},
	{0x3260, 0x00},
	{0x400C, 0x01},
	{0x4018, 0xA7},
	{0x401A, 0x57},
	{0x401C, 0x5F},
	{0x401E, 0x97},
	{0x401F, 0x01},
	{0x4020, 0x5F},
	{0x4022, 0xAF},
	{0x4024, 0x5F},
	{0x4026, 0x9F},
	{0x4028, 0x4F},
	{0x4074, 0x00},
	{REG_NULL, 0x00},
};

// 其他 HDR/Linear 10-bit 模式
// 命名规则：imx415_{linear/hdr2/hdr3}_{10bit}_{分辨率}_{速率}M_regs
// 使用 10-bit 寄存器值
// VTS/HTS 根据帧率调整（如 1485M vs 1782M）
// HDR 模式下 RHS1/RHS2 仍满足 4n+1 / 6n+1 约束
static __maybe_unused const struct regval imx415_linear_10bit_3864x2192_891M_regs[] = {
	{0x3020, 0x00},
	{0x3021, 0x00},
	{0x3022, 0x00},
	{0x3024, 0xCA},
	{0x3025, 0x08},
	{0x3028, 0x4C},
	{0x3029, 0x04},
	{0x302C, 0x00},
	{0x302D, 0x00},
	{0x3033, 0x05},
	{0x3050, 0x08},
	{0x3051, 0x00},
	{0x3054, 0x19},
	{0x3058, 0x3E},
	{0x3060, 0x25},
	{0x3064, 0x4a},
	{0x30CF, 0x00},
	{0x3118, 0xC0},
	{0x3260, 0x01},
	{0x400C, 0x00},
	{0x4018, 0x7F},
	{0x401A, 0x37},
	{0x401C, 0x37},
	{0x401E, 0xF7},
	{0x401F, 0x00},
	{0x4020, 0x3F},
	{0x4022, 0x6F},
	{0x4024, 0x3F},
	{0x4026, 0x5F},
	{0x4028, 0x2F},
	{0x4074, 0x01},
	{REG_NULL, 0x00},
};

// 其他 HDR/Linear 10-bit 模式
// 命名规则：imx415_{linear/hdr2/hdr3}_{10bit}_{分辨率}_{速率}M_regs
// 使用 10-bit 寄存器值
// VTS/HTS 根据帧率调整（如 1485M vs 1782M）
// HDR 模式下 RHS1/RHS2 仍满足 4n+1 / 6n+1 约束
static __maybe_unused const struct regval imx415_linear_10bit_3864x2192_1485M_regs[] = {
	{0x3020, 0x00},
	{0x3021, 0x00},
	{0x3022, 0x00},
	{0x3024, 0xCA},
	{0x3025, 0x08},
	{0x3028, 0x26},
	{0x3029, 0x02},
	{0x302C, 0x00},
	{0x302D, 0x00},
	{0x3033, 0x08},
	{0x3034, 0x08},
	{0x3050, 0x08},
	{0x3051, 0x00},
	{0x3054, 0x19},
	{0x3058, 0x3E},
	{0x3060, 0x25},
	{0x3064, 0x4a},
	{0x30CF, 0x00},
	{0x3118, 0xA0},
	{0x3260, 0x01},
	{0x400C, 0x01},
	{0x4018, 0xA7},
	{0x401A, 0x57},
	{0x401C, 0x5F},
	{0x401E, 0x97},
	{0x401F, 0x01},
	{0x4020, 0x5F},
	{0x4022, 0xAF},
	{0x4024, 0x5F},
	{0x4026, 0x9F},
	{0x4028, 0x4F},
	{0x4074, 0x00},
	{REG_NULL, 0x00},
};

// 模式：Binning 模式（1944×1097，实际寄存器写 1932×1096）
// 特点：
// 2×2 像素合并，降低分辨率提高帧率/灵敏度
// xvclk 仍为 37.125MHz，但带宽降至 594Mbps
static __maybe_unused const struct regval imx415_linear_12bit_1932x1096_594M_regs[] = {
	{0x3020, 0x01},
	{0x3021, 0x01},
	{0x3022, 0x01},
	{0x3024, 0x5D},
	{0x3025, 0x0C},
	{0x3028, 0x0E},
	{0x3029, 0x03},
	{0x302C, 0x00},
	{0x302D, 0x00},
	{0x3031, 0x00},
	{0x3033, 0x07},
	{0x3050, 0x08},
	{0x3051, 0x00},
	{0x3054, 0x19},
	{0x3058, 0x3E},
	{0x3060, 0x25},
	{0x3064, 0x4A},
	{0x30CF, 0x00},
	{0x30D9, 0x02},
	{0x30DA, 0x01},
	{0x3118, 0x80},
	{0x3260, 0x01},
	{0x3701, 0x00},
	{0x400C, 0x00},
	{0x4018, 0x67},
	{0x401A, 0x27},
	{0x401C, 0x27},
	{0x401E, 0xB7},
	{0x401F, 0x00},
	{0x4020, 0x2F},
	{0x4022, 0x4F},
	{0x4024, 0x2F},
	{0x4026, 0x47},
	{0x4028, 0x27},
	{0x4074, 0x01},
	{REG_NULL, 0x00},
};

// Binning + HDR DOL2 模式
static __maybe_unused const struct regval imx415_hdr2_12bit_1932x1096_891M_regs[] = {
	{0x3020, 0x01},
	{0x3021, 0x01},
	{0x3022, 0x01},
	{0x3024, 0xFC},
	{0x3025, 0x08},
	{0x3028, 0x1A},
	{0x3029, 0x02},
	{0x302C, 0x01},
	{0x302D, 0x01},
	{0x3031, 0x00},
	{0x3033, 0x05},
	{0x3050, 0xB8},
	{0x3051, 0x00},
	{0x3054, 0x09},
	{0x3058, 0x3E},
	{0x3060, 0x25},
	{0x3064, 0x4A},
	{0x30CF, 0x01},
	{0x30D9, 0x02},
	{0x30DA, 0x01},
	{0x3118, 0xC0},
	{0x3260, 0x00},
	{0x3701, 0x00},
	{0x400C, 0x00},
	{0x4018, 0xA7},
	{0x401A, 0x57},
	{0x401C, 0x5F},
	{0x401E, 0x97},
	{0x401F, 0x01},
	{0x4020, 0x5F},
	{0x4022, 0xAF},
	{0x4024, 0x5F},
	{0x4026, 0x9F},
	{0x4028, 0x4F},
	{0x4074, 0x01},
	{REG_NULL, 0x00},
};



//  2-lane 模式（低引脚数设计）
// 关键变化：
// xvclk = 27MHz（不是 37.125MHz）
// 使用 2 条 MIPI 通道（节省硬件引脚）
// 带宽仍达 891Mbps（靠更高每 lane 速率）
// 包含 {REG_DELAY, 0x1E} → 启动后延时 30ms 稳定
// 注释掉 {0x3000, 0x00} → 流启动由上层控制
/*
 * Xclk 27Mhz
 * 15fps
 * CSI-2_2lane
 * AD:12bit Output:12bit
 * 891Mbps
 * Master Mode
 * Time 9.988ms Gain:6dB
 * All-pixel
 */
static __maybe_unused const struct regval imx415_linear_12bit_3864x2192_891M_regs_2lane[] = {
	{0x3008, 0x5D},
	{0x300A, 0x42},
	{0x3028, 0x98},
	{0x3029, 0x08},
	{0x3033, 0x05},
	{0x3050, 0x79},
	{0x3051, 0x07},
	{0x3090, 0x14},
	{0x30C1, 0x00},
	{0x3116, 0x23},
	{0x3118, 0xC6},
	{0x311A, 0xE7},
	{0x311E, 0x23},
	{0x32D4, 0x21},
	{0x32EC, 0xA1},
	{0x344C, 0x2B},
	{0x344D, 0x01},
	{0x344E, 0xED},
	{0x344F, 0x01},
	{0x3450, 0xF6},
	{0x3451, 0x02},
	{0x3452, 0x7F},
	{0x3453, 0x03},
	{0x358A, 0x04},
	{0x35A1, 0x02},
	{0x35EC, 0x27},
	{0x35EE, 0x8D},
	{0x35F0, 0x8D},
	{0x35F2, 0x29},
	{0x36BC, 0x0C},
	{0x36CC, 0x53},
	{0x36CD, 0x00},
	{0x36CE, 0x3C},
	{0x36D0, 0x8C},
	{0x36D1, 0x00},
	{0x36D2, 0x71},
	{0x36D4, 0x3C},
	{0x36D6, 0x53},
	{0x36D7, 0x00},
	{0x36D8, 0x71},
	{0x36DA, 0x8C},
	{0x36DB, 0x00},
	{0x3720, 0x00},
	{0x3724, 0x02},
	{0x3726, 0x02},
	{0x3732, 0x02},
	{0x3734, 0x03},
	{0x3736, 0x03},
	{0x3742, 0x03},
	{0x3862, 0xE0},
	{0x38CC, 0x30},
	{0x38CD, 0x2F},
	{0x395C, 0x0C},
	{0x39A4, 0x07},
	{0x39A8, 0x32},
	{0x39AA, 0x32},
	{0x39AC, 0x32},
	{0x39AE, 0x32},
	{0x39B0, 0x32},
	{0x39B2, 0x2F},
	{0x39B4, 0x2D},
	{0x39B6, 0x28},
	{0x39B8, 0x30},
	{0x39BA, 0x30},
	{0x39BC, 0x30},
	{0x39BE, 0x30},
	{0x39C0, 0x30},
	{0x39C2, 0x2E},
	{0x39C4, 0x2B},
	{0x39C6, 0x25},
	{0x3A42, 0xD1},
	{0x3A4C, 0x77},
	{0x3AE0, 0x02},
	{0x3AEC, 0x0C},
	{0x3B00, 0x2E},
	{0x3B06, 0x29},
	{0x3B98, 0x25},
	{0x3B99, 0x21},
	{0x3B9B, 0x13},
	{0x3B9C, 0x13},
	{0x3B9D, 0x13},
	{0x3B9E, 0x13},
	{0x3BA1, 0x00},
	{0x3BA2, 0x06},
	{0x3BA3, 0x0B},
	{0x3BA4, 0x10},
	{0x3BA5, 0x14},
	{0x3BA6, 0x18},
	{0x3BA7, 0x1A},
	{0x3BA8, 0x1A},
	{0x3BA9, 0x1A},
	{0x3BAC, 0xED},
	{0x3BAD, 0x01},
	{0x3BAE, 0xF6},
	{0x3BAF, 0x02},
	{0x3BB0, 0xA2},
	{0x3BB1, 0x03},
	{0x3BB2, 0xE0},
	{0x3BB3, 0x03},
	{0x3BB4, 0xE0},
	{0x3BB5, 0x03},
	{0x3BB6, 0xE0},
	{0x3BB7, 0x03},
	{0x3BB8, 0xE0},
	{0x3BBA, 0xE0},
	{0x3BBC, 0xDA},
	{0x3BBE, 0x88},
	{0x3BC0, 0x44},
	{0x3BC2, 0x7B},
	{0x3BC4, 0xA2},
	{0x3BC8, 0xBD},
	{0x3BCA, 0xBD},
	{0x4001, 0x01},
	{0x4004, 0xC0},
	{0x4005, 0x06},
	{0x400C, 0x00},
	{0x4018, 0x7F},
	{0x401A, 0x37},
	{0x401C, 0x37},
	{0x401E, 0xF7},
	{0x401F, 0x00},
	{0x4020, 0x3F},
	{0x4022, 0x6F},
	{0x4024, 0x3F},
	{0x4026, 0x5F},
	{0x4028, 0x2F},
	{0x4074, 0x01},
	{0x3002, 0x00},
	//{0x3000, 0x00},
	{REG_DELAY, 0x1E},//wait_ms(30)
	{REG_NULL, 0x00},
};


// 超高帧率模式：
// 分辨率：1284×720（通过窗口裁剪 + binning）
// 帧率：90fps
// MIPI 速率：2376Mbps（2-lane × 1188Mbps/lane）
// xvclk = 27MHz
// 寄存器 0x3040~0x3047 配置裁剪窗口（2568×1440 → 1284×720）

/*
 * Xclk 27Mhz
 * 90.059fps
 * CSI-2_2lane
 * AD:10bit Output:12bit
 * 2376Mbps
 * Master Mode
 * Time 9.999ms Gain:6dB
 * 2568x1440 2/2-line binning & Window cropping
 */
static __maybe_unused const struct regval imx415_linear_12bit_1284x720_2376M_regs_2lane[] = {
	{0x3008, 0x5D},
	{0x300A, 0x42},
	{0x301C, 0x04},
	{0x3020, 0x01},
	{0x3021, 0x01},
	{0x3022, 0x01},
	{0x3024, 0xAB},
	{0x3025, 0x07},
	{0x3028, 0xA4},
	{0x3029, 0x01},
	{0x3031, 0x00},
	{0x3033, 0x00},
	{0x3040, 0x88},
	{0x3041, 0x02},
	{0x3042, 0x08},
	{0x3043, 0x0A},
	{0x3044, 0xF0},
	{0x3045, 0x02},
	{0x3046, 0x40},
	{0x3047, 0x0B},
	{0x3050, 0xC4},
	{0x3090, 0x14},
	{0x30C1, 0x00},
	{0x30D9, 0x02},
	{0x30DA, 0x01},
	{0x3116, 0x23},
	{0x3118, 0x08},
	{0x3119, 0x01},
	{0x311A, 0xE7},
	{0x311E, 0x23},
	{0x32D4, 0x21},
	{0x32EC, 0xA1},
	{0x344C, 0x2B},
	{0x344D, 0x01},
	{0x344E, 0xED},
	{0x344F, 0x01},
	{0x3450, 0xF6},
	{0x3451, 0x02},
	{0x3452, 0x7F},
	{0x3453, 0x03},
	{0x358A, 0x04},
	{0x35A1, 0x02},
	{0x35EC, 0x27},
	{0x35EE, 0x8D},
	{0x35F0, 0x8D},
	{0x35F2, 0x29},
	{0x36BC, 0x0C},
	{0x36CC, 0x53},
	{0x36CD, 0x00},
	{0x36CE, 0x3C},
	{0x36D0, 0x8C},
	{0x36D1, 0x00},
	{0x36D2, 0x71},
	{0x36D4, 0x3C},
	{0x36D6, 0x53},
	{0x36D7, 0x00},
	{0x36D8, 0x71},
	{0x36DA, 0x8C},
	{0x36DB, 0x00},
	{0x3701, 0x00},
	{0x3720, 0x00},
	{0x3724, 0x02},
	{0x3726, 0x02},
	{0x3732, 0x02},
	{0x3734, 0x03},
	{0x3736, 0x03},
	{0x3742, 0x03},
	{0x3862, 0xE0},
	{0x38CC, 0x30},
	{0x38CD, 0x2F},
	{0x395C, 0x0C},
	{0x39A4, 0x07},
	{0x39A8, 0x32},
	{0x39AA, 0x32},
	{0x39AC, 0x32},
	{0x39AE, 0x32},
	{0x39B0, 0x32},
	{0x39B2, 0x2F},
	{0x39B4, 0x2D},
	{0x39B6, 0x28},
	{0x39B8, 0x30},
	{0x39BA, 0x30},
	{0x39BC, 0x30},
	{0x39BE, 0x30},
	{0x39C0, 0x30},
	{0x39C2, 0x2E},
	{0x39C4, 0x2B},
	{0x39C6, 0x25},
	{0x3A42, 0xD1},
	{0x3A4C, 0x77},
	{0x3AE0, 0x02},
	{0x3AEC, 0x0C},
	{0x3B00, 0x2E},
	{0x3B06, 0x29},
	{0x3B98, 0x25},
	{0x3B99, 0x21},
	{0x3B9B, 0x13},
	{0x3B9C, 0x13},
	{0x3B9D, 0x13},
	{0x3B9E, 0x13},
	{0x3BA1, 0x00},
	{0x3BA2, 0x06},
	{0x3BA3, 0x0B},
	{0x3BA4, 0x10},
	{0x3BA5, 0x14},
	{0x3BA6, 0x18},
	{0x3BA7, 0x1A},
	{0x3BA8, 0x1A},
	{0x3BA9, 0x1A},
	{0x3BAC, 0xED},
	{0x3BAD, 0x01},
	{0x3BAE, 0xF6},
	{0x3BAF, 0x02},
	{0x3BB0, 0xA2},
	{0x3BB1, 0x03},
	{0x3BB2, 0xE0},
	{0x3BB3, 0x03},
	{0x3BB4, 0xE0},
	{0x3BB5, 0x03},
	{0x3BB6, 0xE0},
	{0x3BB7, 0x03},
	{0x3BB8, 0xE0},
	{0x3BBA, 0xE0},
	{0x3BBC, 0xDA},
	{0x3BBE, 0x88},
	{0x3BC0, 0x44},
	{0x3BC2, 0x7B},
	{0x3BC4, 0xA2},
	{0x3BC8, 0xBD},
	{0x3BCA, 0xBD},
	{0x4001, 0x01},
	{0x4004, 0xC0},
	{0x4005, 0x06},
	{0x4018, 0xE7},
	{0x401A, 0x8F},
	{0x401C, 0x8F},
	{0x401E, 0x7F},
	{0x401F, 0x02},
	{0x4020, 0x97},
	{0x4022, 0x0F},
	{0x4023, 0x01},
	{0x4024, 0x97},
	{0x4026, 0xF7},
	{0x4028, 0x7F},
	{0x3002, 0x00},
	//{0x3000, 0x00},
	{REG_DELAY, 0x1E},//wait_ms(30)
	{REG_NULL, 0x00},
};

/*
 * The width and height must be configured to be
 * the same as the current output resolution of the sensor.
 * The input width of the isp needs to be 16 aligned.
 * The input height of the isp needs to be 8 aligned.
 * If the width or height does not meet the alignment rules,
 * you can configure the cropping parameters with the following function to
 * crop out the appropriate resolution.
 * struct v4l2_subdev_pad_ops {
 *	.get_selection
 * }
 */
 // 这段代码定义了 IMX415 传感器支持的所有工作模式（supported_modes[]），每个元素对应一种特定的配置组合。
// 这是一个 只读数组，包含 IMX415 在 4-lane MIPI 模式下 支持的所有分辨率/帧率/HDR/位深组合。
// 每个 imx415_mode 结构体描述一种“预设模式”，驱动在 set_fmt 时会从中选择最匹配的一项。
 static const struct imx415_mode supported_modes[] = {
	/*
	 * frame rate = 1 / (Vtt * 1H) = 1 / (VMAX * 1H)
	 * VMAX >= (PIX_VWIDTH / 2) + 46 = height + 46
	 */
     // 字段详解（以第1个模式为例,每个模式以{}结构体划分）
    // 分辨率为 3864×2192（全像素输出）。
    // 使用 10-bit 输出格式（MEDIA_BUS_FMT_SGBRG10_1X10）。
    // 最大帧率为 60fps（由 .max_fps = {10000, 600000} 计算得出）。
    // 工作在 Linear（非 HDR） 模式。
    // MIPI 频率索引为 2，对应 743 MHz（即 link_freq_items[2]）。
    // 使用的寄存器表为 imx415_linear_10bit_3864x2192_1485M_regs。
    // 外部时钟 xvclk 为 37.125MHz。
    // Virtual Channel 仅使用 PAD0（VC0），值为 V4L2_MBUS_CSI2_CHANNEL_0。
    // 此模式适用于高帧率预览场景，如无人机图传或高速运动捕捉。
     {
		.bus_fmt = MEDIA_BUS_FMT_SGBRG10_1X10, // 媒体总线格式,表示输出 10-bit Bayer 格式（SGBRG 排列）
		.width = 3864, // 全像素（Full Pixel）
		.height = 2192,
		.max_fps = { // 最大帧率：用分数表示
			.numerator = 10000,
			.denominator = 600000,
		},
        // 默认曝光值（单位：行）
        // 计算方式：VTS - SHR0（即有效曝光行数）
        // 例如 0x08ca - 0x08 = 2250 - 8 = 2242 行
		.exp_def = 0x08ca - 0x08, 

        // 水平总周期（HTS）：一行的总像素数（含 blanking）
        // 0x0226 = 550，乘以 4-lane × 2（DDR）→ 实际 HTS = 550 × 8 = 4400
        // 用于计算像素时钟：pixel_rate = HTS × VTS × fps
		.hts_def = 0x0226 * IMX415_4LANES * 2,

        // 垂直总周期（VTS）：一帧的总行数（含 blanking）
        // 0x08ca = 2250 行
        // 控制帧率：fps = xvclk / (HTS × VTS)
		.vts_def = 0x08ca,

        // 寄存器配置表
		.global_reg_list = imx415_global_10bit_3864x2192_regs, //  // 前面的数组序列配置表，基础配置（如 ADC、黑电平）
		.reg_list = imx415_linear_10bit_3864x2192_1485M_regs, //  前面的数组序列配置表，模式专属配置（帧率、曝光、HDR 开关等）
		.hdr_mode = NO_HDR, // HDR 模式：NO_HDR：线性模式；HDR_X2：双曝光（DOL2）；HDR_X3：三曝光（DOL3）
		
        // MIPI 频率索引：指向 link_freq_items[] 数组
        // static const s64 link_freq_items[] = {
        //     MIPI_FREQ_297M, // idx=0
        //     MIPI_FREQ_446M, // idx=1
        //     MIPI_FREQ_743M, // idx=2
        //     MIPI_FREQ_891M, // idx=3
        //     MIPI_FREQ_1188M,// idx=4
        // };
        .mipi_freq_idx = 2,

        // 每像素位数：10 或 12
		.bpp = 10,

        // Virtual Channel 分配（仅 HDR 模式需要）：
        // HDR_X2：VC0=短曝光，VC1=长曝光
        // HDR_X3：VC0=短，VC1=中，VC2=长
        // 注释如 //L->csi wr0 表示“长曝光映射到 CSI 写通道 0”
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,

        // 外部时钟频率：
        // 37.125 MHz（全分辨率）
        // 27 MHz（2-lane 模式）
		.xvclk = IMX415_XVCLK_FREQ_37M,
	},

    // 第2个模式
    // 分辨率为 3864×2192。
    // 使用 10-bit 输出格式。
    // 最大帧率为 30fps。
    // 工作在 Linear 模式。
    // MIPI 频率索引为 1，对应 446 MHz。
    // 使用的寄存器表为 imx415_linear_10bit_3864x2192_891M_regs。
    // xvclk 为 37.125MHz。
    // VC 为 V4L2_MBUS_CSI2_CHANNEL_0。
    // 这是标准 4K@30fps 模式，带宽较低，适合普通视频录制。
	{
		.bus_fmt = MEDIA_BUS_FMT_SGBRG10_1X10,
		.width = 3864,
		.height = 2192,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x08ca - 0x08,
		.hts_def = 0x044c * IMX415_4LANES * 2,
		.vts_def = 0x08ca,
		.global_reg_list = imx415_global_10bit_3864x2192_regs,
		.reg_list = imx415_linear_10bit_3864x2192_891M_regs,
		.hdr_mode = NO_HDR,
		.mipi_freq_idx = 1,
		.bpp = 10,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
		.xvclk = IMX415_XVCLK_FREQ_37M,
	},

    // 第3个模式
    // 分辨率为 3864×2192。
    // 使用 10-bit 输出格式。
    // 最大帧率为 30fps。
    // 工作在 HDR_X2（DOL2 双曝光） 模式。
    // MIPI 频率索引为 2，对应 743 MHz。
    // 使用的寄存器表为 imx415_hdr2_10bit_3864x2192_1485M_regs。
    // xvclk 为 37.125MHz。
    // Virtual Channel 配置为：
    // PAD0 → VC1（短曝光）
    // PAD1 → VC0（长曝光）
    // PAD2 → VC1
    // PAD3 → VC1（中间曝光映射，实际为双曝光）
    // VTS 被设为 0x08fc * 2，以补偿 HDR 模式下 T-line 减半的问题。
    // 适用于需要中等动态范围的场景，如室内外过渡区域监控。
    {
		.bus_fmt = MEDIA_BUS_FMT_SGBRG10_1X10,
		.width = 3864,
		.height = 2192,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x08fc * 2 - 0x0da8,
		.hts_def = 0x0226 * IMX415_4LANES * 2,
		/*
		 * IMX415 HDR mode T-line is half of Linear mode,
		 * make vts double to workaround.
		 */
		.vts_def = 0x08fc * 2,
		.global_reg_list = imx415_global_10bit_3864x2192_regs,
		.reg_list = imx415_hdr2_10bit_3864x2192_1485M_regs,
		.hdr_mode = HDR_X2,
		.mipi_freq_idx = 2,
		.bpp = 10,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_1,
		.vc[PAD1] = V4L2_MBUS_CSI2_CHANNEL_0,//L->csi wr0，表示“长曝光映射到 CSI 写通道 0”
		.vc[PAD2] = V4L2_MBUS_CSI2_CHANNEL_1,
		.vc[PAD3] = V4L2_MBUS_CSI2_CHANNEL_1,//M->csi wr2
		.xvclk = IMX415_XVCLK_FREQ_37M,
	},

    // 第4个模式
    // 分辨率为 3864×2192。
    // 使用 10-bit 输出格式。
    // 最大帧率为 20fps。
    // 工作在 HDR_X3（DOL3 三曝光） 模式。
    // MIPI 频率索引为 2，对应 743 MHz。
    // 使用的寄存器表为 imx415_hdr3_10bit_3864x2192_1485M_regs。
    // xvclk 为 37.125MHz。
    // VC 配置为：
    // PAD0 → VC2（短曝光）
    // PAD1 → VC1（中曝光）
    // PAD2 → VC0（长曝光）
    // PAD3 → VC2（短曝光）
    // VTS 设为 0x06BD * 4，以适配三曝光时序。
    // 适用于高动态范围需求，如强逆光下的车牌识别。
	{
		.bus_fmt = MEDIA_BUS_FMT_SGBRG10_1X10,
		.width = 3864,
		.height = 2192,
		.max_fps = {
			.numerator = 10000,
			.denominator = 200000,
		},
		.exp_def = 0x13e,
		.hts_def = 0x021A * IMX415_4LANES * 2,
		/*
		 * IMX415 HDR mode T-line is half of Linear mode,
		 * make vts double to workaround.
		 */
		.vts_def = 0x06BD * 4,
		.global_reg_list = imx415_global_10bit_3864x2192_regs,
		.reg_list = imx415_hdr3_10bit_3864x2192_1485M_regs,
		.hdr_mode = HDR_X3,
		.mipi_freq_idx = 2,
		.bpp = 10,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_2,
		.vc[PAD1] = V4L2_MBUS_CSI2_CHANNEL_1,//M->csi wr0
		.vc[PAD2] = V4L2_MBUS_CSI2_CHANNEL_0,//L->csi wr0
		.vc[PAD3] = V4L2_MBUS_CSI2_CHANNEL_2,//S->csi wr2
		.xvclk = IMX415_XVCLK_FREQ_37M,
	},

    // 第5个模式
    // 分辨率为 3864×2192。
    // 使用 10-bit 输出格式。
    // 最大帧率为 20fps。
    // 工作在 HDR_X3 模式。
    // MIPI 频率索引为 3，对应 891 MHz（更高带宽）。
    // 使用的寄存器表为 imx415_hdr3_10bit_3864x2192_1782M_regs。
    // xvclk 为 37.125MHz。
    // VC 配置与模式 4 相同。
    // VTS 设为 0x07ea * 4。
    // 相比模式 4，此模式使用更高 MIPI 速率，图像质量更稳定，适合对画质要求更高的 HDR 应用。
	{
		.bus_fmt = MEDIA_BUS_FMT_SGBRG10_1X10,
		.width = 3864,
		.height = 2192,
		.max_fps = {
			.numerator = 10000,
			.denominator = 200000,
		},
		.exp_def = 0x13e,
		.hts_def = 0x01ca * IMX415_4LANES * 2,
		/*
		 * IMX415 HDR mode T-line is half of Linear mode,
		 * make vts double to workaround.
		 */
		.vts_def = 0x07ea * 4,
		.global_reg_list = imx415_global_10bit_3864x2192_regs,
		.reg_list = imx415_hdr3_10bit_3864x2192_1782M_regs,
		.hdr_mode = HDR_X3,
		.mipi_freq_idx = 3,
		.bpp = 10,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_2,
		.vc[PAD1] = V4L2_MBUS_CSI2_CHANNEL_1,//M->csi wr0
		.vc[PAD2] = V4L2_MBUS_CSI2_CHANNEL_0,//L->csi wr0
		.vc[PAD3] = V4L2_MBUS_CSI2_CHANNEL_2,//S->csi wr2
		.xvclk = IMX415_XVCLK_FREQ_37M,
	},

    // 第6个模式
    // 分辨率为 3864×2192。
    // 使用 12-bit 输出格式（MEDIA_BUS_FMT_SGBRG12_1X12）。
    // 最大帧率为 30fps。
    // 工作在 Linear 模式。
    // MIPI 频率索引为 1，对应 446 MHz。
    // 使用的寄存器表为 imx415_linear_12bit_3864x2192_891M_regs。
    // xvclk 为 37.125MHz。
    // VC 为 V4L2_MBUS_CSI2_CHANNEL_0。
    // 此模式提供更高精度的原始图像数据，适合工业检测或专业摄影。
	{
		/* 1H period = (1100 clock) = (1100 * 1 / 74.25MHz) */
		.bus_fmt = MEDIA_BUS_FMT_SGBRG12_1X12,
		.width = 3864,
		.height = 2192,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x08ca - 0x08,
		.hts_def = 0x044c * IMX415_4LANES * 2,
		.vts_def = 0x08ca,
		.global_reg_list = imx415_global_12bit_3864x2192_regs,
		.reg_list = imx415_linear_12bit_3864x2192_891M_regs,
		.hdr_mode = NO_HDR,
		.mipi_freq_idx = 1,
		.bpp = 12,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
		.xvclk = IMX415_XVCLK_FREQ_37M,
	},

    // 第7个模式
    // 分辨率为 3864×2192。
    // 使用 12-bit 输出格式。
    // 最大帧率为 30fps。
    // 工作在 HDR_X2 模式。
    // MIPI 频率索引为 3，对应 891 MHz。
    // 使用的寄存器表为 imx415_hdr2_12bit_3864x2192_1782M_regs。
    // xvclk 为 37.125MHz。
    // VC 配置为：
    // PAD0 → VC1（短曝光）
    // PAD1 → VC0（长曝光）
    // PAD2 → VC1
    // PAD3 → VC1
    // VTS 设为 0x08CA * 2。
    // 这是 12-bit 双曝光 HDR 模式，兼顾画质与动态范围。
	{
		.bus_fmt = MEDIA_BUS_FMT_SGBRG12_1X12,
		.width = 3864,
		.height = 2192,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x08CA * 2 - 0x0d90,
		.hts_def = 0x0226 * IMX415_4LANES * 2,
		/*
		 * IMX415 HDR mode T-line is half of Linear mode,
		 * make vts double(that is FSC) to workaround.
		 */
		.vts_def = 0x08CA * 2,
		.global_reg_list = imx415_global_12bit_3864x2192_regs,
		.reg_list = imx415_hdr2_12bit_3864x2192_1782M_regs,
		.hdr_mode = HDR_X2,
		.mipi_freq_idx = 3,
		.bpp = 12,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_1,
		.vc[PAD1] = V4L2_MBUS_CSI2_CHANNEL_0,//L->csi wr0
		.vc[PAD2] = V4L2_MBUS_CSI2_CHANNEL_1,
		.vc[PAD3] = V4L2_MBUS_CSI2_CHANNEL_1,//M->csi wr2
		.xvclk = IMX415_XVCLK_FREQ_37M,
	},

    // 第8个模式
    // 分辨率为 3864×2192。
    // 使用 12-bit 输出格式。
    // 最大帧率为 20fps。
    // 工作在 HDR_X3 模式。
    // MIPI 频率索引为 3，对应 891 MHz。
    // 使用的寄存器表为 imx415_hdr3_12bit_3864x2192_1782M_regs。
    // xvclk 为 37.125MHz。
    // VC 配置为：
    // PAD0 → VC2（短）
    // PAD1 → VC1（中）
    // PAD2 → VC0（长）
    // PAD3 → VC2（短）
    // VTS 设为 0x0696 * 4。
    // 这是 最高画质 HDR 模式，支持 12-bit 三曝光，适用于高端安防或车载前视系统。
	{
		.bus_fmt = MEDIA_BUS_FMT_SGBRG12_1X12,
		.width = 3864,
		.height = 2192,
		.max_fps = {
			.numerator = 10000,
			.denominator = 200000,
		},
		.exp_def = 0x114,
		.hts_def = 0x0226 * IMX415_4LANES * 2,
		/*
		 * IMX415 HDR mode T-line is half of Linear mode,
		 * make vts double(that is FSC) to workaround.
		 */
		.vts_def = 0x0696 * 4,
		.global_reg_list = imx415_global_12bit_3864x2192_regs,
		.reg_list = imx415_hdr3_12bit_3864x2192_1782M_regs,
		.hdr_mode = HDR_X3,
		.mipi_freq_idx = 3,
		.bpp = 12,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_2,
		.vc[PAD1] = V4L2_MBUS_CSI2_CHANNEL_1,//M->csi wr0
		.vc[PAD2] = V4L2_MBUS_CSI2_CHANNEL_0,//L->csi wr0
		.vc[PAD3] = V4L2_MBUS_CSI2_CHANNEL_2,//S->csi wr2
		.xvclk = IMX415_XVCLK_FREQ_37M,
	},

    // 第9个模式
    // 分辨率为 1944×1097（通过 2×2 binning 合并像素）。
    // 使用 12-bit 输出格式。
    // 最大帧率为 30fps。
    // 工作在 Linear 模式。
    // MIPI 频率索引为 0，对应 297 MHz（最低带宽）。
    // 使用的寄存器表为 imx415_linear_12bit_1932x1096_594M_regs。
    // xvclk 为 37.125MHz。
    // VC 为 V4L2_MBUS_CSI2_CHANNEL_0。
    // 此模式灵敏度高、带宽低，适合低光环境或资源受限的嵌入式设备。
	{
		.bus_fmt = MEDIA_BUS_FMT_SGBRG12_1X12,
		.width = 1944,
		.height = 1097,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x05dc - 0x08,
		.hts_def = 0x030e * 3,
		.vts_def = 0x0c5d,
		.global_reg_list = imx415_global_12bit_3864x2192_regs,
		.reg_list = imx415_linear_12bit_1932x1096_594M_regs,
		.hdr_mode = NO_HDR,
		.mipi_freq_idx = 0,
		.bpp = 12,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
		.xvclk = IMX415_XVCLK_FREQ_37M,
	},

    // 第10个模式
    // 分辨率为 1944×1097。
    // 使用 12-bit 输出格式。
    // 最大帧率为 30fps。
    // 工作在 HDR_X2 模式。
    // MIPI 频率索引为 1，对应 446 MHz。
    // 使用的寄存器表为 imx415_hdr2_12bit_1932x1096_891M_regs。
    // xvclk 为 37.125MHz。
    // VC 配置为：
    // PAD0 → VC1（短曝光）
    // PAD1 → VC0（长曝光）
    // PAD2 → VC1
    // PAD3 → VC1
    // VTS 设为 0x08FC * 2。
    // 这是 Binning + HDR 组合模式，常用于车载环视或机器人避障系统，在保持低带宽的同时获得良好动态范围。
	{
		.bus_fmt = MEDIA_BUS_FMT_SGBRG12_1X12,
		.width = 1944,
		.height = 1097,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x08FC / 4,
		.hts_def = 0x021A * 4,
		/*
		 * IMX415 HDR mode T-line is half of Linear mode,
		 * make vts double(that is FSC) to workaround.
		 */
		.vts_def = 0x08FC * 2,
		.global_reg_list = imx415_global_12bit_3864x2192_regs,
		.reg_list = imx415_hdr2_12bit_1932x1096_891M_regs,
		.hdr_mode = HDR_X2,
		.mipi_freq_idx = 1,
		.bpp = 12,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_1,
		.vc[PAD1] = V4L2_MBUS_CSI2_CHANNEL_0,//L->csi wr0
		.vc[PAD2] = V4L2_MBUS_CSI2_CHANNEL_1,
		.vc[PAD3] = V4L2_MBUS_CSI2_CHANNEL_1,//M->csi wr2
		.xvclk = IMX415_XVCLK_FREQ_37M,
	},
};

// imx415在2lane条件下支持的模式
static const struct imx415_mode supported_modes_2lane[] = {
	// 第1个模式
    // 分辨率：3864 × 2192（全像素输出，即 Full Pixel）
    // 位深：12-bit（MEDIA_BUS_FMT_SGBRG12_1X12），提供高动态范围和图像精度
    // 帧率：15 fps（由 .max_fps = {10000, 150000} 计算得出：150000 ÷ 10000 = 15）
    // HDR 模式：NO_HDR（线性模式，单次曝光）
    // MIPI 配置：
    // 使用 2 条数据通道（2-lane）
    // mipi_freq_idx = 1 → 对应 446 MHz（来自 link_freq_items[1]）
    // 实际总带宽 ≈ 446 MHz × 2（DDR）× 2（lane） = 1784 Mbps
    // 外部时钟：xvclk = 27 MHz（注意：不是 37.125MHz，这是 2-lane 模式的典型配置）
    // 寄存器表：imx415_linear_12bit_3864x2192_891M_regs_2lane
    // 包含针对 2-lane 和 27MHz xvclk 优化的时序参数
    // 启动后有 {REG_DELAY, 0x1E}（即延时 30ms）确保稳定
    {
		/* 1H period = (1100 clock) = (1100 * 1 / 74.25MHz) */
		.bus_fmt = MEDIA_BUS_FMT_SGBRG12_1X12,
		.width = 3864,
		.height = 2192,
		.max_fps = {
			.numerator = 10000,
			.denominator = 150000,
		},
		.exp_def = 0x08ca - 0x08,
		.hts_def = 0x0898 * IMX415_2LANES * 2,
		.vts_def = 0x08ca,
		.global_reg_list = NULL, // 2-lane这里没有global的配置表
		.reg_list = imx415_linear_12bit_3864x2192_891M_regs_2lane,
		.hdr_mode = NO_HDR,
		.mipi_freq_idx = 1,
		.bpp = 12,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
		.xvclk = IMX415_XVCLK_FREQ_27M,
	},
    
    // 第2个模式
    // 分辨率：1284 × 720（非标准 720p，但接近）
    // 实际传感器窗口为 2568×1440，通过 2×2 binning + 裁剪 得到 1284×720
    // 符合 ISP 输入对齐要求（宽 16 对齐，高 8 对齐）
    // 位深：12-bit（高精度 RAW 输出）
    // 帧率：90 fps（.max_fps = {10000, 900000} → 900000 ÷ 10000 = 90）
    // HDR 模式：NO_HDR（线性）
    // MIPI 配置：
    // 2-lane
    // mipi_freq_idx = 4 → 对应 1188 MHz（最高带宽）
    // 总带宽 ≈ 1188 MHz × 2 × 2 = 4752 Mbps（即 2376 Mbps per lane）
    // 外部时钟：xvclk = 27 MHz
    // 寄存器表：imx415_linear_12bit_1284x720_2376M_regs_2lane
    // 配置了窗口裁剪寄存器（如 0x3040~0x3047 设置 ROI）
    // 同样包含 30ms 启动延时
	{
		/* 1H period = (1100 clock) = (1100 * 1 / 74.25MHz) */
		.bus_fmt = MEDIA_BUS_FMT_SGBRG12_1X12,
		.width = 1284,
		.height = 720,
		.max_fps = {
			.numerator = 10000,
			.denominator = 900000,
		},
		.exp_def = 0x07AB-8,
		.hts_def = 0x01A4 * IMX415_2LANES * 2,
		.vts_def = 0x07AB,
		.global_reg_list = NULL,
		.reg_list = imx415_linear_12bit_1284x720_2376M_regs_2lane, 
		.hdr_mode = NO_HDR,
		.mipi_freq_idx = 4,
		.bpp = 12,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
		.xvclk = IMX415_XVCLK_FREQ_27M,
	},
};

// 定义 MIPI CSI-2 接口支持的 链路频率（Link Frequency）列表。
// 用途：
// V4L2 框架通过 V4L2_CID_LINK_FREQ 控制项上报当前使用的频率索引。
// 例如：mipi_freq_idx = 3 → 对应 MIPI_FREQ_891M = 891,000,000 Hz。
// 为什么需要？
// ISP（图像信号处理器）需要知道传感器输出带宽，以配置接收端 PHY 和缓冲区。
static const s64 link_freq_items[] = {
	MIPI_FREQ_297M,
	MIPI_FREQ_446M,
	MIPI_FREQ_743M,
	MIPI_FREQ_891M,
	MIPI_FREQ_1188M,
};

/* Write registers up to 4 at a time */
// 通过 I2C 向传感器写入一个寄存器值（支持 1~4 字节）
// client：I2C 设备句柄
// reg：寄存器地址（16 位）
// len：要写入的字节数（1~4）
// val：要写入的值（32 位，高位补 0）
static int imx415_write_reg(struct i2c_client *client, u16 reg, u32 len, u32 val)
{
	u32 buf_i, val_i;
	u8 buf[6];
	u8 *val_p;
	__be32 val_be; // 声明一个大端格式的32位变量

    // 安全检查：IMX415 最大寄存器宽度为 4 字节。
	if (len > 4)
		return -EINVAL;

    // 将 16 位寄存器地址拆分为高 8 位 + 低 8 位，放入 I2C 发送缓冲区前两字节。
	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

    // 将 val 转为 大端序（Big Endian），因为 I2C 传输是 MSB 先发。
	val_be = cpu_to_be32(val);
	val_p = (u8 *)&val_be;

    // 技巧：只取 val 的低 len 字节，并按 I2C 顺序填入 buf[2..]。
    // 例如：len=1, val=0xAB → buf = [regH, regL, 0xAB]
	buf_i = 2;
	val_i = 4 - len;
	while (val_i < 4)
		buf[buf_i++] = val_p[val_i++];

    // 调用内核 I2C API 发送数据（地址 2 字节 + 数据 len 字节）。
    // 若发送字节数不匹配，返回 I/O 错误。
	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

// 批量写入寄存器数组，写寄存器配置表，按顺序写入一组寄存器（如启动流时的完整配置）
// 驱动加载寄存器配置的核心函数，支持“写寄存器 + 延时”混合操作。
static int imx415_write_array(struct i2c_client *client, const struct regval *regs)
{
	u32 i;
	int ret = 0;

    // 空指针检查，避免崩溃。
	if (!regs) {
		dev_err(&client->dev, "write reg array error\n");
		return ret;
	}

    // 循环直到遇到 REG_NULL（配置表结束标志）或发生错误。
	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++) {
        // 特殊处理延时：若地址为 REG_DELAY（0xFFFE），则休眠 val 毫秒。
        // 用于满足传感器上电/模式切换的时序要求（如“写完后等待 30ms”）。
		if (regs[i].addr == REG_DELAY) {
			usleep_range(regs[i].val * 1000, regs[i].val * 1000 + 500);
			dev_info(&client->dev, "write reg array, sleep %dms\n", regs[i].val);
		}
        // 否则，调用 imx415_write_reg() 写入 8 位寄存器值。
        else {
			ret = imx415_write_reg(client, regs[i].addr, IMX415_REG_VALUE_08BIT, regs[i].val);
		}
	}
    // 返回最终状态（0=成功，负值=错误码）
	return ret;
}

/* Read registers up to 4 at a time */
// 通过 I2C 从传感器读取寄存器值（1~4 字节）
// reg：寄存器地址
// len：要读取的字节数
// val：输出指针，存放读取结果
static int imx415_read_reg(struct i2c_client *client, u16 reg, unsigned int len, u32 *val)
{
    // 创建I2C复合消息（先写地址，再读数据）变量，msgs。
	struct i2c_msg msgs[2];
	u8 *data_be_p;
	__be32 data_be = 0; // 声明一个大端格式的32位变量并初始化为0
	__be16 reg_addr_be = cpu_to_be16(reg); // 将16位寄存器的值转换为大端格式
	int ret;

    // 安全检查：长度必须为 1~4。
	if (len > 4 || !len)
		return -EINVAL;

    // 寄存器地址转大端序。
	data_be_p = (u8 *)&data_be;

	/* Write register address */
    // 第一条消息：发送 2 字节寄存器地址。
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = (u8 *)&reg_addr_be;

	/* Read data from register */
    // 第二条消息：读取 len 字节数据。
    // &data_be_p[4 - len]：确保数据对齐到 32 位变量的低位（如读 1 字节 → 存入 data_be_p[3]）。
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_be_p[4 - len];

    // 执行 I2C 事务，必须两条消息都成功。
	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

    // 将大端序数据转回主机字节序，存入 *val。
	*val = be32_to_cpu(data_be);

	return 0;
}

// 计算分辨率差异，计算两个分辨率之间的 曼哈顿距离。
// 在多个支持模式中，找到最接近用户请求分辨率的那个
static int imx415_get_reso_dist(const struct imx415_mode *mode, struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

// 查找最佳匹配模式,从 supported_modes[] 中选出最匹配用户请求的模式。
// 参数：fmt 是用户通过 VIDIOC_S_FMT 请求的格式。
static const struct imx415_mode * imx415_find_best_fit(struct imx415 *imx415, struct v4l2_subdev_format *fmt)
{
    // 初始化变量：cur_best_fit 保存最佳模式索引。
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

    // 遍历所有支持的模式：
    // 仅当 总线格式（bus_fmt）匹配 时才考虑（如都是 SGBRG10_1X10）。
    // 选择 分辨率差异最小 的模式。
	for (i = 0; i < imx415->cfg_num; i++) {
		dist = imx415_get_reso_dist(&imx415->supported_modes[i], framefmt);
		if ((cur_best_fit_dist == -1 || dist < cur_best_fit_dist) &&
			imx415->supported_modes[i].bus_fmt == framefmt->code) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}
    // 打印日志并返回最佳匹配的 imx415_mode 指针。
	dev_info(&imx415->client->dev, "%s: cur_best_fit(%d)", __func__, cur_best_fit);

	return &imx415->supported_modes[cur_best_fit];
}

// 前向声明：__imx415_power_on 函数在后面定义，此处提前声明以便调用。
static int __imx415_power_on(struct imx415 *imx415);

// 切换工作模式
static void imx415_change_mode(struct imx415 *imx415, const struct imx415_mode *mode)
{
    // 如果启用了 Thunderboot 模式，且 ISP 状态为“未就绪”（NG），则手动上电传感器。
    // 这是为了支持安防/车载场景的“秒级开机”。
	if (imx415->is_thunderboot && rkisp_tb_get_state() == RKISP_TB_NG) {
		imx415->is_thunderboot = false;
		imx415->is_thunderboot_ng = true;
		__imx415_power_on(imx415);
	}

    // 更新当前模式和 VTS（垂直总行数），用于后续曝光控制。
	imx415->cur_mode = mode;
	imx415->cur_vts = imx415->cur_mode->vts_def;

    // 打印日志：记录当前切换的分辨率、HDR 模式、位深。
	dev_info(&imx415->client->dev, "set fmt: cur_mode: %dx%d, hdr: %d, bpp: %d\n", mode->width, mode->height, mode->hdr_mode, mode->bpp);
}

// 设置输出格式（V4L2 接口）
static int imx415_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
    // 获取主结构体 imx415、MIPI 通道数 lanes。
	struct imx415 *imx415 = to_imx415(sd);
	const struct imx415_mode *mode;
	s64 h_blank, vblank_def, vblank_min;
	u64 pixel_rate = 0;
	u8 lanes = imx415->bus_cfg.bus.mipi_csi2.num_data_lanes;

    // 加锁：防止并发访问导致状态不一致。
	mutex_lock(&imx415->mutex);

    // 调用 imx415_find_best_fit() 从 supported_modes[] 中找到最匹配的模式。
    // 将用户请求的格式强制对齐到硬件支持的格式。
	mode = imx415_find_best_fit(imx415, fmt);
	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;

    // TRY 模式：仅用于查询，不实际生效（常用于应用预览）。
    // 将格式暂存到 pad_config 中，不修改硬件状态。
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&imx415->mutex);
		return -ENOTTY;
#endif
	}
    // ACTIVE 模式：真正切换到该模式。
    else {
		imx415_change_mode(imx415, mode);

        // 计算水平消隐（HBLANK = HTS - width），并锁定 HBLANK 控制项为只读固定值。
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(imx415->hblank, h_blank, h_blank, 1, h_blank);

        // 计算垂直消隐（VBLANK = VTS - height）。
		vblank_def = mode->vts_def - mode->height;
        
		/* VMAX >= (PIX_VWIDTH / 2) + 46 = height + 46 */
        // 设置 VBLANK 的最小值（由 Sony datasheet 规定：VTS ≥ height + 46）。
		vblank_min = (mode->height + 46) - mode->height;

        // 默认设为 vblank_def，但允许用户动态调整以改变帧率。
		__v4l2_ctrl_modify_range(imx415->vblank, vblank_min,
					 IMX415_VTS_MAX - mode->height,
					 1, vblank_def);
        // __v4l2_ctrl_s_ctrl是Linux 内核 V4L2 框架中的一个核心函数，用于设置（写入）V4L2 控制项的值。
		__v4l2_ctrl_s_ctrl(imx415->vblank, vblank_def);

        // 上报 MIPI 链路频率索引和像素率（Pixel Rate）给 ISP，用于带宽分配。
		__v4l2_ctrl_s_ctrl(imx415->link_freq, mode->mipi_freq_idx);
		pixel_rate = (u32)link_freq_items[mode->mipi_freq_idx] / mode->bpp * 2 * lanes;
		__v4l2_ctrl_s_ctrl_int64(imx415->pixel_rate, pixel_rate);
	}

    // 打印日志并解锁。
	dev_info(&imx415->client->dev, "%s: mode->mipi_freq_idx(%d)", __func__, mode->mipi_freq_idx);

	mutex_unlock(&imx415->mutex);

	return 0;
}

// 获取当前激活的模式，让上层知道当前输出的分辨率、格式、以及 HDR 模式下的 VC 映射。
static int imx415_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct imx415 *imx415 = to_imx415(sd);
	const struct imx415_mode *mode = imx415->cur_mode;

	mutex_lock(&imx415->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&imx415->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = mode->bus_fmt;
		fmt->format.field = V4L2_FIELD_NONE;
		if (fmt->pad < PAD_MAX && mode->hdr_mode != NO_HDR)
			fmt->reserved[0] = mode->vc[fmt->pad];
		else
			fmt->reserved[0] = mode->vc[PAD0];
	}
	mutex_unlock(&imx415->mutex);

	return 0;
}

// 枚举支持的总线格式
// 按索引返回第 index 个模式的 bus_fmt（如 SGBRG10_1X10 或 SGBRG12_1X12）。
// 应用可通过循环调用此函数，获取所有支持的格式。
static int imx415_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct imx415 *imx415 = to_imx415(sd);

	if (code->index >= imx415->cfg_num)
		return -EINVAL;

	code->code = imx415->supported_modes[code->index].bus_fmt;

	return 0;
}

// 枚举支持的分辨率
// 返回第 index 个模式的固定分辨率（无范围，因为每个模式分辨率固定）。
// 同时校验 bus_fmt 是否匹配。
static int imx415_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct imx415 *imx415 = to_imx415(sd);

	if (fse->index >= imx415->cfg_num)
		return -EINVAL;

	if (fse->code != imx415->supported_modes[fse->index].bus_fmt)
		return -EINVAL;

	fse->min_width  = imx415->supported_modes[fse->index].width;
	fse->max_width  = imx415->supported_modes[fse->index].width;
	fse->max_height = imx415->supported_modes[fse->index].height;
	fse->min_height = imx415->supported_modes[fse->index].height;

	return 0;
}

// 返回当前模式的最大帧率（以分数形式表示）。
static int imx415_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct imx415 *imx415 = to_imx415(sd);
	const struct imx415_mode *mode = imx415->cur_mode;

	fi->interval = mode->max_fps;

	return 0;
}

// 获取总线配置
// 作用：告诉 ISP 当前 MIPI 配置：
// 使用多少条 lane（通过 1 << (lanes-1) 编码）
// 启用哪些 Virtual Channel（VC0 必选，HDR_X2 加 VC1，HDR_X3 加 VC2）
// 使用连续时钟（CONTINUOUS_CLOCK）
static int imx415_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad_id,
				struct v4l2_mbus_config *config)
{
	struct imx415 *imx415 = to_imx415(sd);
	const struct imx415_mode *mode = imx415->cur_mode;
	u32 val = 0;
	u8 lanes = imx415->bus_cfg.bus.mipi_csi2.num_data_lanes;

	val = 1 << (lanes - 1) |
	      V4L2_MBUS_CSI2_CHANNEL_0 |
	      V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	if (mode->hdr_mode != NO_HDR)
		val |= V4L2_MBUS_CSI2_CHANNEL_1;
	if (mode->hdr_mode == HDR_X3)
		val |= V4L2_MBUS_CSI2_CHANNEL_2;
	config->type = V4L2_MBUS_CSI2_DPHY;
	config->flags = val;

	return 0;
}

// 填充传感器型号、模组名、镜头型号（从设备树解析而来），用于上层识别摄像头硬件配置。
static void imx415_get_module_inf(struct imx415 *imx415, struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, IMX415_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module, imx415->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, imx415->len_name, sizeof(inf->base.lens));
}

// HDR DOL3（三曝光）模式的核心控制逻辑，负责根据上层（如 ISP 或 AIQ 算法）传入的长/中/短三路曝光参数，计算并写入符合 Sony 严格时序约束的寄存器值。
// 在 单帧内 安排三个曝光：
// Long（L）：长曝光（暗部细节）
// Middle（M）：中曝光（主体）
// Short（S）：短曝光（亮部细节）
// 必须满足 Sony datasheet 的 数学约束（如 RHS1 = 6n+1），否则图像撕裂或曝光错误。
static int imx415_set_hdrae_3frame(struct imx415 *imx415, struct preisp_hdrae_exp_s *ae)
{
	struct i2c_client *client = imx415->client;
	u32 l_exp_time, m_exp_time, s_exp_time; // 三路曝光时间（行数）
	u32 l_a_gain, m_a_gain, s_a_gain;      // 三路模拟增益
	int shr2, shr1, shr0 = 0;  // 三路曝光起始行（SHR = Start of Horizontal Readout），每路曝光从哪一行开始
    int rhs2, rhs1 = 0; // 两路曝光结束行（RHS = End of Readout for Short exposure），短/中曝光在哪一行结束
	int rhs1_change_limit, rhs2_change_limit = 0;
	static int rhs1_old = IMX415_RHS1_DEFAULT; // 上一帧 RHS1 值（用于动态调整限制）
	static int rhs2_old = IMX415_RHS2_DEFAULT; // 上一帧 RHS2 值
	int ret = 0; // 返回值
	u32 fsc; // Frame Stabilization Cycle（完整帧周期（HDR 模式下 FSC = 4 × VTS））
	int rhs1_max = 0;
	int shr2_min = 0;

    // 非流状态处理（保存参数）
    // 如果传感器尚未开始流传输，先保存曝光参数，等启动流时再应用。
    // 避免在未上电时写寄存器导致错误。
	if (!imx415->has_init_exp && !imx415->streaming) {
		imx415->init_hdrae_exp = *ae;
		imx415->has_init_exp = true;
		dev_dbg(&imx415->client->dev, "imx415 is not streaming, save hdr ae!\n");
		return ret;
	}

    // 从 ae 结构体获取三路曝光时间和增益。自动曝光（Auto Exposure）相关的结构体指针
	l_exp_time = ae->long_exp_reg;
	m_exp_time = ae->middle_exp_reg;
	s_exp_time = ae->short_exp_reg;
	l_a_gain = ae->long_gain_reg;
	m_a_gain = ae->middle_gain_reg;
	s_a_gain = ae->short_gain_reg;

    // 打印调试日志。
	dev_dbg(&client->dev,
		"rev exp req: L_exp: 0x%x, 0x%x, M_exp: 0x%x, 0x%x S_exp: 0x%x, 0x%x\n",
		l_exp_time, m_exp_time, s_exp_time,
		l_a_gain, m_a_gain, s_a_gain);

    // 启用 Group Hold（原子更新）
    // 写入 0x3001 = 0x01，锁定寄存器，防止中间状态生效。
    // 所有后续寄存器写入会暂存，直到 GROUP_HOLD_END 才一次性生效。
	ret = imx415_write_reg(client, IMX415_GROUP_HOLD_REG,
		IMX415_REG_VALUE_08BIT, IMX415_GROUP_HOLD_START);

	/* gain effect n+1 */
    // 增益生效于下一帧
    // 写入LF/SF1/SF2三路增益，将 16 位增益值拆分为高/低 8 位，写入对应寄存器。
	ret |= imx415_write_reg(client, IMX415_LF_GAIN_REG_H,
		IMX415_REG_VALUE_08BIT, IMX415_FETCH_GAIN_H(l_a_gain));
	ret |= imx415_write_reg(client, IMX415_LF_GAIN_REG_L,
		IMX415_REG_VALUE_08BIT, IMX415_FETCH_GAIN_L(l_a_gain));

	ret |= imx415_write_reg(client, IMX415_SF1_GAIN_REG_H,
		IMX415_REG_VALUE_08BIT, IMX415_FETCH_GAIN_H(m_a_gain));
	ret |= imx415_write_reg(client, IMX415_SF1_GAIN_REG_L,
		IMX415_REG_VALUE_08BIT, IMX415_FETCH_GAIN_L(m_a_gain));

	ret |= imx415_write_reg(client, IMX415_SF2_GAIN_REG_H,
		IMX415_REG_VALUE_08BIT, IMX415_FETCH_GAIN_H(s_a_gain));
	ret |= imx415_write_reg(client, IMX415_SF2_GAIN_REG_L,
		IMX415_REG_VALUE_08BIT, IMX415_FETCH_GAIN_L(s_a_gain));

	/* Restrictions
	 *   FSC = 4 * VMAX and FSC should be 6n;
	 *   exp_l = FSC - SHR0 + Toffset;
	 *
	 *   SHR0 = FSC - exp_l + Toffset;
	 *   SHR0 <= (FSC -12);
	 *   SHR0 >= RHS2 + 13;
	 *   SHR0 should be 3n;
	 *
	 *   exp_m = RHS1 - SHR1 + Toffset;
	 *
	 *   RHS1 < BRL * 3;
	 *   RHS1 <= SHR2 - 13;
	 *   RHS1 >= SHR1 + 12;
	 *   SHR1 >= 13;
	 *   SHR1 <= RHS1 - 12;
	 *   RHS1(n+1) >= RHS1(n) + BRL * 3 -FSC + 3;
	 *
	 *   SHR1 should be 3n+1 and RHS1 should be 6n+1;
	 *
	 *   exp_s = RHS2 - SHR2 + Toffset;
	 *
	 *   RHS2 < BRL * 3 + RHS1;
	 *   RHS2 <= SHR0 - 13;
	 *   RHS2 >= SHR2 + 12;
	 *   SHR2 >= RHS1 + 13;
	 *   SHR2 <= RHS2 - 12;
	 *   RHS1(n+1) >= RHS1(n) + BRL * 3 -FSC + 3;
	 *
	 *   SHR2 should be 3n+2 and RHS2 should be 6n+2;
	 */

	/* The HDR mode vts is double by default to workaround T-line */
    // 核心：计算 SHR0（长曝光起始行），约束：SHR0 ≤ FSC - 12 且 SHR0 ≥ RHS2 + 13
	fsc = imx415->cur_vts; // 当前 VTS
	fsc = fsc / 6 * 6;     // 强制 FSC 为 6 的倍数（Sony 要求）
	shr0 = fsc - l_exp_time; // 长曝光起始行 = FSC - 曝光时间
	dev_dbg(&client->dev,
		"line(%d) shr0 %d, l_exp_time %d, fsc %d\n",
		__LINE__, shr0, l_exp_time, fsc);

    // 核心约束：RHS1 必须 = 6n + 1（如 25, 31, 37...）。(x + 5)/6*6 + 1 是向上取整到 6n+1 的技巧。
	rhs1 = (SHR1_MIN_X3 + m_exp_time + 5) / 6 * 6 + 1;
	
    // 限制 RHS1 在合法范围内（25 ~ rhs1_max）。
    if (imx415->cur_mode->height == 2192)
		rhs1_max = RHS1_MAX_X3(BRL_ALL);  // 全分辨率上限
	else
		rhs1_max = RHS1_MAX_X3(BRL_BINNING); // Binning 模式上限
	if (rhs1 < 25)
		rhs1 = 25;
	else if (rhs1 > rhs1_max)
		rhs1 = rhs1_max;
	dev_dbg(&client->dev,
		"line(%d) rhs1 %d, m_exp_time %d rhs1_old %d\n",
		__LINE__, rhs1, m_exp_time, rhs1_old);

	//Dynamic adjustment rhs2 must meet the following conditions
    // 动态调整限制（防止跳变）,Sony 规定：RHS1(n+1) ≥ RHS1(n) + 3×BRL - FSC + 3，防止 RHS1 剧烈跳变导致图像闪烁。
	if (imx415->cur_mode->height == 2192)
		rhs1_change_limit = rhs1_old + 3 * BRL_ALL - fsc + 3;
	else
		rhs1_change_limit = rhs1_old + 3 * BRL_BINNING - fsc + 3;
	rhs1_change_limit = (rhs1_change_limit < 25) ? 25 : rhs1_change_limit;
	rhs1_change_limit = (rhs1_change_limit + 5) / 6 * 6 + 1;  // 仍满足 6n+1
	if (rhs1_max < rhs1_change_limit) {
		dev_err(&client->dev,
			"The total exposure limit makes rhs1 max is %d,but old rhs1 limit makes rhs1 min is %d\n",
			rhs1_max, rhs1_change_limit);
		return -EINVAL;
	}
	if (rhs1 < rhs1_change_limit)
		rhs1 = rhs1_change_limit;

	dev_dbg(&client->dev,
		"line(%d) m_exp_time %d rhs1_old %d, rhs1_new %d\n",
		__LINE__, m_exp_time, rhs1_old, rhs1);

	rhs1_old = rhs1; // 更新历史值

	/* shr1 = rhs1 - s_exp_time */
    // 计算 SHR1（中曝光起始行），约束：SHR1 ≥ 13 且 SHR1 = 3n+1
	if (rhs1 - m_exp_time <= SHR1_MIN_X3) {
		shr1 = SHR1_MIN_X3;  // 最小值 13
		m_exp_time = rhs1 - shr1; // 反向修正曝光时间
	} else {
		shr1 = rhs1 - m_exp_time;
	}

    // 计算 RHS2（短曝光结束行），核心约束：RHS2 = 6n + 2（如 50, 56, 62...）
	shr2_min = rhs1 + 13;
	rhs2 = (shr2_min + s_exp_time + 5) / 6 * 6 + 2; // RHS2 = 6n+2
	if (rhs2 > (shr0 - 13)) // 不能超过 SHR0
		rhs2 = shr0 - 13;
	else if (rhs2 < 50)
		rhs2 = 50;
	dev_dbg(&client->dev,
		"line(%d) rhs2 %d, s_exp_time %d, rhs2_old %d\n",
		__LINE__, rhs2, s_exp_time, rhs2_old);

	//Dynamic adjustment rhs2 must meet the following conditions
    // RHS2 动态调整限制，类似 RHS1，防止跳变。
	if (imx415->cur_mode->height == 2192)
		rhs2_change_limit = rhs2_old + 3 * BRL_ALL - fsc + 3;
	else
		rhs2_change_limit = rhs2_old + 3 * BRL_BINNING - fsc + 3;
	rhs2_change_limit = (rhs2_change_limit < 50) ?  50 : rhs2_change_limit;
	rhs2_change_limit = (rhs2_change_limit + 5) / 6 * 6 + 2;  // 满足 6n+2
	if ((shr0 - 13) < rhs2_change_limit) {
		dev_err(&client->dev,
			"The total exposure limit makes rhs2 max is %d,but old rhs1 limit makes rhs2 min is %d\n",
			shr0 - 13, rhs2_change_limit);
		return -EINVAL;
	}
	if (rhs2 < rhs2_change_limit)
		rhs2 = rhs2_change_limit;

	rhs2_old = rhs2;

	/* shr2 = rhs2 - s_exp_time */
    // 计算 SHR2（短曝光起始行），约束：SHR2 ≥ RHS1 + 13 且 SHR2 = 3n+2
	if (rhs2 - s_exp_time <= shr2_min) {
		shr2 = shr2_min;
		s_exp_time = rhs2 - shr2;
	} else {
		shr2 = rhs2 - s_exp_time;
	}
	dev_dbg(&client->dev,
		"line(%d) rhs2_new %d, s_exp_time %d shr2 %d, rhs2_change_limit %d\n",
		__LINE__, rhs2, s_exp_time, shr2, rhs2_change_limit);

    // 最终校验 SHR0
	if (shr0 < rhs2 + 13)
		shr0 = rhs2 + 13; // 长曝光必须晚于短曝光结束
	else if (shr0 > fsc - 12)
		shr0 = fsc - 12;

    // 打印最终参数
	dev_dbg(&client->dev,
		"long exposure: l_exp_time=%d, fsc=%d, shr0=%d, l_a_gain=%d\n",
		l_exp_time, fsc, shr0, l_a_gain);
	dev_dbg(&client->dev,
		"middle exposure(SEF1): m_exp_time=%d, rhs1=%d, shr1=%d, m_a_gain=%d\n",
		m_exp_time, rhs1, shr1, m_a_gain);
	dev_dbg(&client->dev,
		"short exposure(SEF2): s_exp_time=%d, rhs2=%d, shr2=%d, s_a_gain=%d\n",
		s_exp_time, rhs2, shr2, s_a_gain);

	/* time effect n+1 */
	/* write SEF2 exposure RHS2 regs*/
    // 写入所有曝光寄存器
    // 按顺序写入 RHS2 → SHR2 → RHS1 → SHR1 → SHR0。
    // 曝光生效于下一帧（time effect n+1）。
	ret |= imx415_write_reg(client,
		IMX415_RHS2_REG_L,
		IMX415_REG_VALUE_08BIT,
		IMX415_FETCH_RHS1_L(rhs2));
	ret |= imx415_write_reg(client,
		IMX415_RHS2_REG_M,
		IMX415_REG_VALUE_08BIT,
		IMX415_FETCH_RHS1_M(rhs2));
	ret |= imx415_write_reg(client,
		IMX415_RHS2_REG_H,
		IMX415_REG_VALUE_08BIT,
		IMX415_FETCH_RHS1_H(rhs2));
	/* write SEF2 exposure SHR2 regs*/
	ret |= imx415_write_reg(client,
		IMX415_SF2_EXPO_REG_L,
		IMX415_REG_VALUE_08BIT,
		IMX415_FETCH_EXP_L(shr2));
	ret |= imx415_write_reg(client,
		IMX415_SF2_EXPO_REG_M,
		IMX415_REG_VALUE_08BIT,
		IMX415_FETCH_EXP_M(shr2));
	ret |= imx415_write_reg(client,
		IMX415_SF2_EXPO_REG_H,
		IMX415_REG_VALUE_08BIT,
		IMX415_FETCH_EXP_H(shr2));
	/* write SEF1 exposure RHS1 regs*/
	ret |= imx415_write_reg(client,
		IMX415_RHS1_REG_L,
		IMX415_REG_VALUE_08BIT,
		IMX415_FETCH_RHS1_L(rhs1));
	ret |= imx415_write_reg(client,
		IMX415_RHS1_REG_M,
		IMX415_REG_VALUE_08BIT,
		IMX415_FETCH_RHS1_M(rhs1));
	ret |= imx415_write_reg(client,
		IMX415_RHS1_REG_H,
		IMX415_REG_VALUE_08BIT,
		IMX415_FETCH_RHS1_H(rhs1));
	/* write SEF1 exposure SHR1 regs*/
	ret |= imx415_write_reg(client,
		IMX415_SF1_EXPO_REG_L,
		IMX415_REG_VALUE_08BIT,
		IMX415_FETCH_EXP_L(shr1));
	ret |= imx415_write_reg(client,
		IMX415_SF1_EXPO_REG_M,
		IMX415_REG_VALUE_08BIT,
		IMX415_FETCH_EXP_M(shr1));
	ret |= imx415_write_reg(client,
		IMX415_SF1_EXPO_REG_H,
		IMX415_REG_VALUE_08BIT,
		IMX415_FETCH_EXP_H(shr1));
	/* write LF exposure SHR0 regs*/
	ret |= imx415_write_reg(client,
		IMX415_LF_EXPO_REG_L,
		IMX415_REG_VALUE_08BIT,
		IMX415_FETCH_EXP_L(shr0));
	ret |= imx415_write_reg(client,
		IMX415_LF_EXPO_REG_M,
		IMX415_REG_VALUE_08BIT,
		IMX415_FETCH_EXP_M(shr0));
	ret |= imx415_write_reg(client,
		IMX415_LF_EXPO_REG_H,
		IMX415_REG_VALUE_08BIT,
		IMX415_FETCH_EXP_H(shr0));

    // 解锁 Group Hold，写入 0x3001 = 0x00，一次性生效所有寄存器，避免中间状态。
	ret |= imx415_write_reg(client, IMX415_GROUP_HOLD_REG,
		IMX415_REG_VALUE_08BIT, IMX415_GROUP_HOLD_END);
	return ret;
}

// HDR DOL2（双曝光）模式，负责根据上层传入的长/短两路曝光参数，计算并写入符合 Sony 严格时序约束的寄存器值
// 在 单帧内 安排两个曝光：Long（L）：长曝光（暗部细节）；Short（S）：短曝光（亮部细节）
// 注意：虽然参数包含 middle_exp_reg，但在 HDR_X2 模式下，Long 实际使用 Middle 的值（见代码特殊处理）。
static int imx415_set_hdrae(struct imx415 *imx415, struct preisp_hdrae_exp_s *ae)
{
	struct i2c_client *client = imx415->client;
	u32 l_exp_time, m_exp_time, s_exp_time; // 三路曝光时间（但 HDR_X2 只用 L/S）
	u32 l_a_gain, m_a_gain, s_a_gain;  // 三路增益
	int shr1, shr0; // 短/长曝光起始行
    int rhs1; // 短曝光结束行
    int rhs1_max, rhs1_min; // RHS1 的合法范围
	static int rhs1_old = IMX415_RHS1_DEFAULT; // 上一帧 RHS1 值
	int ret = 0;
	u32 fsc; // Frame Stabilization Cycle（= 2 × VTS）

    // 非流状态处理（保存参数），如果传感器未启动流，先保存参数，等启动后再应用。
	if (!imx415->has_init_exp && !imx415->streaming) {
		imx415->init_hdrae_exp = *ae;
		imx415->has_init_exp = true;
		dev_dbg(&imx415->client->dev, "imx415 is not streaming, save hdr ae!\n");
		return ret;
	}

    // 从ae处提取曝光与增益参数，自动曝光（Auto Exposure）相关的结构体指针
	l_exp_time = ae->long_exp_reg;
	m_exp_time = ae->middle_exp_reg;
	s_exp_time = ae->short_exp_reg;
	l_a_gain = ae->long_gain_reg;
	m_a_gain = ae->middle_gain_reg;
	s_a_gain = ae->short_gain_reg;
	dev_dbg(&client->dev,
		"rev exp req: L_exp: 0x%x, 0x%x, M_exp: 0x%x, 0x%x S_exp: 0x%x, 0x%x\n",
		l_exp_time, m_exp_time, s_exp_time,
		l_a_gain, m_a_gain, s_a_gain);

    // HDR_X2 模式特殊处理
    // 关键点：在 IMX415 的 HDR_X2 模式中：
    // Long 路实际使用 Middle 的参数
    // Short 路使用 Short 参数
    // 这是 Sony IMX415 的硬件设计特性（非通用规则）。
	if (imx415->cur_mode->hdr_mode == HDR_X2) {
		l_a_gain = m_a_gain;
		l_exp_time = m_exp_time;
	}

    // 启用 Group Hold（原子更新），锁定寄存器，防止中间状态生效。
	ret = imx415_write_reg(client, IMX415_GROUP_HOLD_REG,
		IMX415_REG_VALUE_08BIT, IMX415_GROUP_HOLD_START);

	/* gain effect n+1 */
    // 写入 Long 和 Short 路的模拟增益，LF和SF1，每路2个寄存器。
    // 增益生效于下一帧。
	ret |= imx415_write_reg(client, IMX415_LF_GAIN_REG_H,
		IMX415_REG_VALUE_08BIT, IMX415_FETCH_GAIN_H(l_a_gain));
	ret |= imx415_write_reg(client, IMX415_LF_GAIN_REG_L,
		IMX415_REG_VALUE_08BIT, IMX415_FETCH_GAIN_L(l_a_gain));

	ret |= imx415_write_reg(client, IMX415_SF1_GAIN_REG_H,
		IMX415_REG_VALUE_08BIT, IMX415_FETCH_GAIN_H(s_a_gain));
	ret |= imx415_write_reg(client, IMX415_SF1_GAIN_REG_L,
		IMX415_REG_VALUE_08BIT, IMX415_FETCH_GAIN_L(s_a_gain));

	/* Restrictions
	 *   FSC = 2 * VMAX and FSC should be 4n;
	 *   exp_l = FSC - SHR0 + Toffset;
	 *   exp_l should be even value;
	 *
	 *   SHR0 = FSC - exp_l + Toffset;
	 *   SHR0 <= (FSC -8);
	 *   SHR0 >= RHS1 + 9;
	 *   SHR0 should be 2n;
	 *
	 *   exp_s = RHS1 - SHR1 + Toffset;
	 *   exp_s should be even value;
	 *
	 *   RHS1 < BRL * 2;
	 *   RHS1 <= SHR0 - 9;
	 *   RHS1 >= SHR1 + 8;
	 *   SHR1 >= 9;
	 *   RHS1(n+1) >= RHS1(n) + BRL * 2 -FSC + 2;
	 *
	 *   SHR1 should be 2n+1 and RHS1 should be 4n+1;
	 */

	/* The HDR mode vts is double by default to workaround T-line */
    // 核心：计算 FSC 和 SHR0
	fsc = imx415->cur_vts; // 当前 VTS
	shr0 = fsc - l_exp_time;; // 长曝光起始行

    // rhs1_max 由两个条件决定：
    // RHS1_MAX_X2(BRL)：基于 BRL（Basic Readout Lines）的上限
    // 全分辨率：BRL_ALL = 2228 → RHS1 < 2×2228 = 4456
    // Binning：BRL_BINNING = 1115 → RHS1 < 2230
    // SHR0 - 9：长曝光起始行必须晚于短曝光结束行至少 9 行
    // rhs1_min 由两个条件决定：
    // SHR1_MIN_X2 + 8 = 9 + 8 = 17：短曝光最小长度
    // 动态调整限制：rhs1_old + 2×BRL - FSC + 2（防跳变）
	if (imx415->cur_mode->height == 2192) {
		rhs1_max = min(RHS1_MAX_X2(BRL_ALL), ((shr0 - 9u) / 4 * 4 + 1));
		rhs1_min = max(SHR1_MIN_X2 + 8u, rhs1_old + 2 * BRL_ALL - fsc + 2);
	} else {
		rhs1_max = min(RHS1_MAX_X2(BRL_BINNING), ((shr0 - 9u) / 4 * 4 + 1));
		rhs1_min = max(SHR1_MIN_X2 + 8u, rhs1_old + 2 * BRL_BINNING - fsc + 2);
	}

    // 确保最小值也满足 4n+1 约束。
	rhs1_min = (rhs1_min + 3) / 4 * 4 + 1; // 强制 rhs1_min = 4n+1
	rhs1 = (SHR1_MIN_X2 + s_exp_time + 3) / 4 * 4 + 1; // = 4n+1，核心约束：RHS1 必须 = 4n + 1（如 25, 29, 33...），(x+3)/4*4+1 是向上取整到 4n+1 的技巧。
	
    // 打印日志
    dev_dbg(&client->dev,
		"line(%d) rhs1 %d, rhs1 min %d rhs1 max %d\n",
		__LINE__, rhs1, rhs1_min, rhs1_max);

    // 如果合法范围为空，说明请求的曝光参数无法满足硬件约束，返回错误。
	if (rhs1_max < rhs1_min) {
		dev_err(&client->dev,
			"The total exposure limit makes rhs1 max is %d,but old rhs1 limit makes rhs1 min is %d\n",
			rhs1_max, rhs1_min);
		return -EINVAL;
	}

    // 限制在合法范围内
	rhs1 = clamp(rhs1, rhs1_min, rhs1_max);
	dev_dbg(&client->dev,
		"line(%d) rhs1 %d, short time %d rhs1_old %d, rhs1_new %d\n",
		__LINE__, rhs1, s_exp_time, rhs1_old, rhs1);

    // 用于下一帧的动态调整
	rhs1_old = rhs1;

	/* shr1 = rhs1 - s_exp_time */
    // 计算 SHR1（短曝光起始行），约束：SHR1 ≥ 9 且 SHR1 = 2n+1
	if (rhs1 - s_exp_time <= SHR1_MIN_X2) {
		shr1 = SHR1_MIN_X2; // 最小值 9
		s_exp_time = rhs1 - shr1; // 反向修正曝光时间
	} else {
		shr1 = rhs1 - s_exp_time;
	}

    // 最终校验 SHR0
	if (shr0 < rhs1 + 9)
		shr0 = rhs1 + 9; // 长曝光必须晚于短曝光结束至少 9 行
	else if (shr0 > fsc - 8)
		shr0 = fsc - 8;  // 长曝光不能太晚

    // 打印调试信息
	dev_dbg(&client->dev,
		"fsc=%d,RHS1_MAX=%d,SHR1_MIN=%d,rhs1_max=%d\n",
		fsc, RHS1_MAX_X2(BRL_ALL), SHR1_MIN_X2, rhs1_max);
	dev_dbg(&client->dev,
		"l_exp_time=%d,s_exp_time=%d,shr0=%d,shr1=%d,rhs1=%d,l_a_gain=%d,s_a_gain=%d\n",
		l_exp_time, s_exp_time, shr0, shr1, rhs1, l_a_gain, s_a_gain);

	/* time effect n+2 */
    // 写入曝光寄存器，曝光生效于 n+2 帧
	ret |= imx415_write_reg(client,
		IMX415_RHS1_REG_L,
		IMX415_REG_VALUE_08BIT,
		IMX415_FETCH_RHS1_L(rhs1));
	ret |= imx415_write_reg(client,
		IMX415_RHS1_REG_M,
		IMX415_REG_VALUE_08BIT,
		IMX415_FETCH_RHS1_M(rhs1));
	ret |= imx415_write_reg(client,
		IMX415_RHS1_REG_H,
		IMX415_REG_VALUE_08BIT,
		IMX415_FETCH_RHS1_H(rhs1));

	ret |= imx415_write_reg(client,
		IMX415_SF1_EXPO_REG_L,
		IMX415_REG_VALUE_08BIT,
		IMX415_FETCH_EXP_L(shr1));
	ret |= imx415_write_reg(client,
		IMX415_SF1_EXPO_REG_M,
		IMX415_REG_VALUE_08BIT,
		IMX415_FETCH_EXP_M(shr1));
	ret |= imx415_write_reg(client,
		IMX415_SF1_EXPO_REG_H,
		IMX415_REG_VALUE_08BIT,
		IMX415_FETCH_EXP_H(shr1));
	ret |= imx415_write_reg(client,
		IMX415_LF_EXPO_REG_L,
		IMX415_REG_VALUE_08BIT,
		IMX415_FETCH_EXP_L(shr0));
	ret |= imx415_write_reg(client,
		IMX415_LF_EXPO_REG_M,
		IMX415_REG_VALUE_08BIT,
		IMX415_FETCH_EXP_M(shr0));
	ret |= imx415_write_reg(client,
		IMX415_LF_EXPO_REG_H,
		IMX415_REG_VALUE_08BIT,
		IMX415_FETCH_EXP_H(shr0));

    // 解锁 Group Hold，一次性生效所有寄存器。
	ret |= imx415_write_reg(client, IMX415_GROUP_HOLD_REG,
		IMX415_REG_VALUE_08BIT, IMX415_GROUP_HOLD_END);
	return ret;
}

// 获取 Virtual Channel 信息
// 参数校验：确保 index 在合法范围（PAD0 ~ PAD3）。
// PAD0=0, PAD_MAX=4（通常定义为 4 路输出）。

// 上层（如 ISP 驱动）通过此接口知道“哪个 pad 对应哪路 HDR 数据”
// HDR_X3 模式下：
// PAD0 → VC2（短曝光）
// PAD1 → VC1（中曝光）
// PAD2 → VC0（长曝光）
static int imx415_get_channel_info(struct imx415 *imx415, struct rkmodule_channel_info *ch_info)
{
	if (ch_info->index < PAD0 || ch_info->index >= PAD_MAX)
		return -EINVAL;

    // vc：当前 pad 对应的 Virtual Channel（如 VC0/VC1/VC2）
	ch_info->vc = imx415->cur_mode->vc[ch_info->index];

    // width/height/bus_fmt：当前模式的分辨率和格式
	ch_info->width = imx415->cur_mode->width;
	ch_info->height = imx415->cur_mode->height;
	ch_info->bus_fmt = imx415->cur_mode->bus_fmt;
	return 0;
}

static long imx415_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
    // 从 V4L2 子设备指针获取主结构体 imx415。
    // 获取 MIPI 通道数 lanes（用于计算 pixel_rate）。
	struct imx415 *imx415 = to_imx415(sd);
	struct rkmodule_hdr_cfg *hdr;
	struct rkmodule_channel_info *ch_info;
	u32 i, h, w, stream;
	long ret = 0;
	const struct imx415_mode *mode;
	u64 pixel_rate = 0;
	struct rkmodule_csi_dphy_param *dphy_param;
	u8 lanes = imx415->bus_cfg.bus.mipi_csi2.num_data_lanes;

	switch (cmd) {
    // 设置 HDR 曝光参数，根据当前 HDR 模式，调用对应的曝光设置函数。arg 是 struct preisp_hdrae_exp_s *，包含三路曝光/增益参数。
	case PREISP_CMD_SET_HDRAE_EXP:
		if (imx415->cur_mode->hdr_mode == HDR_X2)
			ret = imx415_set_hdrae(imx415, arg);
		else if (imx415->cur_mode->hdr_mode == HDR_X3)
			ret = imx415_set_hdrae_3frame(imx415, arg);
		break;

    // 获取模组信息，填充传感器型号（"imx415"）、模组名、镜头名（来自设备树）。
	case RKMODULE_GET_MODULE_INFO:
		imx415_get_module_inf(imx415, (struct rkmodule_inf *)arg);
		break;

    // 获取当前 HDR 配置，返回当前 HDR 模式（NO_HDR / HDR_X2 / HDR_X3）。
	case RKMODULE_GET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		hdr->esp.mode = HDR_NORMAL_VC;
		hdr->hdr_mode = imx415->cur_mode->hdr_mode;
		break;
    
    // 动态切换 HDR 模式，查找 相同分辨率 + 目标 HDR 模式 的配置。例如：从 Linear 切换到 HDR_X2，保持 3864×2192 分辨率。

    case RKMODULE_SET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		w = imx415->cur_mode->width;
		h = imx415->cur_mode->height;
		for (i = 0; i < imx415->cfg_num; i++) {
			if (w == imx415->supported_modes[i].width &&
			    h == imx415->supported_modes[i].height &&
			    imx415->supported_modes[i].hdr_mode == hdr->hdr_mode) {
				dev_info(&imx415->client->dev, "set hdr cfg, set mode to %d\n", i);
				imx415_change_mode(imx415, &imx415->supported_modes[i]);
				break;
			}
		}
        // 如果找不到匹配模式，返回错误。
		if (i == imx415->cfg_num) {
			dev_err(&imx415->client->dev,
				"not find hdr mode:%d %dx%d config\n",
				hdr->hdr_mode, w, h);
			ret = -EINVAL;
		} 
        // 关键优化：如果传感器已在 streaming 状态，不重启流，而是直接写入新寄存器表（通过 Group Hold 原子更新）。
        // 实现 HDR/Linear 无缝切换（无黑屏）。
        else {
			mode = imx415->cur_mode;
			if (imx415->streaming) {
				ret = imx415_write_reg(imx415->client, IMX415_GROUP_HOLD_REG,
					IMX415_REG_VALUE_08BIT, IMX415_GROUP_HOLD_START);

				ret |= imx415_write_array(imx415->client, imx415->cur_mode->reg_list);

				ret |= imx415_write_reg(imx415->client, IMX415_GROUP_HOLD_REG,
					IMX415_REG_VALUE_08BIT, IMX415_GROUP_HOLD_END);
				if (ret)
					return ret;
			}
			w = mode->hts_def - imx415->cur_mode->width;
			h = mode->vts_def - mode->height;

            // 同步更新 V4L2 控制项（HBLANK/VBLANK/LINK_FREQ/PIXEL_RATE），确保 ISP 带宽配置正确。
			mutex_lock(&imx415->mutex);
			__v4l2_ctrl_modify_range(imx415->hblank, w, w, 1, w);
			__v4l2_ctrl_modify_range(imx415->vblank, h,
				IMX415_VTS_MAX - mode->height,
				1, h);
			__v4l2_ctrl_s_ctrl(imx415->link_freq, mode->mipi_freq_idx);
			pixel_rate = (u32)link_freq_items[mode->mipi_freq_idx] /
				mode->bpp * 2 * lanes;
			__v4l2_ctrl_s_ctrl_int64(imx415->pixel_rate,
						 pixel_rate);
			mutex_unlock(&imx415->mutex);
		}
		break;

    // 快速启停流
    // 绕过完整初始化流程，直接写 0x3000 寄存器启动/停止流。
    // 用于 Thunderboot 快速启动 或低延迟场景。
	case RKMODULE_SET_QUICK_STREAM:

		stream = *((u32 *)arg);

		if (stream)
			ret = imx415_write_reg(imx415->client, IMX415_REG_CTRL_MODE,
				IMX415_REG_VALUE_08BIT, IMX415_MODE_STREAMING);
		else
			ret = imx415_write_reg(imx415->client, IMX415_REG_CTRL_MODE,
				IMX415_REG_VALUE_08BIT, IMX415_MODE_SW_STANDBY);
		break;

    // 获取基础读出行数，返回 BRL（Basic Readout Lines），用于上层计算 RHS1/RHS2 动态调整限制。
	case RKMODULE_GET_SONY_BRL:
		if (imx415->cur_mode->width == 3864 && imx415->cur_mode->height == 2192)
			*((u32 *)arg) = BRL_ALL;
		else
			*((u32 *)arg) = BRL_BINNING;
		break;

    // 获取通道信息
	case RKMODULE_GET_CHANNEL_INFO:
		ch_info = (struct rkmodule_channel_info *)arg;
		ret = imx415_get_channel_info(imx415, ch_info);
		break;

    // 获取 D-PHY 参数，仅 HDR_X2 模式支持，返回 Samsung D-PHY 专用时序参数（用于高速稳定性）。
	case RKMODULE_GET_CSI_DPHY_PARAM:
		if (imx415->cur_mode->hdr_mode == HDR_X2) {
			dphy_param = (struct rkmodule_csi_dphy_param *)arg;
			*dphy_param = dcphy_param;
			dev_info(&imx415->client->dev,
				 "get sensor dphy param\n");
		} else
			ret = -EINVAL;
		break;

    // 未识别的命令返回错误。
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

// 该函数用于 在 64 位 Linux 内核上兼容 32 位用户空间程序 的 ioctl 调用。
// 在 64 位系统 上，内核指针是 8 字节，而 32 位用户程序 传递的指针是 4 字节。
// 如果直接将 32 位指针传给 64 位驱动，会导致地址错误（高 32 位为 0）。
// Linux 提供 compat_ptr() 将 32 位指针安全转换为 64 位内核指针。
// 同时，结构体在 32/64 位下可能因对齐不同而大小不一致，需特殊处理。
// 💡 简单说：这个函数让 32 位 app（如旧版 Android）能在 64 位 SoC（如 RK3588）上正常控制摄像头。
#ifdef CONFIG_COMPAT
static long imx415_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_awb_cfg *cfg;
	struct rkmodule_hdr_cfg *hdr;
	struct preisp_hdrae_exp_s *hdrae;
	struct rkmodule_channel_info *ch_info;
	long ret;
	u32  stream;
	u32 brl = 0;
	struct rkmodule_csi_dphy_param *dphy_param;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = imx415_ioctl(sd, cmd, inf);
		if (!ret) {
			if (copy_to_user(up, inf, sizeof(*inf))) {
				kfree(inf);
				return -EFAULT;
			}
		}
		kfree(inf);
		break;
	case RKMODULE_AWB_CFG:
		cfg = kzalloc(sizeof(*cfg), GFP_KERNEL);
		if (!cfg) {
			ret = -ENOMEM;
			return ret;
		}

		if (copy_from_user(cfg, up, sizeof(*cfg))) {
			kfree(cfg);
			return -EFAULT;
		}
		ret = imx415_ioctl(sd, cmd, cfg);
		kfree(cfg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		ret = imx415_ioctl(sd, cmd, hdr);
		if (!ret) {
			if (copy_to_user(up, hdr, sizeof(*hdr))) {
				kfree(hdr);
				return -EFAULT;
			}
		}
		kfree(hdr);
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		if (copy_from_user(hdr, up, sizeof(*hdr))) {
			kfree(hdr);
			return -EFAULT;
		}
		ret = imx415_ioctl(sd, cmd, hdr);
		kfree(hdr);
		break;
	case PREISP_CMD_SET_HDRAE_EXP:
		hdrae = kzalloc(sizeof(*hdrae), GFP_KERNEL);
		if (!hdrae) {
			ret = -ENOMEM;
			return ret;
		}

		if (copy_from_user(hdrae, up, sizeof(*hdrae))) {
			kfree(hdrae);
			return -EFAULT;
		}
		ret = imx415_ioctl(sd, cmd, hdrae);
		kfree(hdrae);
		break;
	case RKMODULE_SET_QUICK_STREAM:
		if (copy_from_user(&stream, up, sizeof(u32)))
			return -EFAULT;
		ret = imx415_ioctl(sd, cmd, &stream);
		break;
	case RKMODULE_GET_SONY_BRL:
		ret = imx415_ioctl(sd, cmd, &brl);
		if (!ret) {
			if (copy_to_user(up, &brl, sizeof(u32)))
				return -EFAULT;
		}
		break;
	case RKMODULE_GET_CHANNEL_INFO:
		ch_info = kzalloc(sizeof(*ch_info), GFP_KERNEL);
		if (!ch_info) {
			ret = -ENOMEM;
			return ret;
		}

		ret = imx415_ioctl(sd, cmd, ch_info);
		if (!ret) {
			ret = copy_to_user(up, ch_info, sizeof(*ch_info));
			if (ret)
				ret = -EFAULT;
		}
		kfree(ch_info);
		break;
	case RKMODULE_GET_CSI_DPHY_PARAM:
		dphy_param = kzalloc(sizeof(*dphy_param), GFP_KERNEL);
		if (!dphy_param) {
			ret = -ENOMEM;
			return ret;
		}

		ret = imx415_ioctl(sd, cmd, dphy_param);
		if (!ret) {
			ret = copy_to_user(up, dphy_param, sizeof(*dphy_param));
			if (ret)
				ret = -EFAULT;
		}
		kfree(dphy_param);
		break;

	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

// 配置寄存器并启动流
// 写寄存器 → 同步控制项 → 启动流
static int __imx415_start_stream(struct imx415 *imx415)
{
	int ret;

    // 非 Thunderboot 模式：写入寄存器表
	if (!imx415->is_thunderboot) {
		ret = imx415_write_array(imx415->client, imx415->cur_mode->global_reg_list);
		if (ret)
			return ret;
		ret = imx415_write_array(imx415->client, imx415->cur_mode->reg_list);
		if (ret)
			return ret;
	}

	/* In case these controls are set before streaming */
    // 将当前 V4L2 控制项（如曝光、增益）同步到硬件。
	ret = __v4l2_ctrl_handler_setup(&imx415->ctrl_handler);

    // Thunderboot：快速启动模式，跳过寄存器配置（假设已预配置）。
	if (ret)
		return ret;

    // 否则：
    // 先写 全局寄存器表（如 ADC、黑电平）
    // 再写 模式专属寄存器表（帧率、HDR 开关等）
	if (imx415->has_init_exp && imx415->cur_mode->hdr_mode != NO_HDR) {
		ret = imx415_ioctl(&imx415->subdev, PREISP_CMD_SET_HDRAE_EXP, &imx415->init_hdrae_exp);
		if (ret) {
			dev_err(&imx415->client->dev, "init exp fail in hdr mode\n");
			return ret;
		}
	}

    // 写 0x3000 = 0x00，启动视频流输出
	return imx415_write_reg(imx415->client, IMX415_REG_CTRL_MODE, IMX415_REG_VALUE_08BIT, 0);
}

// 停止流并清理状态
static int __imx415_stop_stream(struct imx415 *imx415)
{
	imx415->has_init_exp = false; // 清除初始曝光标记
	if (imx415->is_thunderboot)
		imx415->is_first_streamoff = true; // 标记首次停止流

    // 写 0x3000 = 0x01，进入待机模式（停止输出）。
	return imx415_write_reg(imx415->client, IMX415_REG_CTRL_MODE, IMX415_REG_VALUE_08BIT, 1);
}

// 用户空间启动/停止流，_s_​ 表示 "set"（设置/控制），是 V4L2 子系统的标准前缀，表示这是一个控制/设置类操作，sd​ 表示 "subdev"（子设备）
static int imx415_s_stream(struct v4l2_subdev *sd, int on)
{
	struct imx415 *imx415 = to_imx415(sd);
	struct i2c_client *client = imx415->client;
	int ret = 0;

	dev_info(&imx415->client->dev, "s_stream: %d. %dx%d, hdr: %d, bpp: %d\n",
	       on, imx415->cur_mode->width, imx415->cur_mode->height,
	       imx415->cur_mode->hdr_mode, imx415->cur_mode->bpp);

    // 先上锁
	mutex_lock(&imx415->mutex);
	on = !!on; // 强制转为 0/1
	if (on == imx415->streaming)
		goto unlock_and_return; // 状态未变，直接返回

    // 启动流（on=1）
	if (on) {
        // Thunderboot 特殊处理：如果 ISP 未就绪，手动上电
		if (imx415->is_thunderboot && rkisp_tb_get_state() == RKISP_TB_NG) {
			imx415->is_thunderboot = false;
			__imx415_power_on(imx415);
		}

        // 增加 runtime PM 引用计数（防止休眠）
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

        // 启动流
		ret = __imx415_start_stream(imx415);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev); // 减少引用计数
			goto unlock_and_return;
		}
	} 
    // 停止流（on=0）
    else {
		__imx415_stop_stream(imx415);
		pm_runtime_put(&client->dev); // 减少引用计数
	}

	imx415->streaming = on;

unlock_and_return:
	mutex_unlock(&imx415->mutex); // C语言中的标签（label）和 goto 语句

	return ret;
}

// V4L2 电源控制接口
// 仅管理 runtime PM 引用计数，不直接操作硬件电源。
// 实际电源控制由 __imx415_power_on/off 在 runtime PM 回调中处理。
static int imx415_s_power(struct v4l2_subdev *sd, int on)
{
	struct imx415 *imx415 = to_imx415(sd);
	struct i2c_client *client = imx415->client;
	int ret = 0;

	mutex_lock(&imx415->mutex);

	if (imx415->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}
		imx415->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		imx415->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&imx415->mutex);

	return ret;
}

// 严格按顺序上电，pinctrl → 电源 → 复位 → 时钟 → 延时
int __imx415_power_on(struct imx415 *imx415)
{
	int ret;
	struct device *dev = &imx415->client->dev;

    // 设置引脚状态（pinctrl），将 GPIO 配置为正常工作状态（如 I2C 功能）。
	if (!IS_ERR_OR_NULL(imx415->pins_default)) {
		ret = pinctrl_select_state(imx415->pinctrl, imx415->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}

    // 非 Thunderboot 模式：上电
	if (!imx415->is_thunderboot) {
        // 1. 使能三路电源（dvdd/avdd/dovdd）
		ret = regulator_bulk_enable(IMX415_NUM_SUPPLIES, imx415->supplies);
		if (ret < 0) {
			dev_err(dev, "Failed to enable regulators\n");
			goto err_pinctrl;
		}

        // 2. 拉高 power_gpio（如有）
		if (!IS_ERR(imx415->power_gpio))
			gpiod_direction_output(imx415->power_gpio, 1);
		/* At least 500ns between power raising and XCLR */
		/* fix power on timing if insmod this ko */

        // 3. 延时 10-20ms（满足电源稳定时序）
		usleep_range(10 * 1000, 20 * 1000);

        // 4. 拉低 reset_gpio（退出复位）
		if (!IS_ERR(imx415->reset_gpio))
			gpiod_direction_output(imx415->reset_gpio, 0);

		/* At least 1us between XCLR and clk */
		/* fix power on timing if insmod this ko */
        // 5. 延时 10-20ms（满足复位释放时序）
		usleep_range(10 * 1000, 20 * 1000);
	}

    // 使能外部时钟（xvclk）
	ret = clk_set_rate(imx415->xvclk, imx415->cur_mode->xvclk);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate\n");
	if (clk_get_rate(imx415->xvclk) != imx415->cur_mode->xvclk)
		dev_warn(dev, "xvclk mismatched\n");
	ret = clk_prepare_enable(imx415->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		goto err_clk;
	}

    // 最终延时
	/* At least 20us between XCLR and I2C communication */
	if (!imx415->is_thunderboot)
		usleep_range(20*1000, 30*1000);  // 等待 20-30ms，确保 I2C 可通信

	return 0;

    //错误处理
err_clk:
	// 复位 GPIO 拉高
	// 关闭电源
	if (!IS_ERR(imx415->reset_gpio))
		gpiod_direction_output(imx415->reset_gpio, 1);
	regulator_bulk_disable(IMX415_NUM_SUPPLIES, imx415->supplies);

err_pinctrl:
	// 切换到 sleep 引脚状态
	if (!IS_ERR_OR_NULL(imx415->pins_sleep))
		pinctrl_select_state(imx415->pinctrl, imx415->pins_sleep);

	return ret;
}

// 安全断电，复位 → 关时钟 → sleep 引脚 → 关电源
static void __imx415_power_off(struct imx415 *imx415)
{
	int ret;
	struct device *dev = &imx415->client->dev;

    // Thunderboot 模式：仅在首次停止流时断电
	if (imx415->is_thunderboot) {
		if (imx415->is_first_streamoff) {
			imx415->is_thunderboot = false;
			imx415->is_first_streamoff = false;
		} else {
			return; // 跳过断电
		}
	}

    // 1. 拉高 reset_gpio（进入复位）
	if (!IS_ERR(imx415->reset_gpio))
		gpiod_direction_output(imx415->reset_gpio, 1);

    // 2. 关闭 xvclk 时钟
	clk_disable_unprepare(imx415->xvclk);

    // 3. 切换引脚到 sleep 状态（省电）
	if (!IS_ERR_OR_NULL(imx415->pins_sleep)) {
		ret = pinctrl_select_state(imx415->pinctrl,
					   imx415->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}

    // 4. 拉低 power_gpio（如有）
	if (!IS_ERR(imx415->power_gpio))
		gpiod_direction_output(imx415->power_gpio, 0);
    
    // 5. 关闭三路电源
	regulator_bulk_disable(IMX415_NUM_SUPPLIES, imx415->supplies);
}

// 设备唤醒（上电）
// 触发时机：当用户空间打开摄像头设备（如 /dev/video0）时，内核自动调用此函数。!
static int __maybe_unused imx415_runtime_resume(struct device *dev)
{
    // 将通用设备指针 dev 转换为 I2C 客户端结构体（因为 IMX415 通过 I2C 控制）。
	struct i2c_client *client = to_i2c_client(dev);

    // 从 I2C client 中获取 V4L2 子设备数据
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

    // 通过容器转换宏，从 V4L2 子设备获取主驱动结构体 imx415。
	struct imx415 *imx415 = to_imx415(sd);

    // 核心操作：调用完整的上电流程（使能电源、释放复位、启动时钟等）。
	return __imx415_power_on(imx415);
}

//  设备挂起（断电）
// 当摄像头设备空闲一段时间后，内核自动调用此函数以省电。
static int __maybe_unused imx415_runtime_suspend(struct device *dev)
{
    // // ... 同样的三层转换获取 imx415
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx415 *imx415 = to_imx415(sd);

    // 核心操作：调用安全断电流程（拉高复位、关闭时钟、切断电源）。
	__imx415_power_off(imx415);

	return 0;
}

// V4L2 Subdev Open 接口
// CONFIG_VIDEO_V4L2_SUBDEV_API是内核配置选项，启用高级子设备 API。
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API

//  初始化 TRY 格式
// 此函数在用户空间首次打开子设备（如 /dev/v4l-subdevX）时调用。
static int imx415_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct imx415 *imx415 = to_imx415(sd);

	// 获取 TRY 格式 指针：
	// TRY 格式是 V4L2 的“预览”机制，用于在不改变硬件状态的情况下测试格式设置。
	// 存储在 fh（file handle）中，每个打开的文件描述符独立。
	struct v4l2_mbus_framefmt *try_fmt = v4l2_subdev_get_try_format(sd, fh->pad, 0);

	// 选择第一个支持的模式作为默认格式（通常是 3864×2192@30fps Linear）。
	const struct imx415_mode *def_mode = &imx415->supported_modes[0];

	// 加锁确保线程安全。
	mutex_lock(&imx415->mutex);
	
	// 初始化 TRY 格式 为默认值,避免未定义行为。
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = def_mode->bus_fmt;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&imx415->mutex);
	/* No crop or compose */

	return 0;
}
#endif

// 此函数用于枚举特定分辨率/格式下的帧率。
// 用户空间通过 VIDIOC_SUBDEV_ENUM_FRAME_INTERVAL 调用。
// 应用程序调用此函数循环 index=0,1,2...，获取所有支持的 (分辨率, 帧率, HDR) 组合。
static int imx415_enum_frame_interval(struct v4l2_subdev *sd,
	struct v4l2_subdev_pad_config *cfg,
	struct v4l2_subdev_frame_interval_enum *fie)
{
	struct imx415 *imx415 = to_imx415(sd);

	// 检查索引是否越界（cfg_num 是支持的模式总数）。
	if (fie->index >= imx415->cfg_num)
		return -EINVAL;

	// 填充对应模式的：
	// 总线格式（如 SGBRG10_1X10）
	// 分辨率
	// 最大帧率（以分数形式表示，如 {10000, 300000} = 30fps）
	fie->code = imx415->supported_modes[fie->index].bus_fmt;
	fie->width = imx415->supported_modes[fie->index].width;
	fie->height = imx415->supported_modes[fie->index].height;
	fie->interval = imx415->supported_modes[fie->index].max_fps;
	// 通过 reserved[0] 返回 HDR 模式（NO_HDR / HDR_X2 / HDR_X3）。
	fie->reserved[0] = imx415->supported_modes[fie->index].hdr_mode;
	return 0;
}

// 裁剪起始位置计算宏
// 作用：计算从原始分辨率 SRC 裁剪到目标分辨率 DST 的起始坐标。
// 关键点：
// /2：居中裁剪（左右/上下各留一半黑边）
// /4 * 4：对齐到 4 的倍数（满足 ISP 输入要求）
// 示例：
// CROP_START(3864, 3840) = (24)/2/4*4 = 12/4*4 = 8
// 实际裁剪：从第 8 列开始，取 3840 列
#define CROP_START(SRC, DST) (((SRC) - (DST)) / 2 / 4 * 4)

// 标准分辨率常量
// 定义 ISP 要求的标准分辨率（宽 16 对齐，高 8 对齐）。
#define DST_WIDTH_3840 3840
#define DST_HEIGHT_2160 2160
#define DST_WIDTH_1920 1920
#define DST_HEIGHT_1080 1080

/*
 * The resolution of the driver configuration needs to be exactly
 * the same as the current output resolution of the sensor,
 * the input width of the isp needs to be 16 aligned,
 * the input height of the isp needs to be 8 aligned.
 * Can be cropped to standard resolution by this function,
 * otherwise it will crop out strange resolution according
 * to the alignment rules.
 */
 // 裁剪配置
//  为什么需要裁剪？
// Sony IMX415 输出 3864×2192，但 ISP 要求输入 宽 16 对齐、高 8 对齐。
// 3864 % 16 = 8 → 不对齐 → 必须裁剪为 3840×2160。
static int imx415_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_selection *sel)
{
	struct imx415 *imx415 = to_imx415(sd);

	// 仅处理 CROP_BOUNDS 请求
	// V4L2_SEL_TGT_CROP_BOUNDS：查询传感器支持的最大裁剪范围。
	// 其他类型（如 CROP_ACTIVE）返回 -EINVAL。
	if (sel->target == V4L2_SEL_TGT_CROP_BOUNDS) {
		// 全像素模式（3864×2192）,裁剪为 3840×2160（UHD 4K），居中对齐。
		if (imx415->cur_mode->width == 3864) {
			sel->r.left = CROP_START(imx415->cur_mode->width, DST_WIDTH_3840);
			sel->r.width = DST_WIDTH_3840;
			sel->r.top = CROP_START(imx415->cur_mode->height, DST_HEIGHT_2160);
			sel->r.height = DST_HEIGHT_2160;
		}
		// Binning 模式（1944×1097）, 裁剪为 1920×1080（Full HD），居中对齐。
		else if (imx415->cur_mode->width == 1944) {
			sel->r.left = CROP_START(imx415->cur_mode->width, DST_WIDTH_1920);
			sel->r.width = DST_WIDTH_1920;
			sel->r.top = CROP_START(imx415->cur_mode->height, DST_HEIGHT_1080);
			sel->r.height = DST_HEIGHT_1080;
		} 
		// 其他模式（兜底）,CROP_START(X, X) = 0，即不裁剪。
		else {
			sel->r.left = CROP_START(imx415->cur_mode->width, imx415->cur_mode->width);
			sel->r.width = imx415->cur_mode->width;
			sel->r.top = CROP_START(imx415->cur_mode->height, imx415->cur_mode->height);
			sel->r.height = imx415->cur_mode->height;
		}
		return 0;
	}
	return -EINVAL;
}

// V4L2 操作集注册
// 电源管理操作集,绑定 Runtime PM 回调函数（前面已解释）。
static const struct dev_pm_ops imx415_pm_ops = {
	SET_RUNTIME_PM_OPS(imx415_runtime_suspend,
			   imx415_runtime_resume, NULL)
};

// 内部操作集（V4L2 高级 API）
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API

// 在打开子设备时初始化 TRY 格式。
static const struct v4l2_subdev_internal_ops imx415_internal_ops = {
	.open = imx415_open,
};
#endif

// 核心操作集
static const struct v4l2_subdev_core_ops imx415_core_ops = {
	.s_power = imx415_s_power,      // 电源控制
	.ioctl = imx415_ioctl,          // 专有 ioctl
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = imx415_compat_ioctl32, // 32 位兼容
#endif
};

// 视频操作集
static const struct v4l2_subdev_video_ops imx415_video_ops = {
	.s_stream = imx415_s_stream,           // 启停流
	.g_frame_interval = imx415_g_frame_interval, // 获取帧率
};

// Pad 操作集（格式/裁剪/枚举）
static const struct v4l2_subdev_pad_ops imx415_pad_ops = {
	.enum_mbus_code = imx415_enum_mbus_code,     // 枚举总线格式
	.enum_frame_size = imx415_enum_frame_sizes,  // 枚举分辨率
	.enum_frame_interval = imx415_enum_frame_interval, // 枚举帧率
	.get_fmt = imx415_get_fmt,                   // 获取当前格式
	.set_fmt = imx415_set_fmt,                   // 设置格式
	.get_selection = imx415_get_selection,       // 获取裁剪配置
	.get_mbus_config = imx415_g_mbus_config,     // 获取 MIPI 配置
};

// v4l2总操作集注册,汇总上面三个操作
// 通过 subdev_ops 完整实现 V4L2 子设备标准接口
// 这是驱动对接 V4L2 框架的“接口表”，在 imx415_probe() 中通过 v4l2_subdev_init() 注册。
static const struct v4l2_subdev_ops imx415_subdev_ops = {
	.core	= &imx415_core_ops,
	.video	= &imx415_video_ops,
	.pad	= &imx415_pad_ops,
};

//  控制项处理
static int imx415_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx415 *imx415 = container_of(ctrl->handler,
					     struct imx415, ctrl_handler);
	struct i2c_client *client = imx415->client;
	s64 max;
	u32 vts = 0, val;
	int ret = 0;
	u32 shr0 = 0;

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	// 特殊处理：VBLANK 修改曝光范围,增大 VBLANK（垂直消隐） → 增大 VTS → 允许更长曝光时间。动态调整 V4L2_CID_EXPOSURE 的最大值。
	case V4L2_CID_VBLANK:
		if (imx415->cur_mode->hdr_mode == NO_HDR) {
			/* Update max exposure while meeting expected vblanking */
			max = imx415->cur_mode->height + ctrl->val - 8;
			__v4l2_ctrl_modify_range(imx415->exposure,
					 imx415->exposure->minimum, max,
					 imx415->exposure->step,
					 imx415->exposure->default_value);
		}
		break;
	}

	// 检查设备是否在使用,如果设备已挂起（未使用），跳过寄存器写入。
	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	// 曝光时间,仅 Linear 模式生效，HDR 模式需用 PREISP_CMD_SET_HDRAE_EXP。
	case V4L2_CID_EXPOSURE:
		if (imx415->cur_mode->hdr_mode != NO_HDR)
			goto ctrl_end; // HDR 模式由专用 ioctl 控制
		shr0 = imx415->cur_vts - ctrl->val;

		// 写入 SHR0 寄存器（长曝光起始行）
		ret = imx415_write_reg(imx415->client, IMX415_LF_EXPO_REG_L,
				       IMX415_REG_VALUE_08BIT,
				       IMX415_FETCH_EXP_L(shr0));
		ret |= imx415_write_reg(imx415->client, IMX415_LF_EXPO_REG_M,
				       IMX415_REG_VALUE_08BIT,
				       IMX415_FETCH_EXP_M(shr0));
		ret |= imx415_write_reg(imx415->client, IMX415_LF_EXPO_REG_H,
				       IMX415_REG_VALUE_08BIT,
				       IMX415_FETCH_EXP_H(shr0));
		dev_dbg(&client->dev, "set exposure(shr0) %d = cur_vts(%d) - val(%d)\n",
			shr0, imx415->cur_vts, ctrl->val);
		break;
	// 模拟增益,同样仅 Linear 模式生效。
	case V4L2_CID_ANALOGUE_GAIN:
		if (imx415->cur_mode->hdr_mode != NO_HDR)
			goto ctrl_end;
		// 写入 LF_GAIN 寄存器
		ret = imx415_write_reg(imx415->client, IMX415_LF_GAIN_REG_H,
				       IMX415_REG_VALUE_08BIT,
				       IMX415_FETCH_GAIN_H(ctrl->val));
		ret |= imx415_write_reg(imx415->client, IMX415_LF_GAIN_REG_L,
				       IMX415_REG_VALUE_08BIT,
				       IMX415_FETCH_GAIN_L(ctrl->val));
		dev_dbg(&client->dev, "set analog gain 0x%x\n",
			ctrl->val);
		break;
	// 垂直消隐,HDR 模式的 VTS 存储值被放大（X2/X4），需还原后写入硬件。
	case V4L2_CID_VBLANK:
		vts = ctrl->val + imx415->cur_mode->height;
		/*
		 * vts of hdr mode is double to correct T-line calculation.
		 * Restore before write to reg.
		 */
		if (imx415->cur_mode->hdr_mode == HDR_X2) {
			vts = (vts + 3) / 4 * 4; // 对齐到 4n
			imx415->cur_vts = vts;
			vts /= 2; // HDR 模式 VTS 存储值是实际值的 2 倍
		} else if (imx415->cur_mode->hdr_mode == HDR_X3) {
			vts = (vts + 11) / 12 * 12;  // 对齐到 12n
			imx415->cur_vts = vts;
			vts /= 4;
		} else {
			imx415->cur_vts = vts;
		}
		// 写入 VTS 寄存器
		ret = imx415_write_reg(imx415->client, IMX415_VTS_REG_L,
				       IMX415_REG_VALUE_08BIT,
				       IMX415_FETCH_VTS_L(vts));
		ret |= imx415_write_reg(imx415->client, IMX415_VTS_REG_M,
				       IMX415_REG_VALUE_08BIT,
				       IMX415_FETCH_VTS_M(vts));
		ret |= imx415_write_reg(imx415->client, IMX415_VTS_REG_H,
				       IMX415_REG_VALUE_08BIT,
				       IMX415_FETCH_VTS_H(vts));
		dev_dbg(&client->dev, "set vblank 0x%x vts %d\n",
			ctrl->val, vts);
		break;
	// 水平翻转
	// 读-改-写 0x3030 寄存器，避免覆盖其他位（如垂直翻转位）。
	case V4L2_CID_HFLIP:
		ret = imx415_read_reg(imx415->client, IMX415_FLIP_REG,
				      IMX415_REG_VALUE_08BIT, &val);
		if (ret)
			break;
		if (ctrl->val)
			val |= IMX415_MIRROR_BIT_MASK;// BIT(0)
		else
			val &= ~IMX415_MIRROR_BIT_MASK;
		ret = imx415_write_reg(imx415->client, IMX415_FLIP_REG,
				       IMX415_REG_VALUE_08BIT, val);
		break;
	// 垂直翻转
	case V4L2_CID_VFLIP:
		ret = imx415_read_reg(imx415->client, IMX415_FLIP_REG,
				      IMX415_REG_VALUE_08BIT, &val);
		if (ret)
			break;
		if (ctrl->val)
			val |= IMX415_FLIP_BIT_MASK;
		else
			val &= ~IMX415_FLIP_BIT_MASK;
		ret = imx415_write_reg(imx415->client, IMX415_FLIP_REG,
				       IMX415_REG_VALUE_08BIT, val);
		break;
	// 默认处理,打印未处理的控制项（用于调试）。
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	// 释放 Runtime PM 引用
ctrl_end:
	pm_runtime_put(&client->dev);

	return ret;
}

// v4l2接口,定义控制项的 设置回调函数，当用户空间修改 V4L2 控制项（如曝光、增益）时调用 imx415_set_ctrl()。
static const struct v4l2_ctrl_ops imx415_ctrl_ops = {
	.s_ctrl = imx415_set_ctrl,
};

//  创建 V4L2 控制项初始化
static int imx415_initialize_controls(struct imx415 *imx415)
{
	const struct imx415_mode *mode;
	struct v4l2_ctrl_handler *handler;
	s64 exposure_max, vblank_def;

	u64 pixel_rate;
	u64 max_pixel_rate;
	u32 h_blank;
	int ret;
	u8 lanes = imx415->bus_cfg.bus.mipi_csi2.num_data_lanes;

	handler = &imx415->ctrl_handler;
	mode = imx415->cur_mode;

	// // 初始化控制项处理器
	ret = v4l2_ctrl_handler_init(handler, 8);  // 预分配 8 个控制项
	if (ret)
		return ret;
	handler->lock = &imx415->mutex; // 绑定互斥锁

	// 创建 LINK_FREQ 控制项（只读菜单）
	// 类型：整数菜单（只能选预定义值）
	// 值：MIPI 频率索引（0~4）
	imx415->link_freq = v4l2_ctrl_new_int_menu(handler, NULL,
				V4L2_CID_LINK_FREQ,
				ARRAY_SIZE(link_freq_items) - 1, 0,
				link_freq_items);
	v4l2_ctrl_s_ctrl(imx415->link_freq, mode->mipi_freq_idx);

	/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
	// 创建 PIXEL_RATE 控制项（只读）
	// 计算公式：像素率 = (MIPI 频率 × 2 × lane 数) / 每像素位数
	// 只读（无 imx415_ctrl_ops）
	pixel_rate = (u32)link_freq_items[mode->mipi_freq_idx] / mode->bpp * 2 * lanes;
	max_pixel_rate = MIPI_FREQ_1188M / mode->bpp * 2 * lanes;
	imx415->pixel_rate = v4l2_ctrl_new_std(handler, NULL,
		V4L2_CID_PIXEL_RATE, 0, max_pixel_rate,
		1, pixel_rate);

	// 创建 HBLANK 控制项（只读）
	// 水平消隐 = HTS - width，固定值，不可修改。
	h_blank = mode->hts_def - mode->width;
	imx415->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
				h_blank, h_blank, 1, h_blank);
	if (imx415->hblank)
		imx415->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	// 创建 VBLANK 控制项（可读写）
	// 范围：[vblank_def, IMX415_VTS_MAX - height]
	// 修改 VBLANK 可动态调整帧率。
	vblank_def = mode->vts_def - mode->height;
	imx415->vblank = v4l2_ctrl_new_std(handler, &imx415_ctrl_ops,
				V4L2_CID_VBLANK, vblank_def,
				IMX415_VTS_MAX - mode->height,
				1, vblank_def);
	imx415->cur_vts = mode->vts_def;

	// 创建EXPOSURE控制项,曝光范围：[4, VTS-8]（Sony 要求最小 4 行）
	exposure_max = mode->vts_def - 8;
	imx415->exposure = v4l2_ctrl_new_std(handler, &imx415_ctrl_ops,
				V4L2_CID_EXPOSURE, IMX415_EXPOSURE_MIN,
				exposure_max, IMX415_EXPOSURE_STEP,
				mode->exp_def);

	//  创建ANALOGUE_GAIN控制项,增益范围：[0, 0xf0]
	imx415->anal_a_gain = v4l2_ctrl_new_std(handler, &imx415_ctrl_ops,
				V4L2_CID_ANALOGUE_GAIN, IMX415_GAIN_MIN,
				IMX415_GAIN_MAX, IMX415_GAIN_STEP,
				IMX415_GAIN_DEFAULT);

	// 创建翻转控制项,开关型控制项（0=关闭，1=开启）
	v4l2_ctrl_new_std(handler, &imx415_ctrl_ops, V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(handler, &imx415_ctrl_ops, V4L2_CID_VFLIP, 0, 1, 1, 0);

	// 错误处理
	if (handler->error) {
		ret = handler->error;
		dev_err(&imx415->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	// 绑定
	imx415->subdev.ctrl_handler = handler;
	imx415->has_init_exp = false;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

// 验证芯片型号
// Thunderboot 模式跳过检查（假设硬件正确）
// 读取 0x311A 寄存器验证是否为 IMX415（ID=0xE0）
static int imx415_check_sensor_id(struct imx415 *imx415,
				  struct i2c_client *client)
{
	struct device *dev = &imx415->client->dev;
	u32 id = 0;
	int ret;

	if (imx415->is_thunderboot) {
		dev_info(dev, "Enable thunderboot mode, skip sensor id check\n");
		return 0;
	}

	ret = imx415_read_reg(client, IMX415_REG_CHIP_ID,
			      IMX415_REG_VALUE_08BIT, &id);
	if (id != CHIP_ID) {
		dev_err(dev, "Unexpected sensor id(%06x), ret(%d)\n", id, ret);
		return -ENODEV;
	}

	dev_info(dev, "Detected imx415 id %06x\n", CHIP_ID);

	return 0;
}

// 电源配置
// 使用 devm_ 版本自动管理资源释放
// 申请三路电源：dvdd, dovdd, avdd
static int imx415_configure_regulators(struct imx415 *imx415)
{
	unsigned int i;

	for (i = 0; i < IMX415_NUM_SUPPLIES; i++)
		imx415->supplies[i].supply = imx415_supply_names[i];

	return devm_regulator_bulk_get(&imx415->client->dev,
				       IMX415_NUM_SUPPLIES,
				       imx415->supplies);
}

// 设备探测（Probe）,驱动初始化入口!!!
static int imx415_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct imx415 *imx415;
	struct v4l2_subdev *sd;
	struct device_node *endpoint;
	char facing[2];
	int ret;
	u32 i, hdr_mode = 0;

	// 打印驱动版本
	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	// 分配内存
	imx415 = devm_kzalloc(dev, sizeof(*imx415), GFP_KERNEL);
	if (!imx415)
		return -ENOMEM;

	// 解析设备树属性,获取模组信息（用于 /sys 和日志）
	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX, &imx415->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING, &imx415->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME, &imx415->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME, &imx415->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	// 解析 HDR 模式
	ret = of_property_read_u32(node, OF_CAMERA_HDR_MODE, &hdr_mode);
	if (ret) {
		hdr_mode = NO_HDR;
		dev_warn(dev, " Get hdr mode failed! no hdr default\n");
	}

	// 解析 MIPI 连接
	endpoint = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!endpoint) {
		dev_err(dev, "Failed to get endpoint\n");
		return -EINVAL;
	}

	// 获取 lane 数（2-lane 或 4-lane）
	ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(endpoint), &imx415->bus_cfg);
	of_node_put(endpoint);
	if (ret) {
		dev_err(dev, "Failed to get bus config\n");
		return -EINVAL;
	}

	// 选择模式表
	imx415->client = client;
	if (imx415->bus_cfg.bus.mipi_csi2.num_data_lanes == IMX415_4LANES) {
		imx415->supported_modes = supported_modes;
		imx415->cfg_num = ARRAY_SIZE(supported_modes);
	} else {
		imx415->supported_modes = supported_modes_2lane;
		imx415->cfg_num = ARRAY_SIZE(supported_modes_2lane);
	}
	dev_info(dev, "detect imx415 lane %d\n",
		imx415->bus_cfg.bus.mipi_csi2.num_data_lanes);

	// 匹配 HDR 模式
	for (i = 0; i < imx415->cfg_num; i++) {
		if (hdr_mode == imx415->supported_modes[i].hdr_mode) {
			imx415->cur_mode = &imx415->supported_modes[i];
			break;
		}
	}

	// 解析 Thunderboot 使能
	of_property_read_u32(node, RKMODULE_CAMERA_FASTBOOT_ENABLE,
		&imx415->is_thunderboot);

	// 获取硬件资源
	imx415->xvclk = devm_clk_get(dev, "xvclk"); // 外部时钟
	if (IS_ERR(imx415->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}
	imx415->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_ASIS); // 复位 GPIO
	if (IS_ERR(imx415->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");
	imx415->power_gpio = devm_gpiod_get(dev, "power", GPIOD_ASIS); // 电源 GPIO
	if (IS_ERR(imx415->power_gpio))
		dev_warn(dev, "Failed to get power-gpios\n");
	imx415->pinctrl = devm_pinctrl_get(dev); // 引脚控制

	// 引脚控制报错处理
	if (!IS_ERR(imx415->pinctrl)) {
		imx415->pins_default =
			pinctrl_lookup_state(imx415->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(imx415->pins_default))
			dev_info(dev, "could not get default pinstate\n");

		imx415->pins_sleep =
			pinctrl_lookup_state(imx415->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_SLEEP);
		if (IS_ERR(imx415->pins_sleep))
			dev_info(dev, "could not get sleep pinstate\n");
	} else {
		dev_info(dev, "no pinctrl\n");
	}

	// 配置选项写入
	ret = imx415_configure_regulators(imx415);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	// 初始化互斥锁
	mutex_init(&imx415->mutex);

	// 初始化 V4L2 子设备
	sd = &imx415->subdev;
	v4l2_i2c_subdev_init(sd, client, &imx415_subdev_ops);

	// 初始化控制项
	ret = imx415_initialize_controls(imx415);
	if (ret)
		goto err_destroy_mutex;

	// 上电并验证 ID
	ret = __imx415_power_on(imx415);
	if (ret)
		goto err_free_handler;

	ret = imx415_check_sensor_id(imx415, client);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &imx415_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;
#endif

	// 设置 Media Controller（可选）
#if defined(CONFIG_MEDIA_CONTROLLER)
	imx415->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &imx415->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(imx415->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	// 设置子设备名称,示例：m00_b_imx415 i2c-1a
	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 imx415->module_index, facing,
		 IMX415_NAME, dev_name(sd->dev));
	
	// 注册子设备,到 V4L2 框架.!!!
	// 此时 /dev/v4l-subdevX 和 /dev/mediaX 节点创建成功。
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	// 启用 Runtime PM
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev); // 允许自动挂起

	return 0;

	// 错误处理
err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__imx415_power_off(imx415);
err_free_handler:
	v4l2_ctrl_handler_free(&imx415->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&imx415->mutex);

	return ret;
}

// 设备移除,驱动卸载
static int imx415_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx415 *imx415 = to_imx415(sd);

	v4l2_async_unregister_subdev(sd); // 注销子设备
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity); // 清理 Media Controller 
#endif
	v4l2_ctrl_handler_free(&imx415->ctrl_handler);// 释放控制项
	mutex_destroy(&imx415->mutex); // 销毁互斥锁

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev)) // 如果未挂起，手动断电
		__imx415_power_off(imx415);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

// 驱动注册
#if IS_ENABLED(CONFIG_OF)
// 设备树匹配表
static const struct of_device_id imx415_of_match[] = {
	{ .compatible = "sony,imx415" },
	{},
};
MODULE_DEVICE_TABLE(of, imx415_of_match);
#endif

// I2C 设备 ID 表
static const struct i2c_device_id imx415_match_id[] = {
	{ "sony,imx415", 0 },
	{ },
};

// I2C 驱动结构体
static struct i2c_driver imx415_i2c_driver = {
	.driver = {
		.name = IMX415_NAME,
		.pm = &imx415_pm_ops, // 绑定 Runtime PM
		.of_match_table = of_match_ptr(imx415_of_match),
	},
	.probe		= &imx415_probe,
	.remove		= &imx415_remove,
	.id_table	= imx415_match_id,
};

// 模块初始化
static int __init sensor_mod_init(void)
{
	// I2C函数,根据I2C地址创建
	return i2c_add_driver(&imx415_i2c_driver);
}

// 模块退出
static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&imx415_i2c_driver);
}

// device_initcall_sync() 和 module_exit() 是 Linux 内核模块初始化/退出机制 的宏
// 它们定义在内核头文件中（如 <linux/init.h>），用于控制驱动加载和卸载的时机。
// 注册一个同步初始化函数 sensor_mod_init，在内核启动的 设备初始化阶段 被调用。
// "同步" 意味着：
// 该函数在 内核主线程 中执行（非异步工作队列）
// 执行完成前，不会继续后续初始化步骤
// 避免因异步初始化导致的资源竞争或依赖问题
device_initcall_sync(sensor_mod_init); // 同步初始化（避免异步问题）

// 注册模块卸载函数 sensor_mod_exit。
// 当用户执行 rmmod imx415 时，内核调用此函数：
// 清理资源（注销子设备、释放内存等）
// 安全卸载驱动
module_exit(sensor_mod_exit);

// Linux 内核模块的元信息声明，用于描述模块的基本属性。
// 它们由内核构建系统（Kbuild）处理，并嵌入到最终的 .ko 驱动文件中。
// 提供模块的简短描述，说明该驱动的功能。
// `modinfo imx415.ko`
// # 输出示例：
// # description: Sony imx415 sensor driver
MODULE_DESCRIPTION("Sony imx415 sensor driver");
MODULE_LICENSE("GPL v2");
