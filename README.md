# Winner Micro W60X 板级支持包

## 1. 简介

W600芯片是[联盛德微电子](http://www.winnermicro.com)推出的一款嵌入式Wi-Fi SoC芯片。该芯片集成度高，所需外围器件少，性价比高。适用于IoT（智能家庭）领域各种智能产品。高度集成的Wi-Fi功能是其主要功能；另外，该芯片集成Cortex-M3内核，内置QFlash，SDIO、SPI、UART、GPIO、I²C、PWM、I²S、7816等接口, 支持多种硬件加解密算法。：

| 硬件 | 描述 |
| -- | -- |
|芯片型号| W60X全系列 |
|CPU| Cortex-M3 |
|主频| 80MHz |
|Flash|1MB|
|SRAM|288KB|

## 2. 编译说明


| 环境         | 说明                                                         |
| ------------ | ------------------------------------------------------------ |
| PC操作系统   | Linux/MacOS/Windows                                          |
| 编译器       | arm-none-eabi-gcc version 5.4.1 20160919 (release)/armcc/iar |
| 构建工具     | scons/mdk5/iar                                               |
| 依赖软件环境 | Env工具/(MDK或IAR或arm-none-eabi-gcc)/git/调试器驱动         |

1) 下载源码

```bash
    git clone https://github.com/RT-Thread/rt-thread.git
    cd rt-thread/bsp
    git clone https://github.com/flyingcys/w600
```

2) 配置工程并准备env

（Linux/Mac）

```bash
    cd w600
    scons --menuconfig
    source ~/.env/env.sh
    pkgs --upgrade
```

（Windows）

>在[RT-Thread官网][1]下载ENV工具包

3) 配置芯片型号

（Linux/Mac）

```bash
    scons --menuconfig
```

（Windows(ENV环境中)）

```bash
    menuconfig
```

在menuconfig页面配置并选择对应的芯片型号和配置外设:
![figure1](/figures/menuconfig.png)

![figure2](/figures/Deviceconfig.png)

若开发环境为MDK/IAR，则需要生成工程

4) 生成工程(Mac/Linux下请跳过此步骤)

（Windows IAR）

```bash
    SET RTT_CC=iar
    scons --target=iar -s
```

（Windows MDK5）*

```bash
    scons --target=mdk5 -s
```
（Windows MDK4）*

```bash
    scons --target=mdk4 -s
```
1) 编译

使用MDK或IAR请参见对应教程

（Windows arm-none-eabi-gcc）
使用以下指令设置gcc路径

```bash
    SET RTT_EXEC_PATH=[GCC路径]
```

（Linux/Mac arm-none-eabi-gcc）
使用以下指令设置gcc路径

```bash
    export RTT_EXEC_PATH=[GCC路径]
```

编译（WindowsLinux/Mac arm-none-eabi-gcc）

```bash
    scons -j4
```

## 3. 烧写及执行

如果编译正确无误，在`/w600/Bin`文件夹中生成img文件，参考[《WM_W600_固件升级指导》](/Libraries/DOC/WM_W600_固件升级指导_V1.1.pdf)进行固件烧写。

### 3.1 运行结果

如果编译 & 烧写无误，会在串口0*上看到RT-Thread的启动logo信息：

```bash
 \ | /
- RT -     Thread Operating System
 / | \     4.0.0 build Nov 22 2018
 2006 - 2018 Copyright by rt-thread team
```

*默认串口UART0 波特率115200


## 4. 驱动支持情况

| 驱动       | 支持情况 | 备注                                  |
| ---------- | :------: | :--------------------------:         |
| UART       | 支持     | UART0/UART1                          |
| GPIO       | 支持     | 自动根据芯片型号选择引脚布局            |
| SPI        | 支持     | 低速SPI,支持SPI BUS，8/32bit主机模式   |
| SPI Flash  | 支持     | 支持W25QXX、SFUD                      |
| WDT        | 支持     | 支持                                  |
| I2c        | 支持     | 硬件I2C                               |
| RTC        | 支持     | 支持ntp同步                           |
| ADC        | 支持     | 8 channel ADC采集 CPU温度采集          |
| PWM        | 支持     | 5 channel PWM输出 (3Hz~160kHz)        | 
| Timer      | 支持     | 5个Timers独立工作，支持1M(默认)频率     |
| WiFi       | 支持     | STA模式                                |

## 5. menuconfig Bsp菜单详解

| |选项 | 解释 |
| -- | -- | --|
| W60x Device Config|
| |Device type | 选择芯片型号 |
| |ADC Channel Config |ADC通道选择|
| |HW Timers Config| 硬件定时器选择|
| |PWM Channel Config |PWM 通道及Pin配置（编号）|
| |WM HW I2C Config | 硬件I2C配置 Pin以及频率|
| | SPI BUs Pin Config | SPI总线Pin配置及Flash CS Pin| 
| Using UART0 || 开启串口1，串口1的设备名为"uart0" |
| Using UART1 || 开启串口2，串口1的设备名为"uart1" |

*部分选项需要在RT-Thread组件菜单中开启对应的设备框架才能显示。

## 6. 联系人信息

维护人:

[flyingcys][2] < [294102238@qq.com][3] >

[fanwenl][4] < [fanwenl_g@126.com][5] >

  [1]: https://www.rt-thread.org/page/download.html
  [2]: https://github.com/flyingcys
  [3]: mailto:294102238@qq.com]
  [4]: https://github.com/fanwenl
  [5]: mailto:fanwenl_g@126.com]