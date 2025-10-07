# N32WB452 EIDE工程说明 - 使用pyocd下载调试

## 1. 创建并导入keil工程

## 2. 关于pyocd

### 2.1 前言
`pyocd`是开源的python库，它可以用来连接和调试各种ARM Cortex-M系列微控制器。但是，它目前只支持官方支持的MCU，如果要连接其他的MCU，需要自行适配。

`pyocd`具有多种调试器支持，包括`JLink`、`ST-Link`、`CMSIS-DAP`等。相较于`openocd`，`pyocd`的优势在于：
- 支持更多的MCU
- 烧写速度更快
- 支持的调试器更多

虽然调试功能可能不如openocd丰富，但使用`pyocd`可以使用RTT实时输出，方便查看程序运行时的输出。

**硬件要求：**
- 支持pyocd的MCU
- 支持pyocd的调试器，例如`JLink`、`ST-Link`、`daplink`等

**软件要求：**
- Python 3.6及以上版本
- pip工具

### 2.2 安装pyocd

#### 安装Python
安装Python 3.6及以上版本，并确保pip可用。

#### 安装pyocd库
使用pip安装pyocd库：
```bash
pip3 install pyocd
```

验证是否安装成功：
```bash
pyocd --version
```
可以查看到当前pyocd的版本号。

#### 更新pyocd官方pack库
```bash
pyocd pack update
```
等待更新完成。

### 2.3 添加常规MCU支持

如果要添加的MCU在官方有支持，即执行：
```bash
pyocd list --target
```
可以看到有相关的MCU，可以直接安装使用。

可以使用find命令快速查找相关MCU package：
```bash
pyocd pack -f stm32f103
```

查询结果如下：
```
  Part          Vendor               Pack                 Version   Installed
-------------------------------------------------------------------------------
  STM32F103C4   STMicroelectronics   Keil.STM32F1xx_DFP   2.4.1     false
  STM32F103C6   STMicroelectronics   Keil.STM32F1xx_DFP   2.4.1     false
```

说明pyocd支持这个型号的MCU。可以直接通过指令安装支持这个型号的MCU：
```bash
pyocd pack -i stm32f103
```

安装完成后，使用`pyocd pack -f stm32f103`查询：
```
  Part          Vendor               Pack                 Version   Installed
-------------------------------------------------------------------------------
  STM32F103C4   STMicroelectronics   Keil.STM32F1xx_DFP   2.4.1     true
  STM32F103C6   STMicroelectronics   Keil.STM32F1xx_DFP   2.4.1     true
```

## 3. 添加非官方支持的MCU - N32WB452

对于`N32WB452`需要添加非官方支持的MCU。有pack包的情况下可以较容易地添加对应MCU的支持。

### 3.1 添加支持步骤

在Windows下，这里的`name`为自己的用户名。

打开目录：
```
C:\Users\name\AppData\Local\cmsis-pack-manager\cmsis-pack-manager
```

1. 新建一个名字为`Nationstech`的文件夹
2. 在该文件夹内部创建一个文件夹名字为`N32WB452_DFP`
3. 将下载的pack包放入该文件夹
4. 修改名字为`1.1.0.pack`

### 3.2 修改index.json文件

在MCU列表中添加对应MCU的描述信息：
```json
"N32WB452CE": {
    "name": "N32WB452CEQ6",
    "memories": {
      "IRAM1": {
        "p_name": null,
        "access": {
          "read": true,
          "write": true,
          "execute": false,
          "peripheral": false,
          "secure": false,
          "non_secure": false,
          "non_secure_callable": false
        },
        "start": 536870912,
        "size": 153600,
        "startup": false,
        "default": true
      },
      "IROM1": {
        "p_name": null,
        "access": {
          "read": true,
          "write": false,
          "execute": true,
          "peripheral": false,
          "secure": false,
          "non_secure": false,
          "non_secure_callable": false
        },
        "start": 134217728,
        "size": 524288,
        "startup": true,
        "default": true
      }
    },
    "algorithms": [
      {
        "file_name": "Flash/N32WB452.FLM",
        "start": 134217728,
        "size": 524288,
        "default": true,
        "ram_start": null,
        "ram_size": null,
        "style": "Keil"
      }
    ],
    "processors": [
      {
        "core": "CortexM4",
        "fpu": "SinglePrecision",
        "mpu": "Present",
        "ap": {
          "Index": 0
        },
        "dp": 0,
        "address": null,
        "svd": "svd/N32WB452.svd",
        "name": null,
        "unit": 0,
        "default_reset_sequence": null
      }
    ],
    "from_pack": {
      "vendor": "Nationstech",
      "pack": "N32WB452_DFP",
      "version": "1.1.0",
      "url": "https://www.nsing.com.sg/public/uploads/armpacks/"
    },
    "vendor": "Nationstech:184",
    "family": "N32WB452 Series",
    "sub_family": "N32WB452"
  }
```

### 3.3 验证支持

完成后查看pack：
```bash
pyocd pack -f n32wb452
```

查询结果：
```
  Part           Vendor        Pack                       Version   Installed
-------------------------------------------------------------------------------
  N32WB452CEQ6   Nationstech   Nationstech.N32WB452_DFP   1.1.0     True
```

可以看到已经支持了N32WB452系列的MCU，`Installed`显示为`True`。如果这里为`false`，可执行安装命令：
```bash
pyocd pack -i N32WB452
```

至此，N32WB452系列的MCU的支持已经完成，可以使用pyocd执行烧写、调试等操作。
```