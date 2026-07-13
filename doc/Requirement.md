# Requirement

关节电机控制器程序，适配包括但不限于GIM4310-10和GIM4305-10电机，CAN通讯控制，串口打印调试数据，控制电机（转矩环、转速环、位置环、MIT、开环等）。此外，具有如下需求和规范：

- 裸机前后台控制，注意开销
- 初始工程为cubemx生成的hal库空工程，各外设配置参数均为默认不可参考，实际参数以实际工况调整。
- 留意cubemx覆盖区域，覆盖区域外编写
- 电机参数、控制相关参数和匹配硬件参数宏定义，并具有必要的统一文件的抽象
- can通信无明确协议，依照一般关节电机编写
- 具有预定位程序，预定位编码器和相序校正
- 生成的应用层和外设部分存于application文件夹的对应文件夹内，并在工程中具有必要引用
- 相应硬件资料见datasheet文件夹，未提及的自行搜索
- 2812灯珠作用为指示状态，需要具有必要灯语且亮度可控
- 程序具有必要的抽象以分离控制部分与通信应用层
- 命名方式、代码风格和调用风格与已有hal库一致，具有必要的注释且注释专业
- 输出必要的markdown
- 在资源保证的前提下可实现必要的shell、

此外，以下为来自硬件人员的补充内容：

- 外设电路：母线电压采样使用两个电阻串联分压后再输出到MCU引脚上；电机温度采样也是两个电阻分压，热敏电阻为电机内部的电阻，需要查找确定内部电阻阻值与温度的关系；MOS管的温度采样也是使用两个电阻进行串联分压，其中热敏电阻接在MCU引脚和GND之间，使用型号为[KNTC0603/10KF3950](https://item.szlcsc.com/3124166.html?fromZone=s_s__"C2892547"&spm=sc.gbn.xh1.zy.n___sc.hm.hd.ss&lcsc_vid=EVYLVl0AQgJdUlFTEQUKX1xRTgJZA1NfRlUMUgZXElUxVlNeRFZYVFxfRldfXjtW)；RGB灯使用型号为[XL-3528RGBW-2812B](https://item.szlcsc.com/3114290.html?fromZone=s_s__"C2890364"&spm=sc.gbn.xh1.zy.n___sc.gbn.hd.ss&lcsc_vid=EVYLVl0AQgJdUlFTEQUKX1xRTgJZA1NfRlUMUgZXElUxVlNeRFZYVFxfRldfXjtW)；CAN通讯使用的芯片为TJA1051TK/3/1J，其中还使用了一个电子开关芯片，型号为TS5A3159DCKR，用来控制120Ω电阻是否连接在电路中，可以应用在多个控制器连接的时候，可以控制第一个和最后一个控制器开启120Ω电阻。
- 使用MCU内部的运放配合外围电路进行电流采样，ADC基准1.65V，初始化时执行偏置补偿

