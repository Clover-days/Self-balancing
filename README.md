# 32_自平衡
# library,project是单片机的项目文件
# utilities是单片机的开发板头文件
# cfg.ht32是芯片配置文件，[HT32_CodeConfig](https://www.holtek.com.cn/page/detail/ice/HT32_CodeConfig)生成
##  GPIO使用方案
|引脚|IO名称|接入电路|工作方式|说明|
|----|------|-------|-------|----|
|1   |PA0   |V_BAT  |ADC    |检测电池电压|
|2   |PA1   |GPIO4  |OUT    |指示运行状态，每次主循环后反转电平|
|3   |PA2   |RGB    |OUT    |控制RGB，24bit一个|
|6   |PA5   |W_EN   |OUT    |WiFi使能控制，默认输出高电平|
|11  |PC4   |KEY3   |ADC    |检测触摸|
|12  |PC5   |KEY1   |ADC    |检测触摸|
|14  |PC9   |KEY2   |ADC    |检测触摸|
|28  |Pb15  |TXD1   |USART1 |WIFI通信|
|29  |PC0   |RXD1   |USART1 |WIFI通信|
|32  |PC12  |TXD0   |UART1  |电脑通信，重定向（printf）|
|33  |PC13  |RXD0   |UART1  |电脑通信，重定向（printf）|
|35  |PA9   |GPIO0  |BOOT   |    |
|38  |PA12  |SCK    |SWDIO  |下载电路|
|39  |PA13  |SDO    |SWCLK  |下载电路|
|40  |PA14  |MO_IN1 |MCMT   |马达控制|
|44  |PB0   |MO_IN2 |MCMT   |马达控制|
|46  |PD1   |MO_IN2 |MCMT   |马达控制|
|52  |PB5   |SCN    |OUT    |SSI通信，选择信号，低电平有效，默认高|
|53  |PC14  |SCLK   |OUT    |SSI通信，时钟信号，下降沿取值，默认高|
|54  |PC15  |SCDO   |IN     |SSI通信，数据信号，第二位有效，悬空|
|61  |PB7   |SCL_0  |I2C    |读取mpu6050|
|62  |PB8   |SDA_0  |I2C    |读取mpu6050|
## 任务列表
- MT6701
  >数据[24]{[0:13]角度数据;[14:17]磁场状态数据;[18:23]CRC校验码}

- WiFi
    >芯片esp8266

    >使用usart0

    >发送内容
    
    |   变量          | 说明    |
    |---------------|---------------------- |
    | vmotor.shaft_velocity  | 轴速度| 
    | motor.voltage.q | 摇摆电压|
    |pendulum_angle   |当前角度与期望角度差值|
    |target_angle   |平衡角度|
    |kalAngleZ   |使用卡尔曼滤波器计算的角度|
    |gyroZrate  |转换后的deg/s|
   >接收内容

    |   参数命令           | 说明    |
    | ---------------- |---------------------- |
    | TA | target_angle平衡角度 例如TA89.3 设置平衡角度89.3| 
    | SV | swing_up_voltage摇摆电压 左右摇摆的电压，越大越快到平衡态，但是过大会翻过头|
    |SA|swing_up_angle摇摆角度 离平衡角度还有几度时候，切换到自平衡控制|
    |VP1|速度环的PID的P，1是稳定在平衡角度之前的P值|
    |VI1|速度环的PID的I，1是稳定在平衡角度之前的I值|
    |VP2|速度环的PID的P，2是稳定后的P值|
    |VI2|速度环的PID的I，2是稳定后的I值|
    |K为LQR参数|**3和4**是速度控制稳定前和后|
    |K3**1**|LQR的参数1：稳定前的角度差值|
    |K3**2**|LQR的参数2：稳定前的左右倾倒加速度|
    |K3**3**|LQR的参数3：稳定前的当前速度|
    |K4**1**|LQR的参数1：稳定后的角度差值|
    |K4**2**|LQR的参数2：稳定后的左右倾倒加速度|
    |K4**3**|LQR的参数3：稳定后的当前速度|  
## 任务列表
 为了精确计时SSI使用SysTick计时
> Md语法:https://markdown.com.cn/basic-syntax/# Self-balancing
