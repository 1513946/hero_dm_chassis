# RoboMaster M3508 直流无刷电机_使用说明（中英）

> 本文由 PyMuPDF 快速提取；页码标题对应原 PDF 页序。

<!-- page: 1 -->

## 第 1 页

M3508 Brushless DC Motor
M3508   直流无刷电机
User Guide
使用说明
 v1.0  2025.10

<!-- page: 2 -->

## 第 2 页

*[本页未检测到可提取文本]*

<!-- page: 3 -->

## 第 3 页

Contents

      Disclaimer                                                  2
      Introduction                                                2
       In the Box                                                  2
      Using the Product                                           2
          Connecting to the MSC                                            2
             Installation                                                     3
           Precautions                                                     3
      Performance and Parameters                                  4
          Payload Characteristic Curves                                      4
            Characteristic Parameters                                         5
          Output Gear Parameters                                          5

目录
                                                                6   免责声明
                                                                6   简介
                                                                6   物品清单
                                                                6   产品使用                                                                             6      连接电调
                                                                             7      安装
                                                                             7      注意事项
                                                                8   性能与参数                                                                             8      负载特性曲线
                                                                             9      特征参数
                                                                             9      输出齿参数

                                                                1

<!-- page: 4 -->

## 第 4 页

Disclaimer

By using this product, you signify that you have read, understand, and accept the terms
and conditions of this guideline and all instructions at https://www.robomaster.com.
THE PRODUCT AND ALL MATERIALS AND CONTENT AVAILABLE THROUGH THE PRODUCT
ARE PROVIDED "AS IS" AND ON AN "AS AVAILABLE" BASIS WITHOUT WARRANTY OR
CONDITION OF ANY KIND.

Introduction

RoboMaster M3508 Brushless DC Motor (hereinafter referred to as the "motor") is
designed for high-performance robots. When working with RoboMaster C620 Brushless
DC Motor Speed Controller (hereinafter referred to as the "MSC"; the MSC is sold
separately), it supports sinusoidal (sine‑wave) drive, which provides higher efficiency,
better dynamic performance, and greater stability than traditional square‑wave drive.

In the Box

Motor                                × 1    7-Pin Data Cable

                                                                                 × 1

Using the Product

 Connecting to the MSC
1. Use the included 7-pin data cable to connect the 7-pin ports on the MSC and the motor.
2. Connect the three-phase cable connectors of the MSC and the motor, ensuring that the
   wire colors match. DO NOT insert the connectors in the wrong direction.

                                           2

                                           1

2

<!-- page: 5 -->

## 第 5 页

Installation
Refer to the following dimensions to select the mounting holes and install the motor
based on your needs.

                                             4- M3  4 EQS
Unit: mm

                                              26.8 -0.060
                                            35.00                                  13.00
                                            42.00
                                                                                             2- M3  4 EQS

                                  43.40                                             7.00 0.02
                                 30.10
                                              8.30 1
                               17.60
                                                                                                                                                                          -0.025-0.035
                                                                     5.00

                                            2.00

 Precautions

       • The motor has M3 threaded holes with a depth of 4 mm. DO NOT use excessively
      long screws. Otherwise, the printed circuit board (PCB) may be damaged.
       • Make sure that the motor is securely installed.
       • Use the motor under the maximum winding temperature. Prolonged overheating
      can permanently damage the motor.
       • Avoid damage to the cables and wires. Otherwise, the motor may work improperly.
       • DO NOT touch the rotors to avoid injury.
       • DO NOT allow any foreign materials to come into contact with the rotors, as this
     may affect performance.
       • The motor will heat during high-power operation. Avoid direct contact to prevent
      burns.

                                                                3

<!-- page: 6 -->

## 第 6 页

Performance and Parameters

 Payload Characteristic Curves

Closed-Loop Speed Control Using C620 MSC

            n (rpm)  η (%)                                                                            I (A) P (W)

              8000    80                                      16  400                               η              7000    70                                      14  350                        n                                                 I              6000    60                                      12  300
              5000    50                           P        10  250
              4000    40                                       8   200
              3000    30                                       6   150
              2000    20                                       4   100
              1000    10                                       2   50
                0      0                                       0    0
                         0    0.05  0.10  0.15  0.20  0.25  0.30  0.35
                                                            T (N·m)

Closed-Loop Current Control Using C620 MSC

            n (rpm)  η (%)                                                                           I (A) P (W)
               9000    90   n                                  18  450               8000    80                                      16  400
               7000    70                                      14  350                                                                                                        I
               6000    60   η                                  12  300
               5000    50                                      10  250                                             P               4000    40                                      8   200
               3000    30                                      6   150
               2000    20                                      4   100
               1000    10                                      2   50
                0      0                                      0    0
                         0   0.05  0.10  0.15  0.20  0.25  0.30  0.35
                                                           T (N·m)

Operating Range Diagram

                n (rpm)                                      125° C (257° F)

                    10000

                     8000

                     6000

                     4000

                     2000

                        0        0.04      0.08      0.12       0.16
                              Continuous Operating Range    T (N·m)

            n-Rotational Speed, η-Efficiency, I-Current, P-Output Power, T-Torque

The above data were measured under laboratory conditions at an input voltage of 24 V, an ambient temperature of 25
°C (77 °F), and under normal cooling. They are provided for reference only. Adjust the motor running time based on the
actual conditions such as operating temperature and cooling conditions.

4

<!-- page: 7 -->

## 第 7 页

Characteristic Parameters

 Motor
 Rated Voltage                        24 V
 Torque Constant                         0.02 N·m/A
 Rotational Speed Constant              390 rpm/V
 Speed-Torque Gradient                260 rpm/N·m
 Mechanical Time Constant              49 ms
 Phase Resistance                        0.194 Ω
 Phase Inductance                       0.097 mH
 Operating Temperature                  0° to 50° C (32° to 122° F)
 Maximum Winding Temperature         125° C (257° F)
 Number of Pole Pairs                  7
 Rated Dynamic Load                   276 N
 Motor Weight                        120 g
 Motor and C620 MSC at Rated Voltage*
 No-Load Speed                       9400 rpm
 No-Load Current                          0.6 A
 Rated Rotational Speed                9085 rpm
 Rated Torque (Maximum Continuous      0.16 N·m
 Torque)
 Rated Current                        10 A
 Maximum Efficiency                 83%
 Peak Torque                             0.33 N·m
 Peak Current                              2.5 A

* All data in this table were measured under laboratory conditions. All current values represent the input
  current of the MSC, and the efficiency refers to the overall efficiency of the motor-MSC drive system.

 Output Gear Parameters

 Number of Teeth                     17
 Module                                   0.6
 Pressure Angle                         20°
 Pitch Diameter                           10.2
 Addendum Diameter                    11.38
 Dedendum Diameter                     8.68
 Profile Shift Coefficient                     -0.01
 Tooth Span                          2
Common Normal                        2.79 -0.02-0.04
                                                                5

<!-- page: 8 -->

## 第 8 页

免责声明
使用本产品前，请您仔细阅读本文、访问https://www.robomaster.com 阅读本产品相关的所
有指引。使用本产品视为您已经阅读并接受本文与本产品相关的全部条款。
简介
RoboMaster M3508         直流无刷电机（以下简称为“电机”）专为高性能机器人设计，搭配
RoboMaster C620         无刷电机调速器（以下简称为“电调”，电调需另外购买）可以实现正弦驱动，
相比传统方波驱动具有更高的效率、机动性和稳定性。
物品清单
                                     × 1    7-Pin                                 × 1电机                      数据线

产品使用
连接电调
1.              7-Pin                        7-Pin 使用包装内的   数据线连接电调和电机的   数据端口。
2. 连接电机的三相输入接头与电调的三相动力线接头，确保相同颜色的线对应连接。请勿强行
 反插接头。

                                           2

                                           1

6

<!-- page: 9 -->

## 第 9 页

安装
参考以下尺寸，按需选择孔位进行安装。
单位：mm                                   4- M3  4 EQS

                                              26.8 -0.060
                                            35.00                                  13.00
                                            42.00
                                                                                             2- M3  4 EQS

                                  43.40                                             7.00 0.02
                                 30.10
                                              8.30 1
                               17.60
                                                                                                                                                                          -0.025-0.035
                                                                     5.00

                                            2.00

注意事项
       •         M3   电机安装孔为  螺纹孔，深度 4mm，请勿使用过长的螺丝，否则可能会顶到电机的
     PCB     板从而损坏电机。
       •   确保电机安装稳固。
       •   请在绕组最大允许温度范围内使用电机，避免长时间过热而损坏电机。
       •   请避免损伤线材，以防电机运行异常。
       •   使用时请勿触摸电机转子部分，避免割伤。
       •   避免杂物进入转子内部，以免导致转子运行异常。
       •   电机大功率运行时表面温度较高，请勿直接触碰以免烫伤。

                                                                7

<!-- page: 10 -->

## 第 10 页

性能与参数
负载特性曲线
    C620使用   电调做速度闭环控制
            n (rpm)  η (%)                                                                            I (A) P (W)

              8000    80                                      16  400                               η              7000    70                                      14  350                        n                                                 I              6000    60                                      12  300
              5000    50                           P        10  250
              4000    40                                       8   200
              3000    30                                       6   150
              2000    20                                       4   100
              1000    10                                       2   50
                0      0                                       0    0
                         0    0.05  0.10  0.15  0.20  0.25  0.30  0.35
                                                            T (N·m)

    C620使用   电调做电流闭环控制

            n (rpm)  η (%)                                                                           I (A) P (W)
               9000    90   n                                  18  450               8000    80                                      16  400
               7000    70                                      14  350                                                                                                        I
               6000    60   η                                  12  300
               5000    50                                      10  250                                             P               4000    40                                      8   200
               3000    30                                      6   150
               2000    20                                      4   100
               1000    10                                      2   50
                0      0                                      0    0
                         0   0.05  0.10  0.15  0.20  0.25  0.30  0.35
                                                           T (N·m)
工作范围图                 n (rpm)                                        125°C

                    10000

                     8000

                     6000

                     4000

                     2000

                         0        0.04      0.08      0.12       0.16
                持续工作范围                 T (N·m)
              n-转速，η-效率，I-电流，P-输出功率，T-扭矩
以上数据均在输入电压 24V、25℃ 室温、正常散热的实验环境下测得，仅供参考。实际使用时，请根据工作环境温度、
散热条件等实际情况来控制电机运行时间。
8

<!-- page: 11 -->

## 第 11 页

特征参数
电机
                         24 V额定电压
                             0.02 N·m/A转矩常数
                         390 rpm/V转速常数
                         260 rpm/N·m转速转矩梯度
                         49 ms机械时间常数
                            0.194相电阻                   Ω
                            0.097 mH相电感
使用环境温度          0℃至50℃
                         125绕组最高允许温度        ℃
                         7极对数
                         276 N额定动载荷
                         120 g电机重量
        C620电机搭配   电调在额定电压下使用*
                         9400 rpm空载转速
                               0.6 A空载电流
                         9085 rpm额定转速
                             0.16 N·m额定转矩（最大连续转矩）
                         10 A额定电流
                      83%最大效率
                             0.33 N·m峰转矩
                               2.5 A峰转电流
* 本表格中的数据均在实验环境下测得。电流均为电调的输入端电流，效率为电机与电调组成的驱动系统整体效率。
输出齿参数
                         17齿数
                               0.6模数
压力角                      20°
                             10.2分度圆直径
                            11.38齿顶圆直径
                             8.68齿根圆直径
                                -0.01变位系数
                         2跨齿数
                             2.79 -0.02-0.04公法线长度

                                                                9

<!-- page: 12 -->

## 第 12 页

如果您对说明书有任何疑问或建议，请通过DocSupport@dji.com 联系我们。If you have any questions about this document, please contact DJI by sending a message to DocSupport@dji.com.

                               RoboMaster and                        are trademarks of DJI.
                                               Copyright © 2025 DJI All Rights Reserved.

                                 RoboMaster                                                 Copyright和    © 2025     是大疆创新的商标。                                 大疆创新 版权所有

