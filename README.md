# MSPM0G3507 小车工程

本工程基于 TI MSPM0G3507 DriverLib 和 SysConfig，目前用于整合电机、编码器、IMU、OLED、灰度检测和 PID 控制。当前主程序支持 PB21/PB22/PB23/PB24/PB25 任务/配置按键：PB21 启动任务一，PB22 启动任务二，PB23 进入任务三圈数设定并启动任务三，PB24 用于上电等待阶段的灰度白底/黑线两段式阈值校准，PB25 进入低速选择模式；进入低速选择模式后，PB21 启动原 PB24 的 `150 mm/s` 任务二，PB22 启动原 PB25 的 `150 mm/s` 1 圈任务三。任务一为 A 到 B 直行、B 点停车提示、B 到 C 循迹并在 C 点停车；任务二为 A 到 B 直行、B 到 C 循迹、C 到 D 按 `-180°` yaw 目标直行、D 到 A 循迹并在 A 点停车；任务三按设定圈数执行 A-C-B-D-A 八字循环。

## 协作约定

- 本工程的说明文档、注释说明和后续沟通默认使用中文。
- 每次修改程序行为、引脚配置、模块接口、构建方式或调试流程后，都需要同步维护本 `README.md`。
- `README.md` 至少要保持这些内容准确：当前程序流程、引脚分配、模块目录、重要接口、构建/烧录注意事项。
- 临时测试代码可以保留在独立文件里，但主程序接入或移除测试代码时必须在 README 中说明。

## 当前程序状态

- `empty.c` 只保留主程序流程：初始化 SysConfig、编码器、电机、PID、UART、OLED、ICM42688 和灰度传感器，随后快速标定陀螺仪、等待 PB21/PB22/PB23 任务选择、PB24 灰度校准或 PB25 低速选择按键、启动 `control.c` 中的任务状态机，并周期发送调试数据。
- 主程序初始化完成后会先让小车保持静止并自动采约 2 秒陀螺仪 Z 轴角速度，得到一个基础零偏。标定期间电机和速度环不启动，UART0 会输出 `GYRO_CAL,...` 状态行。
- 快速标定完成后会把固定残差修正 `-0.00514078 dps` 叠加到 2 秒快速零偏上，随后进入任务选择按键等待；等待逻辑会先要求 PB21/PB22/PB23/PB24/PB25/PB26 全部松开稳定，再接受一次新的单键按下确认，避免上电或复位瞬间的低电平误启动。等待期间电机和速度环仍不启动，但 UART0 会每 50 ms 输出一行 `GYRO,gz=...,gzc=...,yaw=...,raw=...,ok=...`，OLED 会约每 100 ms 刷新 yaw、任务状态、任务三圈数 `N:`、灰度校准状态 `C:-/C:W/C:B/C:S/C:E/C:D` 和 8 路灰度 digital 位。
- PB21 确认后调用 `Control_StartTask1()`，PB22 确认后调用 `Control_StartTask2()`；PB23 第一次按下后进入任务三圈数设定，PB22 每按一次让圈数在 `1 -> 2 -> 3 -> 4 -> 1` 之间循环，PB23 再次按下后调用 `Control_StartTask3()` 正式启动。PB25 确认后进入低速选择模式，随后 PB21 调用 `Control_StartTask2LowSpeed()`，PB22 调用 `Control_StartTask3LowSpeedOneLap()`。任务启动时都会先把当前 yaw 和编码器清零，主循环每 10 ms 调用 `Control_Task10ms()` 推进任务状态。
- 当前进入速度环 PID 调试阶段：TimerG0 每 10 ms 触发一次，调用 `SpeedLoop_TimerIRQHandler()` 完成编码器测速、一阶低通滤波、左右轮速度 PID 更新和 `Motor_SetSpeed(left, right)` 输出。
- 速度环加入目标速度斜坡，`motorA.ref` / `motorB.ref` 表示最终目标，实际送入 PID 的目标默认每 10 ms 最多变化 `10 mm/s`；通过 PB25 低速选择模式启动的低速快捷任务会把斜坡降到每 10 ms `5 mm/s`，用于减小启动冲击。
- 任务参数集中在 `control.c`：默认基础速度 `220 mm/s`，PB25 低速选择模式下的快捷任务基础速度 `150 mm/s`，直行段启用陀螺仪 yaw 目标角修正；任务三的空白直线段改为先转目标角、清零编码器、按编码器平均绝对计数定距，A-C 为 105 cm，B-D 为 110 cm，末端摆正后低速找黑线。循迹段使用 8 路灰度黑度加权偏差更新左右轮速度目标。低速快捷任务在准备从找线段进入循迹前，会先原地慢速居中到中间两个灰度点同时压黑线，再放行循迹。
- 声光提示引脚暂定为 `PB0` 和 `PB1`，SysConfig 名称为 `Alert.LED` / `Alert.BUZZER`。LED 低电平点亮，蜂鸣器高电平响；提示开启时 PB0 拉低、PB1 拉高，提示关闭时 PB0 拉高、PB1 拉低。任务一 B 点停车 3 秒期间持续输出提示，任务一 C 点最终停车后提示 1 秒再关闭；任务二经过 B/C/D 时短提示一次，最终 A 点停车提示 1 秒；任务三每经过 C/B/D/A 均提示一次，最后 A 点停车提示 1 秒。
- 主循环每 50 ms 通过 UART0 输出 VOFA+ 波形数据：左轮斜坡目标速度、左轮当前速度、右轮斜坡目标速度、右轮当前速度；随后输出一行陀螺仪文本数据，包含 `gz`、`yaw`、原始 `raw` 和读取状态 `ok`。
- OLED 调试页约每 100 ms 刷新一次：正常等待或任务运行时显示当前 yaw、C 点锁存 yaw、当前任务状态、灰度读取状态、任务三圈数和 8 路灰度 digital 位；进入 PB24 灰度校准后切换到专用校准页，显示当前步骤和采样/保存结果；不再显示 8 路原始 ADC 捕获值。
- 灰度 OLED 测试入口 `GrayTest_Run()` 已从主程序移除；`gray_test.c` 作为独立测试代码仍参与当前构建，但未被主程序调用。
- 灰度采样当前已初始化，按键等待阶段和主循环都会周期调用 `GraySensor_Task()` 刷新 `g_gray_sensor.analog[]` 与 `digital`。速度环仍在 TimerG0 中断中运行，灰度/OLED 刷新只占用主循环时间。
- ADC 当前使用 DMA 批量采集灰度数据：每路切换地址后等待 10 us，随后由 DMA 连续搬运 40 个 ADC 结果到 `Gray_ADCValue[]`，CPU 等待本批完成后只负责求平均值。

## 目录说明

| 路径 | 说明 |
| --- | --- |
| `empty.c` | 当前主程序入口：初始化硬件，等待按键，启动控制任务，并转发周期调试与中断 |
| `control.c` / `control.h` | 任务状态机、OLED 调试页、按键等待服务和临时声光提示控制 |
| `empty.syscfg` | SysConfig 外设和引脚配置 |
| `BSP/GRAY` | 八路灰度传感器驱动和保留的 OLED 测试代码 |
| `BSP/MOTOR` | 电机 PWM、方向控制、速度环状态更新、直线目标 yaw 修正和 VOFA+ 数据格式化 |
| `BSP/ENCODER` | 编码器计数、10 ms 计数增量和轮子线速度计算 |
| `BSP/PID` | PID 控制模块、PID 参数设置和状态复位 |
| `BSP/OLED` | OLED SPI 显示驱动 |
| `BSP/ICM42688` | ICM42688 SPI 驱动和 yaw 测试代码 |
| `BSP/DELAY` | 基于 `delay_cycles` 的延时函数 |
| `BSP/UART` | UART0 初始化封装、收发接口、文本调参命令解析和 `printf` 重定向 |
| `BSP/KEY` | PB21/PB22/PB23/PB24/PB25/PB26 按键读取、任务选择、低速选择和灰度校准辅助函数 |
| `tools` | 上位机辅助和静态检查脚本，目前包含陀螺仪 Z 轴零偏采集脚本、任务三 README 规格检查脚本、灰度 Flash 校准检查脚本和 PB25 低速选择入口检查脚本 |

## 功能按键

当前使用任务选择、灰度校准和低速选择按键，电路均为按下接地，因此使用内部上拉输入：

| 按键 | 功能 | 未按下电平 | 按下电平 |
| --- | --- | --- | --- |
| PB21 / `Key.User` | 启动任务一 | 高电平 | 低电平 |
| PB22 / `Key.Key1` | 启动任务二 | 高电平 | 低电平 |
| PB23 / `Key.Key2` | 进入任务三圈数设定/确认启动任务三 | 高电平 | 低电平 |
| PB24 / `Key.Key4` | 灰度阈值校准：进入校准、采白底、采黑线、保存 Flash | 高电平 | 低电平 |
| PB25 / `Key.Key5` | 进入低速选择模式 | 高电平 | 低电平 |
| PB26 / `Key.Key3` | 当前不作为任务/校准入口 | 高电平 | 低电平 |

主程序在进入主循环前调用 `Control_WaitForStartSelection()`。该函数会先等待 PB21/PB22/PB23/PB24/PB25/PB26 全部松开稳定约 20 ms，之后再等待单个按键低电平稳定约 20 ms，确认任务选择、灰度校准或低速选择模式。若多个键同时按下，本次稳定计数会清零，避免误判。PB23 进入任务三设定状态后，PB22 用于增加圈数，PB23 用于确认启动；PB25 进入低速选择模式后，PB21 启动低速任务二，PB22 启动低速 1 圈任务三。

PB24 灰度校准只在任务启动前的等待阶段生效。第一次按 PB24 会进入灰度校准页，OLED 显示 `GRAY CAL`、`MODE:ON` 和 `KEY4 WHITE`，状态为 `C:W`；把 8 路灰度都放在白底上再按一次 PB24，UART 会输出 `GRAY_CAL,WHITE,...` 和 `GRAY_CAL,WHITE_OK`，OLED 显示 `WHITE OK` 并进入 `C:B`；把传感器放到黑线上再按一次 PB24，UART 会输出 `GRAY_CAL,BLACK,...` 和 `GRAY_CAL,BLACK_OK`，OLED 显示 `BLACK OK` 并进入 `C:S`；最后再按一次 PB24 才擦写片内 Flash，保存成功后 UART 输出 `GRAY_CAL,SAVED` / `GRAY_CAL,DONE`，OLED 显示 `SAVE OK` / `THR UPDATED`，并用本次白/黑 ADC 数组重新计算 `gray_white[]`、`gray_black[]` 和归一化系数。保存失败时 OLED 显示 `SAVE ERR`，状态为 `C:E`，可再次按 PB24 重试；校准未完成前任务启动按键会被忽略。

当前 SysConfig 生成代码里六个按键输入为 `DL_GPIO_RESISTOR_NONE`，因此固件会在 `Control_Init()` 中调用 `Key_EnablePullUps()`，强制给 PB21/PB22/PB23/PB24/PB25/PB26 打开内部上拉。若后续在 SysConfig 中直接配置上拉，这段代码保留也不会影响按下接地的按键逻辑。

## 灰度传感器

当前灰度传感器接法：

| 灰度模块信号 | MSPM0G3507 引脚 | 工程配置 |
| --- | --- | --- |
| OUT / Analog | PA27 | `ADC0_CH0`，`ADC12_0` |
| A0 / S0 | PB17 | `Gray_Address.PIN_0` |
| A1 / S1 | PB18 | `Gray_Address.PIN_1` |
| A2 / S2 | PB19 | `Gray_Address.PIN_2` |
| VCC | 5V | 与 MCU 共地 |
| GND | GND | 与 MCU 共地 |

注意：

- 当前 OUT 不使用 PA15 或 PA22；这些引脚接 GND/3.3V 不会影响灰度 ADC 读数。
- 灰度 OUT 现接到 PA27，PA27 已从右轮编码器释放并配置为 `ADC0_CH0`。
- 2026-05-18 实测白底 ADC：`1059, 1196, 1235, 1245, 1205, 1084, 1273, 1177`。
- 2026-05-18 实测黑线 ADC：`44, 53, 52, 72, 56, 67, 109, 53`。默认标定值已写入 `gray_sensor.c` 的 `s_gray_default_white[]` / `s_gray_default_black[]`。
- 上电时 `GraySensor_InitDefault()` 会先尝试从片内 Flash 的灰度校准扇区读取保存值；记录校验通过则加载保存的白/黑标定值，校验失败或未保存过则使用 `gray_sensor.c` 里的固定默认白/黑标定值。
- 当前主程序 OLED 不再显示 8 路原始 ADC，只显示 8 路 digital 位，白色为 `1`，黑色为 `0`。

PB24 校准值现在会保存到 MSPM0G3507 片内 Flash 的最后 1 KB 扇区，起始地址为 `0x0001FC00`。保存记录包含 magic、版本、8 路白底 ADC、8 路黑线 ADC 和校验值；启动时校验通过才加载，校验失败自动回退默认值。嘉立创天猛星文档里的 `AT24C02-EEPROM存储器` 属于常见模块移植手册的外接模块条目，不是当前工程已确认接入的板载存储，因此当前不依赖外置 EEPROM。由于 SysConfig 生成的 `device_linker.cmd` 仍覆盖完整 128 KB Flash，构建后需要运行 `tools/check_key3_flash_calibration.py` 检查 map 文件，确保程序镜像没有增长到 `0x0001FC00` 之后再烧录。

常用接口：

| 接口/变量 | 用途 |
| --- | --- |
| `GraySensor_InitDefault()` | 优先加载 Flash 保存标定，失败时用默认标定初始化全局灰度对象 |
| `GraySensor_LoadSavedCalibration(white, black)` | 从片内 Flash 读取已保存灰度白/黑标定值，校验失败返回 0 |
| `GraySensor_SaveCalibration(white, black)` | 擦写片内 Flash 最后 1 KB 扇区并保存灰度白/黑标定值 |
| `GraySensor_Task(&g_gray_sensor)` | 轮询 8 路灰度地址，每路用 DMA 采 40 点均值并刷新数据 |
| `g_gray_sensor.analog[8]` | 8 路原始 ADC 均值 |
| `g_gray_sensor.digital` | 8 路数字状态位 |
| `g_gray_sensor.normalized[8]` | 归一化后的灰度值 |
| `GraySensor_GetLastError()` | 读取最近一次 ADC 超时错误 |

## 任务一控制

任务一逻辑已从 `empty.c` 移到 `control.c`。按键启动后状态机依次执行：

1. `TASK1_AB`：清零 yaw 和编码器，AB 段直行，速度 `220 mm/s`，启用 `SpeedLoop_SetStraightMode(1)` 并把直线目标 yaw 设为 `0°`；灰度连续 3 个 10 ms 周期检测到黑线后判定到达 B 点。
2. `TASK1_STOP_B`：到达 B 点后停止速度环，`Alert.LED` / `Alert.BUZZER` 输出 3 秒。
3. `TASK1_TRACK_TO_C`：继续启动速度环并关闭 yaw 直线修正，进入灰度循迹；黑线偏差用 8 路 `normalized[]` 黑度加权计算，差速目标为 `motorA.ref = base * leftScale + correction`、`motorB.ref = base - correction`。
4. `TASK1_STOP_C`：循迹过程中连续 10 个 10 ms 周期检测到全白，即 `digital == 0xFF`，判定到达 C 点；停车前锁存当时 yaw 角度用于任务二 C 到 D 直线参考，随后停车并声光提示 1 秒，然后保持停车。

当前循迹参数为 `CONTROL_TRACK_KP = 0.45`、`CONTROL_TRACK_KD = 0.10`、`CONTROL_TRACK_CORRECTION_LIMIT = 160`、`CONTROL_TRACK_BLACK_SUM_MIN = 260`。如果 B/D 点黑线误触发，可增加 `CONTROL_BLACK_STABLE_TICKS` 或提高黑度阈值；如果 C/A 点全白停车太慢或太早，可调整 `CONTROL_WHITE_STABLE_TICKS`。

## 任务二控制

PB22 启动任务二，默认基础速度为 `220 mm/s`；PB25 进入低速选择模式后按 PB21 启动相同任务二逻辑，但基础速度改为 `150 mm/s`，速度斜坡降为每 10 ms `5 mm/s`。任务二复用任务一的直行和循迹参数，但 B 点不停下，只进行一次短声光提示：

1. `TASK2_AB`：清零 yaw 和编码器，A 到 B 按 `0°` yaw 目标直行；连续 3 个 10 ms 周期检测到黑线后判定经过 B 点，声光短提示一次。PB22 普通任务立即进入循迹；PB25+PB21 低速任务先进入 `TASK2_CENTER_TO_C`，等待中间两个灰度点同时压到黑线并稳定 3 个 10 ms 周期后再进入循迹。
2. `TASK2_TRACK_TO_C`：沿黑线循迹到 C；连续 10 个 10 ms 周期检测到全白，即 `digital == 0xFF`，判定经过 C 点，锁存当时 yaw 到 OLED 的 `CY:`，声光短提示一次。
3. `TASK2_STRAIGHT_TO_D`：从 C 点切回直线模式，调用 `SpeedLoop_SetStraightTargetYaw(-180.0f)`，按 `-180°` 目标角直行；重新稳定识别到黑线后判定经过 D 点，声光短提示一次。
4. `TASK2_TRACK_TO_A`：PB22 普通任务从 D 点重新进入灰度循迹；PB25+PB21 低速任务先进入 `TASK2_CENTER_TO_A` 做同样的中心两点压线确认，再进入循迹。再次稳定识别到全白后判定到达 A 点，停止速度环并声光提示 1 秒，随后保持停车。

任务二当前默认假设：D 点为 C 后直行遇到的黑线，A 点为 D 后循迹遇到的全白终点。如果实际地图标记不同，需要调整 `CONTROL_STATE_TASK2_STRAIGHT_TO_D` 和 `CONTROL_STATE_TASK2_TRACK_TO_A` 中的判定条件。

## 任务三控制

PB23 第一次按下进入任务三圈数设定状态，默认 `N = 1`。设定状态下，PB22 每按一次增加一圈，圈数在 `1~4` 之间循环；PB23 再次按下后正式启动任务三，默认基础速度为 `220 mm/s`。PB25 进入低速选择模式后按 PB22，直接启动基础速度 `150 mm/s` 的 1 圈任务三，不进入圈数设定，并把速度斜坡降为每 10 ms `5 mm/s`。OLED 调试页右侧 `N:` 显示当前设定圈数。

任务三从 A 点面向 B 点出发，按八字循环执行 A-C-B-D-A。空白直线段不再直接依赖“跑到黑线”，而是先用陀螺仪转向，再按编码器定距跑完大部分空白段，末端摆正后低速找黑线，提高正面进入端点的概率：

1. `TASK3_TURN_TO_C`：从 A 点起步先原地转向；第一圈到第四圈都转到 `-44°`，误差进入 `±1.5°` 并稳定约 80 ms 后进入下一段。
2. `TASK3_DRIVE_TO_C`：转向完成后清零左右编码器，按当前圈的 A-C yaw 目标直线跑 `1050 mm`。距离用左右轮累计绝对编码值平均值判断，避免单边轮误差影响定距。
3. `TASK3_ALIGN_C`：定距完成后停车并转回 `1.5°`，让车身正对 C 点黑线。
4. `TASK3_SEEK_C`：以 `120 mm/s` 低速正向找黑线；稳定识别到黑线后判定到达 C 点，声光短提示一次。PB23 普通任务直接进入 `TASK3_TRACK_TO_B` 循迹；PB25+PB22 低速任务先进入 `TASK3_CENTER_TO_B`，等待中间两个灰度点同时压到黑线并稳定 3 个 10 ms 周期后再进入循迹。
5. `TASK3_TRACK_TO_B`：沿半弧线循迹到 B 点；稳定识别到全白后判定经过 B 点，声光短提示一次，并切入 B-D 空白段。
6. `TASK3_TURN_TO_D` / `TASK3_DRIVE_TO_D` / `TASK3_ALIGN_D` / `TASK3_SEEK_D`：第一圈到第四圈都先转到 `225°`，清零编码器后定距直线跑 `1100 mm`，再转回 `178°` 并低速找黑线；稳定识别到黑线后判定到达 D 点。PB23 普通任务直接进入 `TASK3_TRACK_TO_A`；PB25+PB22 低速任务先进入 `TASK3_CENTER_TO_A` 做中心两点压线确认，再进入循迹。
7. `TASK3_TRACK_TO_A`：沿半弧线循迹到 A 点；稳定识别到全白后判定完成 1 圈。若当前圈数未达到设定 `N`，声光短提示一次后继续下一圈，从 `TASK3_TURN_TO_C` 重新开始；若达到 `N`，停止速度环并声光提示 1 秒，随后保持停车。

任务三当前默认假设：C/D 点为末端低速找线阶段遇到的黑线，B/A 点为循迹段遇到的全白区域。八字循环中的 yaw 目标使用连续角度：A-C 段第一圈到第四圈都先转 `-44°`，随后摆到 `1.5°`；B-D 段第一圈到第四圈都先转 `225°`，随后摆到 `178°`。第 4 圈沿用第三圈参数。任务三原地转向已降低速度，当前转向差速上限为 `40 mm/s`，最低转向差速为 `25 mm/s`。

## 引脚分配

### 电机

| 功能 | MSPM0G3507 引脚 | SysConfig 名称 |
| --- | --- | --- |
| PWM 通道 0 | PA0 | `PWM_Motor C0` |
| PWM 通道 1 | PA1 | `PWM_Motor C1` |
| AIN1 | PB4 | `Motor.AIN1` |
| AIN2 | PB5 | `Motor.AIN2` |
| BIN1 | PB15 | `Motor.BIN1` |
| BIN2 | PB16 | `Motor.BIN2` |

### 声光提示

声光提示引脚目前只是临时定义，后续按实际硬件连接修改 `empty.syscfg` 和 `control.c` 即可。LED 低电平点亮，蜂鸣器高电平响。

| 功能 | MSPM0G3507 引脚 | SysConfig 名称 | 有效电平 |
| --- | --- | --- | --- |
| 提示灯 | PB0 | `Alert.LED` | 低电平点亮 |
| 蜂鸣器 | PB1 | `Alert.BUZZER` | 高电平响 |

### 编码器

| 功能 | MSPM0G3507 引脚 | SysConfig 名称 |
| --- | --- | --- |
| 左 A | PA24 | `Encoder.Pin_Left_A` |
| 左 B | PA25 | `Encoder.Pin_Left_B` |
| 右 A | PA28 | `Encoder.Pin_Right_A` |
| 右 B | PA31 | `Encoder.Pin_Right_B` |

说明：左右轮编码器方向均按“小车往前运动时计数、速度为正”的约定处理。新 PCB 调试时先发现往前推车 `CL` 为正、`CR` 为负，因此右轮编码器方向已在软件中反向修正；随后确认左 B 相恢复后，左轮正转为负编码、反转为正编码，因此左轮编码器方向也已在软件中反向修正。修正后往前推车时 `CL/CR` 和 `VL/VR` 都应为正。`Encoder_GetLeftSpeed()` / `Encoder_GetRightSpeed()` 返回 10 ms 编码器计数增量；`Encoder_GetLeftLinearSpeed()` / `Encoder_GetRightLinearSpeed()` 返回轮子线速度，单位为 mm/s。旧接口 `Encoder_GetLeftRPM()` / `Encoder_GetRightRPM()` 当前兼容返回同样的 mm/s，调速度环时可直接使用，但需要注意它不再是 RPM。

速度环中断中保留 10 ms 原始测速周期，并对线速度做一阶低通滤波：

```c
filtered += 0.35f * (raw - filtered);
```

PID 输入和 VOFA+ 当前打印值均使用滤波后的速度，用于减小编码器瞬时测速和机械纹波带来的高频抖动。速度目标使用斜坡限制，默认每 10 ms 变化 `10 mm/s`；PB25 低速选择模式启动的任务运行期间使用每 10 ms `5 mm/s`：

```c
rampRef = SpeedLoop_RampToTarget(rampRef, motor.ref);
```

直线模式下速度环使用 `SpeedLoop_SetStraightTargetYaw(targetYawDeg)` 设置目标角。中断中用 `ICM42688_YawGetDeg() - targetYawDeg` 计算 yaw 误差，再按 `STRAIGHT_YAW_KP` 转成左右轮差速修正。任务一 AB 段和任务二 AB 段目标为 `0°`，任务二 C 到 D 段目标为 `-180°`。

### ICM42688

| ICM42688 信号 | MSPM0G3507 引脚 | SysConfig 名称 |
| --- | --- | --- |
| SCLK | PA12 | `SPI_ICM42688 SCLK` |
| MOSI / SDI | PA14 | `SPI_ICM42688 PICO` |
| MISO / SDO | PA13 | `SPI_ICM42688 POCI` |
| CS | PA16 | `ICM.ICM_CS` |
| VCC | 3.3V | - |
| GND | GND | - |

陀螺仪 yaw 直线控制前，需要先标定 Z 轴角速度零偏。当前固件会在上电后、按下任务选择按键前自动用约 2 秒数据快速得到一个基础零偏，避免 yaw 立即快速漂移。

标定期间请让小车静止放在水平面，串口会输出类似：

```text
GYRO_CAL,KEEP_STILL
GYRO_CAL,QUICK,samples=100,bias=0.884146
GYRO_CAL,QUICK_DONE,samples=400,bias=0.889349
```

当前固件在复位初始化完成后、按下任务选择按键启动车辆前，就会持续发送：

```text
GYRO,gz=0.884146,gzc=-0.005203,yaw=0.012,raw=29,ok=1
```

其中 `gz` 单位为 dps，是 ICM42688 原始 Z 轴角速度换算后的值，未减零偏；`gzc` 是已经减掉当前零偏并经过当前 yaw 模式处理后的角速度残差；`yaw` 是对 `gzc` 积分得到的角度。当前 yaw 不再限制到 `-180°~+180°`，而是连续累加，允许出现 `-180°`、`-360°`、`-720°` 等多圈角度，方便后续多圈任务。当前运行零偏来自 `2 秒快速标定值 + ICM42688_YAW_QUICK_BIAS_OFFSET_DPS`，其中固定残差 `ICM42688_YAW_QUICK_BIAS_OFFSET_DPS` 当前为 `-0.00514078f`。`ICM42688_YAW_DEFAULT_BIAS_DPS` 只作为初始化默认值，UART `G bias` 或 `GA residual` 命令仍可用于调试时在线更新零偏。

当前固化参数位于 `BSP/ICM42688/ICM42688.h`：

```c
#define ICM42688_YAW_DEFAULT_BIAS_DPS             (0.87547255f)
#define ICM42688_YAW_QUICK_BIAS_OFFSET_DPS        (-0.00514078f)
#define ICM42688_YAW_QUICK_CALIB_MS               (2000U)
```

最近一次静止实测中，快速标定后脚本测得 `gzc` 残差均值约 `-0.00271513 dps`，因此将固定残差修正从 `-0.00242565f` 累加到 `-0.00514078f`。若后续温度或模块更换导致静止持续正漂，说明 bias 偏小，可把 offset 调大一点；若持续负漂，说明 bias 偏大，可把 offset 调小一点。每次建议只调 `0.001~0.003 dps`。

比赛或实际运行时不需要每次跑上位机脚本。上位机脚本 `tools/gyro_bias_calibrate.py` 只用于调试阶段重新测量固定残差；如需重新测，可在 2 秒快速标定后先用 `YM 0` 切到纯减零偏积分模式，再采 2000 个 `gzc` 残差求平均，并把结果写回 `ICM42688_YAW_QUICK_BIAS_OFFSET_DPS`：

```powershell
python tools/gyro_bias_calibrate.py --port COM8 --baud 115200 --precmd "YM 0" -n 2000 --discard 100 --require-gz --field gzc --send-to-mcu --send-mode add
```

脚本会先发送 `YM 0` 并短暂等待，避免死区、低通或运行时 bias 学习影响固定残差测量；采样完成后再发送 `GA <residual_dps>` 命令，固件会临时执行 `bias = bias + residual` 并清零 yaw。固件后续积分 yaw 时做的是 `gz - bias`；如果残差是负数，本质上就是把当前零偏减小。确认残差有效后，可把该值固化到 `ICM42688_YAW_QUICK_BIAS_OFFSET_DPS`。

如果打开 Python 串口会导致板子复位，`YM 0` 可能会发在初始化或快速标定期间而被忽略；这种情况下可以加长打开串口后的等待时间，例如 `--open-delay 5`，让脚本等快速标定结束后再发 `YM 0`。

如果只想离线验证零偏和噪声水平，不发回 MCU，可运行：

```powershell
python tools/gyro_bias_calibrate.py --port COM8 --baud 115200 --precmd "YM 0" -n 1500 --discard 100 --require-gz --field gzc
```

如果串口里同时混有 VOFA+ 速度 CSV 行，必须加 `--require-gz`，否则脚本会把速度数字误当成陀螺仪数据，算出的零偏会明显偏大。

需要实时看静止零偏和 yaw 漂移曲线时，可以运行自带 `tkinter` 界面的脚本：

```powershell
python tools/gyro_live_view.py --port COM8 --baud 115200 -n 1500
```

如果 MCU 输出为 CSV，例如 `gx,gy,gz`，并且 gz 是第 3 列，使用：

```powershell
python tools/gyro_bias_calibrate.py --port COM8 --baud 115200 -n 1500 --column 2
```

如果 MCU 输出的是 ICM42688 原始 `gz` 计数，当前 +/-1000 dps 量程可用 `--scale 32.8` 转成 dps：

```powershell
python tools/gyro_bias_calibrate.py --port COM8 --baud 115200 -n 1500 --column 0 --scale 32.8
```

脚本会保存 `gyro_z_bias.json`，其中 `bias_dps` 就是后续固件积分 yaw 前需要减掉的零偏值。加 `--monitor` 可以在标定后持续打印减零偏后的 `gz` 和积分得到的 yaw，方便观察静止漂移是否减小。

### OLED

| OLED 信号 | MSPM0G3507 引脚 | SysConfig 名称 |
| --- | --- | --- |
| SCLK | PB9 | `SPI_OLED SCLK` |
| MOSI | PB8 | `SPI_OLED PICO` |
| MISO | PB7 | `SPI_OLED POCI` |
| RES | PB10 | `OLED.RES` |
| DC | PB11 | `OLED.DC` |
| CS | PB14 | `OLED.CS` |

### UART0

| 功能 | MSPM0G3507 引脚 | SysConfig 名称 |
| --- | --- | --- |
| RX | PA11 | `UART0 RX` |
| TX | PA10 | `UART0 TX` |

### 功能按键

| 功能 | MSPM0G3507 引脚 | SysConfig 名称 |
| --- | --- | --- |
| 任务一启动 | PB21 | `Key.User` |
| 任务二启动 | PB22 | `Key.Key1` |
| 任务三设定/启动 | PB23 | `Key.Key2` |
| 灰度白/黑阈值校准 | PB24 | `Key.Key4` |
| 低速选择模式 | PB25 | `Key.Key5` |
| 低速任务二确认 | PB25 后按 PB21 | `Key.Key5` 后 `Key.User` |
| 低速任务三 1 圈确认 | PB25 后按 PB22 | `Key.Key5` 后 `Key.Key1` |

UART0 当前配置为 115200 baud，8N1。`BSP/UART/uart.c` 提供 `UART_Init()`、`UART_SendByte()`、`UART_SendString()`、`UART_ReadByte()` 和接收环形缓冲，并把 `printf` 重定向到 UART0。由于当前 TI 运行库中 `printf` 的数字输出可能进入 CCS CIO 通道，VOFA 数据和调参回显统一使用 `snprintf` 先格式化到缓冲区，再用 `UART_SendString()` 发送。

VOFA+ 接收格式为 `左斜坡目标,左当前,右斜坡目标,右当前` 四路波形。当前主循环调用 `SpeedLoop_FormatVofaLine()` 格式化整行，再通过 `UART_SendString()` 发送，避免部分运行库 `printf` 输出数字时不走 UART 重定向：

```c
SpeedLoop_FormatVofaLine(vofaLine, sizeof(vofaLine));
UART_SendString(vofaLine);
```

速度环调试阶段支持通过 UART0 发送文本命令在线调参，命令解析位于 `BSP/UART/uart.c` 的 `UART_ProcessInput()`，命令以回车或换行结束。发送命令时建议先暂停 VOFA 接收显示或忽略 `OK/ERR/CFG` 文本行，纯数字 CSV 行仍用于波形显示。

| 命令 | 说明 |
| --- | --- |
| `P kp ki kd` | 同时设置左右轮速度环 PID，并清空左右轮 PID 历史状态 |
| `PL kp ki kd` | 只设置左轮速度环 PID，并清空左轮 PID 历史状态 |
| `PR kp ki kd` | 只设置右轮速度环 PID，并清空右轮 PID 历史状态 |
| `R left right` | 设置左右轮目标线速度，单位 mm/s，并清空左右轮 PID 历史状态 |
| `S` | 停止：左右目标速度归零，清空 PID 状态，并立即输出 0 PWM |
| `G bias` | 设置陀螺仪 Z 轴角速度零偏，单位 dps，并清零 yaw；通常由 `gyro_bias_calibrate.py --send-to-mcu` 自动发送 |
| `GA residual` | 在当前陀螺仪零偏上追加一个残差，单位 dps，并清零 yaw；用于 2 秒快速标定后的 2000 点残差均值修正 |
| `Q` | 查询当前左右轮 PID 参数和目标速度 |

示例：先发 `P 0.8 0 0` 设置比例参数，再发 `R 100 100` 给左右轮 100 mm/s 目标速度；需要停车时发送 `S`。

## 构建步骤

1. 在 CCS 中打开工程。
2. 修改 `empty.syscfg` 后，保存文件以重新生成 `ti_msp_dl_config.c/.h`。
3. 右键工程执行 `Refresh`。
4. 执行 `Clean Project`。
5. 执行 `Build Project`。
6. 烧录 `Debug/empty_LP_MSPM0G3507_nortos_ticlang.out`。

命令行构建可使用：

```powershell
& 'D:\TI\CCS\ccs2050\ccs\utils\bin\gmake.exe' -C Debug all
```

任务三 README 规格回归检查可使用：

```powershell
python tools/check_task3_readme_spec.py
```

灰度 Flash 校准和临时诊断清理回归检查可使用：

```powershell
python tools/check_key3_flash_calibration.py
```

PB24/PB25 低速任务入口回归检查可使用：

```powershell
python tools/check_low_speed_keys.py
```

## 烧录失败恢复

如果 CCS Theia 下载时报类似下面的错误：

```text
Trouble Writing Register PC: Verification of RAMCode failed @ address 0x202001E8
Failed to prepare for programming. Failed to download RAMCode
```

通常说明芯片里已有程序影响了调试器下载 RAMCode，常见于刚烧过其它例程、程序占用 SRAM/DMA/低功耗或调试连接不稳定时。当前验证可用的恢复方式是用 J-Link Commander 先擦除主 flash：

```powershell
& 'D:\32keil\ARM\Segger\JLink.exe' -CommanderScript 'C:\Users\WeiBu\workspace_ccstheia\erase_mspm0g3507.jlink'
```

脚本内容：

```text
si SWD
speed 1000
device MSPM0G3507
connect
r
h
erase
r
q
```

擦除时可能出现 `0x41C00000 sector is locked`，这是配置/保护相关区域，不代表主程序区没有擦掉。可用下面的脚本检查主 flash 起始地址：

```powershell
& 'D:\32keil\ARM\Segger\JLink.exe' -CommanderScript 'C:\Users\WeiBu\workspace_ccstheia\read_mspm0_flash0.jlink'
```

若输出类似 `00000000 = FFFFFFFF FFFFFFFF FFFFFFFF FFFFFFFF`，说明主 flash 已经为空，可以回 CCS Theia 重新烧录。若仍失败，断电重插板子、降低 SWD/JTAG 速度到 `1000 kHz` 或更低，必要时使用 connect under reset。

## 调试记录

- 灰度 OUT 已改为使用 `PA27/ADC0_CH0`。
- 右轮编码器 AB 相已改到 `PA28/PA31`，释放 `PA27` 给灰度 ADC。
- `PA15` 测 GND/3.3V 读数不变，是因为当前程序并未采样 PA15。
- 灰度读数已验证可随目标变化：未识别约一百多，识别灰色约五六百。
- 灰度测试 OLED 界面已移除，后续进入循迹 PID 调试。
- 新 PCB 电机诊断时发现往前推车 `CL/VL` 为正、`CR/VR` 为负，已在 `encoder.c` 中反转右轮编码器软件计数方向；后续验证标准为往前推车时 `CL/CR` 和 `VL/VR` 都为正。
- 新 PCB 左轮诊断时发现电机不转来自线松；左编码器正反转都会累加时，优先检查左 B 相线、焊点、插头和 PA25 引脚连接。
- 左 B 相恢复后，实测左轮正转为负编码、反转为正编码，已在 `encoder.c` 中反转左轮编码器软件计数方向；最终验证标准仍为小车往前运动时 `CL/CR` 和 `VL/VR` 都为正。

## ICM42688 滤波接入记录

- `BSP/ICM42688/icm.c` / `icm.h` 现在作为 ICM42688 yaw 滤波模块参与构建，不再依赖外部 `dmx_icm42688.h`。
- `ICM42688_YawUpdate()` 仍然由 `ICM42688.c` 负责 SPI 读取原始 Z 轴角速度和减零偏，然后把 `correctedGz` 输入 `ICM_YawFilterUpdate()`。
- 当前默认 yaw 链路为 `0.05 dps` 死区，不启用低通滤波；`GYRO` 调试行里的 `gzc` 表示当前 yaw 模式处理后的校正角速度，`yaw` 表示用该角速度积分得到的连续 yaw 角度。低通滤波仍可通过 UART `YM 2` 临时切换测试。
- OLED 调试页的 `Y:` / `CY:` 现在显示到 3 位小数，UART `GYRO` 行的 `yaw` 现在显示到 5 位小数，便于观察滤波后的微小漂移。
- 默认关闭运行时 bias 学习，避免小车转弯、循迹或八字任务中把真实角速度误学习成零偏。
- 快速标定前会先执行 `ICM42688_YAW_WARMUP_MS = 1500 ms` 预热丢弃样本，随后再进行 2 秒 Z 轴零偏平均，减少上电早期 bias 未稳定造成的漂移。
- UART 支持 yaw 优化分步测试命令：`YM 0` 纯减零偏积分，`YM 1` 加死区，`YM 2` 加死区和低通，`YM 3` 加死区、低通和运行时 bias 学习；`YQ` 查询当前 yaw 模式、bias、runtime bias、滤波角速度和 yaw。每次 `YM` 会清零 yaw，便于逐项比较。
