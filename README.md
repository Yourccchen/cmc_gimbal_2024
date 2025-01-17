# 2024英雄云台代码

* 本代码为2024版本的英雄云台代码，该文档为开发历程以及代码结构说明，方便后续修改和移植

## 硬件信息：

主控: Robomaster C板

云台Yaw轴、Pitch轴电机：达妙4310电机

摩擦轮电机:去掉减速箱后的大疆M3508电机

拨弹轮电机:大疆M3508电机

底盘电机:大疆M3508电机

IMU:CH100



**串口使用：**

| 串口 | UART1    | USART3  | USB    |
|----|----------|---------|--------|
|用途  | Vofa+调试 | 遥控器接收数据 | 与上位机通信 |

**CAN_ID表：**

**CAN1：**

| ID   | 0x201    | 0x202    | 0x02 0x03       |
| ---- | -------- | -------- | --------------- |
| 电机 | 左摩擦轮 | 右摩擦轮 | Pitch发 Pitch收 |

**CAN2：**

| ID   | 0x01 0x00   | 0x203  | 0x206  | 0x207  |
| ---- | ----------- | ------ | ------ | ------ |
| 电机 | Yaw发 Yaw收 | 拨弹轮 | 开镜上 | 开镜下 |



## 主要内容：

- 云台PID位置、速度环控制

> 本工程的PID计算中，速度环计算用的是真实转速(rpm)，位置环计算用的角度(°)。
>
> 在赋目标值时，直接赋自己期望的速度值和自己期望的角度值。

- 摩擦轮、拨弹轮的控制
- 与遥控器的连接
- 与底盘的通信
- 视觉通信

## 代码说明：

### 1.云台与底盘CAN通信协议

云台`CAN2`与底盘`CAN2`相连，具体协议如下表所示：

| 标识符   | DATA[0]&[1] | DATA[2]&[3] | DATA[4]&[5] | DATA[6] | DATA[7] |
|-------|-------------|-------------|-------------|-------------|---------|
| 0x401 | x轴速度(%)     | y轴速度(%)     | z轴速度(%)     | car_mode | is_aimbot |
| 0x402 | yaw角        | pitch角      | servo_status、fric_status | rammer_status | redraw_status |
| 0x405 | cool_rate       | cool_limit  | shoot_spd_max  |  color |     smallOrBig  |
| 0x407 | shoot_frequency |             |             |             |         |

### 2.云台与上位机串口（USB）通信协议



### 3.控制算法切换

本代码中包含多种控制算法的函数，其中包括ADRC,PID,PID(matlab),具体使用说明如下：

切换算法：在gimbal.h的初始化类中将各类的algorithm进行更改即可

#### 3.1 PID

在gimbal.h中初始化各电机PID的参数

在Gimbal_ParamChoose()函数中设置不同反馈模式下云台PID的参数

位置式PID计算

```c++
SetKpid() //设置PID参数
SetMax()  //设置限幅
PID_GetPositionPID()//位置式PID
```

增量式PID计算

```c++
SetKpid() //设置PID参数
SetMax()  //设置限幅
PID_GetIncrementalPID()//增量式PID
```

#### 3.2 PID(MATLAB）

用matlab生成的PID,自带微分低通滤波以及clamping积分抗饱和方法，相对于纯PID上限更高，更容易调参。

定位到PID_stm32中的PIDC.h，定义了如下变量：

```c
typedef struct {
real_T angle_set;                    /* '<Root>/angle_set' */
real_T angle_feedback;               /* '<Root>/angle_feedback' */
real_T speed_feedback;               /* '<Root>/speed_feedback' */
real_T P_P;                          /* '<Root>/P_P' */
real_T P_I;                          /* '<Root>/P_I' */
real_T P_D;                          /* '<Root>/P_D' */
real_T P_N;                          /* '<Root>/P_N' */
real_T S_P;                          /* '<Root>/S_P' */
real_T S_I;                          /* '<Root>/S_I' */
real_T S_D;                          /* '<Root>/S_D' */
real_T S_N;                          /* '<Root>/S_N' */
real_T P_MO;                         /* '<Root>/P_MO' */
real_T P_LO;                         /* '<Root>/P_LO' */
real_T S_MO;                         /* '<Root>/S_MO' */
real_T S_LO;                         /* '<Root>/S_LO' */
} ExtU;
typedef struct {
  real_T Current;                      /* '<Root>/Current' */
} ExtY;
```

其中ExtU即为该系统的输入,ExtY即为该系统的输出，在需要使用PID的地方extern一下，应用其中的子变量即可。然后在该文件中使用如下函数：

```c
PID_step(1);
```



**注意：使用Yaw轴电机机械角度反馈请勿开小陀螺或者随动模式！**

### 4.键位设置

#### 4.1.英雄遥控器键位设置

- 左摇杆：车体前后左右平移

- 右遥杆：云台pitch和yaw运动

- 左拨杆（控制模式）：上：开摩擦轮     ；中：关摩擦轮    ；下：键鼠模式 

- 右拨杆（车体模式）：上：小陀螺模式 ；中：随动模式    ；下：自瞄模式

- 左侧拨盘：往下负责拨弹轮的转动（摩擦轮开时才有效）

  **注意：遥控器拨杆自上至下是1、3、2！！**

#### 4.2.键盘键位设置

* 鼠标左键：发射——点按
* 鼠标右键：开启打带数字的自瞄——长按 不用
* WASD：上下左右——长按
* Q：倍镜角度控制（顺时针） 不用
* E：倍镜角度控制（逆时针） 不用
* R：开启自瞄装甲板——点按
* F：开关摩擦轮——点按
* G：切换为保护——点按
* Z：
* X：小陀螺和随动模式切换——点按
* C：
* V：云台反转——点按
* B：重画UI——点按
* Shift：冲刺（超级电容）——长按
* Ctrl：按下进入自由模式，松开回到随动



#### 4.3.控制模式切换

遥控器左侧拨杆往下即可为键鼠模式

遥控器左侧拨杆到中间即为遥控器操作模式
