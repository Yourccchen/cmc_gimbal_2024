//
// Created by DELL on 2023/9/23.
//

#include "gimbalc.h"
#include "laser.h"

bool GIMBAL=0;
cGimbal gimbal; //定义云台总类
extern ReceivePacket vision_pkt;

/**
  *@brief   云台控制总函数
  */
void cGimbal::Gimbal_ControlLoop()
{
    //处理电机信息
    for (auto &motor: motors)
        motor.UpdateMotorInfo();

    //喂狗
//    FeedDog();

    //位置环
    Gimbal_PosC();

    //遥控器控制云台(顺序不可换!)
    Gimbal_ControlWithRC();

    //速度环（发电流用）
    Gimbal_SpeedC();
}

/**
  *@brief   遥控器控制云台主函数
  */
void cGimbal::Gimbal_ControlWithRC(void)
{
    ///云台控制相关部分///
    //获取遥控器数据并做卡尔曼滤波
    MousePih= KalmanFilter(&Gimbal_MouseX,portSetPihSpeed());
    MouseYaw= KalmanFilter(&Gimbal_MouseY,portSetYawSpeed());

    RCPih=KalmanFilter(&Gimbal_PihAngle,portSetPihSpeed());
    RCYaw=KalmanFilter(&Gimbal_YawAngle,portSetYawSpeed());
    //将遥控器数据转换为Yaw、Pih目标值
    if(CarMode!=PROTECT && CarMode!=ZIMIAO && ProtectFlag!=OFFLINE && ControlMode!=KEY_MODE)
    {
        //保护模式和自瞄模式下不允许读取遥控器键值，防止回到正常时疯转
        PihTarget += RCPih * 2 / 1000;
        YawTarget += RCYaw * 5 / 1000;
    }

    if(CarMode!=PROTECT && CarMode!=ZIMIAO && ProtectFlag!=OFFLINE && ControlMode==KEY_MODE && ZimiaoFlag==CLOSEZIMIAO)
    {
        //保护模式和自瞄模式下不允许读取遥控器键值，防止回到正常时疯转
        PihTarget += MousePih * 2 / 1000;
        YawTarget += MouseYaw * 5 / 1000;
    }

    ///底盘及车体控制相关部分///
    CarMode=portSetCarMode();                //右侧拨杆控制车体模式，赋值给CarMode
    ControlMode=portSetControlMode();        //左侧拨杆控制控制模式，赋值给ControlMode
    ShootMode=portSetShootMode();            //左侧拨盘控制射击模式，赋值给ShootMode
    portIsZimiao();

    vx=portSetVx();                          //控制底盘水平方向运动
    vy=portSetVy();                          //控制底盘垂直方向运动

    //车体模式设置
    Gimbal_CarMode(CarMode);
    //控制模式设置
    Gimbal_ControlMode(ControlMode);
    //射击模式设置
    Gimbal_ShootMode(ShootMode);

    //底盘通信循环
    Chassis_ComLoop(vx,vy,vz,CarMode,gimbal.ZimiaoFlag);
}

/**
  *@brief  云台车体模式设置
  *@param  car_mode决定三个模式，ZIMIAO自瞄模式，SUIDONG随动模式，TUOLUO小陀螺模式，PROTEC保护模式
  */
void cGimbal::Gimbal_CarMode(int8_t car_mode)
{
    ///***********遥控器掉电保护**************///
    Online_Check();
    ///*************************************///

    if(ProtectFlag==OFFLINE)
    {
        car_mode=PROTECT;
    }

    switch(car_mode)
    {
        case TUOLUO:
        {
            vz=100.0f;
            break;
        }
        case SUIDONG:
        {
            ///优弧劣弧处理
            if(portSetFree()==1)//判断是否进入自由模式（按下ctrl）
            {
                vz=0;
            }
            else
            {
                if (ChassisYawTarget - motors[YawMotor].RealAngle_Ecd > 180)
                    ChassisYawTarget -= 360;            //加减2π
                if (motors[YawMotor].RealAngle_Ecd - ChassisYawTarget > 180)
                    ChassisYawTarget += 360;
                vz= -motors_pid[ChassisYaw].PID_Out;    //控制底盘转动速度0
//                vz=0;
            }
            break;
        }
        case ZIMIAO:
        {
            if (ChassisYawTarget - motors[YawMotor].RealAngle_Ecd > 180)
                ChassisYawTarget -= 360;            //加减2π
            if (motors[YawMotor].RealAngle_Ecd - ChassisYawTarget > 180)
                ChassisYawTarget += 360;
            vz= -motors_pid[ChassisYaw].PID_Out;    //控制底盘转动速度0
            if (vision_pkt.packet_id != Last_ID)
            {
                lowfilter_pih.Init(50,0.005);//低通滤波器，设置截止频率和采样周期
                lowfilter_yaw.Init(20,0.005);//低通滤波器，设置截止频率和采样周期

                PihTarget =-lowfilter_pih.Filter(vision_pkt.offset_pitch)
                           +mi_motor[0].Angle;
//                    YawTarget = KalmanFilter(&ZIMIAO_Yaw,vision_pkt.offset_yaw)
                YawTarget =lowfilter_yaw.Filter(vision_pkt.offset_yaw)
                           + motors[YawMotor].RealAngle_Imu;
                Last_ID = vision_pkt.packet_id;
            }
            break;
        }
        case PROTECT:
        {
            //摩擦轮、拨弹轮输出为0
            shoot.ShootSpeedClean();
            //YAW轴输出为0
            motors_pid[YawSpd].PID_Out=0;
            Pid_Out.YawCurrent = 0;
            YawTarget=motors[YawMotor].RealAngle_Imu;
            motor[Motor1].ctrl.kd_set=0;
            //PIH轴输出为0
            motors_pid[PihPos].PID_Out=0;
            motors_pid[PihSpd].PID_Out=0;
            Pid_Out.PihCurrent=0;
            PihTarget=mi_motor[0].Angle;
            //底盘目标值给0
            vx=vy=vz=0;
            Laser_Off();
            break;
        }
    }
}

/**
  *@brief  云台控制模式设置
  *@param  control_mode决定三个模式，KEY_MODE键鼠操作，CLOSEFRIC关闭摩擦轮，OPENFRIC开启摩擦轮
  */
void cGimbal::Gimbal_ControlMode(int8_t control_mode)
{
    switch (control_mode)
    {
        case KEY_MODE:
        {
            ControlMode=KEY_MODE;
            if(ZimiaoFlag==OPENZIMIAO)
            {
                if (ChassisYawTarget - motors[YawMotor].RealAngle_Ecd > 180)
                    ChassisYawTarget -= 360;            //加减2π
                if (motors[YawMotor].RealAngle_Ecd - ChassisYawTarget > 180)
                    ChassisYawTarget += 360;
                vz= -motors_pid[ChassisYaw].PID_Out;    //控制底盘转动速度0

                if (vision_pkt.packet_id != Last_ID)
                {
                    lowfilter_pih.Init(50,0.005);//低通滤波器，设置截止频率和采样周期
                    lowfilter_yaw.Init(50,0.005);//低通滤波器，设置截止频率和采样周期

                    PihTarget =-lowfilter_pih.Filter(vision_pkt.offset_pitch)
                               +mi_motor[0].Angle;
//                    YawTarget = KalmanFilter(&ZIMIAO_Yaw,vision_pkt.offset_yaw)
                    YawTarget =lowfilter_yaw.Filter(vision_pkt.offset_yaw)
                               + motors[YawMotor].RealAngle_Imu;
                    Last_ID = vision_pkt.packet_id;
                }
            }
            break;
        }

    }
}

/**
  *@brief  云台射击模式设置
  *@param  shoot_mode决定两个模式，OPENFRIC 0摩擦轮打开状态，CLOSEFRIC 1摩擦轮关闭状态
  */
void cGimbal::Gimbal_ShootMode(int8_t shoot_mode)
{
    switch(shoot_mode)
    {
        case CLOSEFRIC:
        {
            shoot.ShootSpeedClean();//摩擦轮与拨弹轮的速度清零
            break;
        }
        case OPENFRIC:
        {
            ///射击相关部分///
            shoot.Shoot_ControlLoop(); //射击主循环，主要包括堵转检测、热量检测、速度设置、参数选择等
            break;
        }
    }
}

/**
  *@brief  底盘控制循环
  *@param  水平运动速度vx，垂直运动速度vy，旋转速度vz,保护模式protect_mode,车体模式car_mode,自瞄模式aimbot_mode
  */
void cGimbal::Chassis_ComLoop(float vx,float vy,float vz,int8_t car_mode,int8_t aimbot_mode)
{
    //向底盘发送速度
    if(count_time_send==0)
    {
        CAN_ChasisSendSpd(vx,vy,vz,car_mode,aimbot_mode);
    }
    //向底盘发送云台信息
    if(count_time_send==1)
    {
        CAN_ChasisSendMsg(-(motors[YawMotor].RealAngle_Ecd-ChassisYawTarget),motors[PihMotor].RealAngle_Imu,0,gimbal.shoot.fric_flag,0,portSetRedraw());
    }
}


///电机位置环
void cGimbal::Gimbal_PosC()
{
    PID_step(1); //进行MATLAB的PID计算，两套算法并行计算，只在最后CAN发送电流的时候作区分

    //根据控制模式选择云台参数
    if(CarMode==ZIMIAO||ZimiaoFlag==OPENZIMIAO)
    {
        Gimbal_ParamChoose(ZIMIAO);
    }
    else
        Gimbal_ParamChoose(IMU_MODE);

    //Pitch轴限幅选择
    switch(motors[PihMotor].Which_Mode)
    {
        case ECD_MODE:
            Pitch_EcdLimit(PihTarget); //编码器
            break;
        case IMU_MODE:
            Pitch_ImuLimit(PihTarget); //陀螺仪
            break;
        case CYBERGEAR:
//            Pitch_MILimit(PihTarget); //小米电机
            break;
    }

    portSetScope();//倍镜角度控制
    //通过遥控器设置Pih轴、Yaw轴、底盘跟随的目标值
    setMotorPos(PihPos,PihTarget);
    setMotorPos(YawPos,YawTarget);
    setMotorPos(ChassisYaw,ChassisYawTarget);
    //开镜电机的位置环目标值
    setMotorPos(ScopeUPos,ScopeUTarget);

    portSetTurn();//云台反转。如果按下V，云台立马反转180°，如果没有按下，不影响程序运行

    //MATLAB的PID数据更新
    Pid_In.PihAngle_set = PihTarget;

    Pid_In.PihAngle_Now=motors[PihMotor].RealAngle_Imu;
    Pid_In.PihSpeed_Now=motors[PihMotor].RealSpeed;

    Pid_In.YawAngle_set = YawTarget;

    Pid_In.YawAngle_Now=motors[YawMotor].RealAngle_Imu;
    Pid_In.YawSpeed_Now=motors[YawMotor].RealSpeed;

    //计算Pih轴和Yaw轴的位置环输出
    float Pihout=motors_pid[PihPos].PID_GetPositionPID(motors[PihMotor].RealAngle_Imu);
    float YawOut=motors_pid[YawPos].PID_GetPositionPID(motors[YawMotor].RealAngle_Imu);
    float ScopeUOut=motors_pid[ScopeUPos].PID_GetPositionPID(motors[ScopeUMotor].RealAngle_Ecd);

    motors_pid[YawPos].PID_LastTarget=motors_pid[YawPos].PID_Target;
    YawOut=YawOut+Pid_In.Yaw_Dif_Gain* (motors_pid[YawPos].PID_Target - motors_pid[YawPos].PID_LastTarget);

    //计算出底盘Yaw的PID_Out,赋值给vz
    motors_pid[ChassisYaw].PID_OutMax=100;
    motors_pid[ChassisYaw].PID_GetPositionPID(motors[YawMotor].RealAngle_Ecd);

    //串级PID，位置环的输出是速度环的目标值
    setMotorSpeed(PihSpd,Pihout);
    setMotorSpeed(YawSpd,-YawOut);
    setMotorSpeed(ScopeUSpd,ScopeUOut);

    //计算Pih轴和Yaw轴的速度环输出
    motors_pid[YawSpd].PID_GetPositionPID(motors[YawMotor].RealSpeed);
    motors_pid[PihSpd].PID_GetPositionPID(motors[PihMotor].RealSpeed);

    //计算开镜电机速度环输出
    motors_pid[ScopeUSpd].PID_GetPositionPID(motors[PihMotor].RealSpeed);

    //拨弹轮、摩擦轮的位置环和速度环
    shoot.Shoot_PosC();
}

///电机速度环
void cGimbal::Gimbal_SpeedC()
{
    //根据算法发送电流
    //Yaw轴算法选择
    if(count_time_send==2)
    {
        switch(motors[YawMotor].Algorithm)
        {
            case NORMAL:
            {
                CAN_YawSendCurrent((int16_t) motors_pid[YawSpd].PID_Out);
                break;
            }
            case MATLAB:
            {
                CAN_YawSendCurrent((int16_t)Pid_Out.YawCurrent);
                break;
            }
            case DAMIAO:
            {
                if(ControlMode!=PROTECT)
                {
                    ctrl_torset(-motors_pid[YawSpd].PID_Out);
                    ctrl_send(); //达妙电机的发送can信号
                }
            }
        }
    }
    //Pitch轴算法选择
    switch(motors[PihMotor].Algorithm)
    {
        case NORMAL:
        {
            CAN_PitchSendCurrent((int16_t)motors_pid[PihSpd].PID_Out);
            break;
        }
        case MATLAB:
        {
            CAN_PitchSendCurrent((int16_t)Pid_Out.PihCurrent);
            break;
        }
        case CYBERGEAR:
        {
            if(ZimiaoFlag==OPENZIMIAO||CarMode==ZIMIAO)
                motor_controlmode(&mi_motor[0],0,PihTarget,0,20,2);
            else if(CarMode==PROTECT || ProtectFlag==OFFLINE)
                motor_controlmode(&mi_motor[0],0,PihTarget,0,0,0);
            else
                motor_controlmode(&mi_motor[0],0,PihTarget,0,150,3);

        }
    }

    //摩擦轮与拨弹轮发电流
    if(count_time_send==3)
    {
        switch (motors[ShootLMotor].Algorithm)
        {
            case NORMAL:
            {
                shoot.Shoot_SendCurrent(motors_pid[ShootSpdL].PID_Out, motors_pid[ShootSpdR].PID_Out,
                                        motors_pid[ShootSpdU].PID_Out, motors_pid[RamSpd].PID_Out);
                break;
            }
            case ADRC:
            {
                shoot.Shoot_SendCurrent(shoot.ShootLOUT_ADRC, shoot.ShootROUT_ADRC, shoot.ShootUOUT_ADRC,
                                        motors_pid[RamSpd].PID_Out);
                break;
            }
        }
    }
    //开镜电机发电流
    if(count_time_send==4)
    {
        CAN_ScopeSendCurrent((int16_t)motors_pid[ScopeUSpd].PID_Out);
    }
}

/**
  *@brief   设置电机速度环目标值
  *@param   需要设置目标值的电机;需要电机对外实际转的速度spd
  */
void cGimbal::setMotorSpeed(int WhichMotorPid, float spd)
{
    motors_pid[WhichMotorPid].PID_SetTarget(spd);
}

/**
  *@brief   设置电机位置环目标值
  *@param   需要设置目标值的电机;需要电机对外实际转的角度angle * M3508_RATION
  */
void cGimbal::setMotorPos(int WhichMotorPid,float angle)
{
    motors_pid[WhichMotorPid].PID_SetTarget(angle);
}

///Pitch轴的编码器限幅
void cGimbal::Pitch_EcdLimit(float & Target)
{
    if (Target > _Pitch_EcdUpLimit) //ecd pitch限位
    {
        Target = _Pitch_EcdUpLimit;
    }
    if (Target < _Pitch_EcdLowLimit)
    {
        Target = _Pitch_EcdLowLimit;
    }
}

///Pitch轴的陀螺仪限幅
void cGimbal::Pitch_ImuLimit(float& Target)
{
    if (Target >  _Pitch_ImuUpLimit) //ecd pitch限位
    {
        Target =  _Pitch_ImuUpLimit;
    }
    if (Target < _Pitch_ImuLowLimit)
    {
        Target = _Pitch_ImuLowLimit;
    }
}
void cGimbal::Pitch_MILimit(float& Target)
{
    if (Target >  _Pitch_MIUpLimit) //ecd pitch限位
    {
        Target =  _Pitch_MIUpLimit;
    }
    if (Target < _Pitch_MILowLimit)
    {
        Target = _Pitch_MILowLimit;
    }
}
///云台的卡尔曼滤波算法初始函数
void cGimbal::Gimbal_KalmanInit(void)
{   //初始化该滤波器的Q=1 R=40参数
    KalmanCreate(&Gimbal_YawAngle, 40, 200);
    KalmanCreate(&Gimbal_PihAngle, 40, 200);
    KalmanCreate(&Gimbal_MouseX, 30, 200);
    KalmanCreate(&Gimbal_MouseY, 30, 200);
    KalmanCreate(&ZIMIAO_Yaw, 30, 200);
    KalmanCreate(&ZIMIAO_Pih, 30, 200); //暂时没用
}

///选择各种模式下的PID参数、ADRC参数
void cGimbal::Gimbal_ParamChoose(int8_t mode)
{
    adrc[0].ADRCParamInit(1500, 0.05, 1, 0.5, 1, 1000, 10,0.5, 1.25, 80, 1);
    adrc[1].ADRCParamInit(1500, 0.05, 1, 0.5, 1, 1000, 10,0.5, 1.25, 80, 1);
    adrc[2].ADRCParamInit(1500, 0.05, 5, 0.5, 1, 1000, 10,0.5, 1.25, 80, 1);

    switch (mode)
    {
        case IMU_MODE://陀螺仪反馈模式
        {
            ///Yaw轴的MATLAB_PID参数///
//            Pid_In.YawP_P = 4;
//            Pid_In.YawP_I = 0.00;
//            Pid_In.YawP_D = 0;
//            Pid_In.YawP_N = 175;
//            Pid_In.YawP_MO = 300;
//            Pid_In.Yaw_Dif_Gain = 0;
//
//            Pid_In.YawS_P = 0.025;
//            Pid_In.YawS_I = 0.005;
//            Pid_In.YawS_D = 0;
//            Pid_In.YawS_N = 0;
//            Pid_In.YawS_MO = 10;
            ///Yaw轴的普通PID参数///
            motors_pid[YawPos].Kp = 5;
            motors_pid[YawPos].Ki = 0;
            motors_pid[YawPos].Kd = 0.2;
            motors_pid[YawPos].SetMax(50,300,50);

            motors_pid[YawSpd].Kp = 0.02;
            motors_pid[YawSpd].Ki = 0.004;
            motors_pid[YawSpd].Kd = 0;
            motors_pid[YawSpd].SetMax(50,10,5);
            Pid_In.Yaw_Dif_Gain = 0.2;

            break;
        }
        case ECD_MODE://编码器反馈模式
        {
            ///Yaw轴的MATLAB_PID参数///
            Pid_In.YawP_P = 0.3;
            Pid_In.YawP_I = 0;
            Pid_In.YawP_D = 0;
            Pid_In.YawP_N = 180;
            Pid_In.YawP_MO = 300;
            Pid_In.Yaw_Dif_Gain = 0;

            Pid_In.YawS_P = 400;
            Pid_In.YawS_I = 400;
            Pid_In.YawS_D = 0;
            Pid_In.YawS_N = 0;
            Pid_In.YawS_MO = 28000;
            break;
        }
        case ZIMIAO://自瞄模式
        {
            ///Yaw轴的MATLAB_PID参数///
            motors_pid[YawPos].Kp = 8;
            motors_pid[YawPos].Ki = 0;
            motors_pid[YawPos].Kd = 0.4;
            motors_pid[YawPos].SetMax(50,300,50);

            motors_pid[YawSpd].Kp = 0.02;
            motors_pid[YawSpd].Ki = 0.002;
            motors_pid[YawSpd].Kd = 0;
            motors_pid[YawSpd].SetMax(50,10,5);
            Pid_In.Yaw_Dif_Gain = 0;
            break;
        }
    }//switch结束
    Pid_In.YawP_LO = -Pid_In.YawP_MO;
    Pid_In.YawS_LO = -Pid_In.YawS_MO;
}

///遥控器在线状态检测
void cGimbal::Online_Check()
{
    RC_CheckTimes--;
    if(RC_CheckTimes==0)
    {   //每1s(20*5ms)一次检测，查看键值以及底盘状态值是否更新，若没有，则断电
        RC_CheckTimes=20;
        if(RC_UpdateData() == RC_GetLastData)
        {
            ProtectFlag=OFFLINE;
        }
        else
        {
            ProtectFlag=ONLINE;
        }
        RC_GetLastData=RC_UpdateData();//更新遥控器接受数据的检测判断
    }
}

///打印函数
void cGimbal::Printf_Test()
{
    //Yaw打印//
//    usart_printf("%f,%f,%f\r\n",motors_pid[YawSpd].PID_Out,motors_pid[YawPos].PID_Target,motors[YawMotor].RealAngle_Imu);
//    usart_printf("%f,%f,%f\r\n",Pid_Out.YawCurrent,motors_pid[YawPos].PID_Target,motors[YawMotor].RealAngle_Imu);
//    usart_printf("%f,%f,%f\r\n", motors_pid[YawSpd].PID_Out,motors_pid[YawSpd].PID_Target,motors[YawMotor].RealSpeed);
//    usart_printf("%d,%d\r\n",Debug_Param().pos_maxIntegral,motors[YawMotor].RawSpeed);
    //Pih打印//
//    usart_printf("%f,%f,%f\r\n",Pid_Out.PihCurrent,PihTarget,motors[PihMotor].RealAngle_Imu);
//    usart_printf("%f,%f,%f\r\n",motors_pid[PihSpd].PID_Out,PihTarget,motors[PihMotor].RealAngle_Imu);
//    usart_printf("%f,%f,%f\r\n",PihTarget,mi_motor[0].Angle,mi_motor[0].Speed);
//    usart_printf("%f,%f\r\n",mi_motor[0].Angle,IMU_Angle_CH100(PIH_ANGLE));
    //底盘打印//
//    usart_printf("%f,%f\r\n",vx,vy);
//    usart_printf("%f,%f,%f,%f\r\n",motors[YawMotor].RealAngel_Ecd,motors_pid[ChassisYaw].PID_Out,ChassisYawTarget,vz);
    //摩擦轮打印//
//    usart_printf("%f,%f,%f,%f\r\n",
//                 gimbal.motors_pid[ShootSpdL].PID_Target,gimbal.motors[ShootLMotor].RealSpeed,
//                 gimbal.motors_pid[ShootSpdR].PID_Target,gimbal.motors[ShootRMotor].RealSpeed);
    //ADRC打印//
//    usart_printf("%f,%f,%f,%f,%f,%f\r\n",gimbal.motors_pid[ShootSpdL].PID_Target,shoot.ShootLOUT_ADRC,motors[ShootLMotor].RealSpeed,
//                 gimbal.motors_pid[ShootSpdR].PID_Target,shoot.ShootROUT_ADRC,motors[ShootRMotor].RealSpeed);
    //拨弹轮打印//
//    usart_printf("%f,%f,%f,%d,%d\r\n",motors_pid[RamSpd].PID_Out,motors_pid[RamPos].PID_Target,
//                 motors[RamMotor].RealAngle_Ecd,gimbal.shoot.heat_now,gimbal.shoot.heat_now_user);
    //自瞄打印
    usart_printf("%f,%f,%f,%f,%f,%f,%f\r\n",vision_pkt.offset_yaw, lowfilter_yaw.Filter(vision_pkt.offset_yaw),YawTarget, motors[YawMotor].RealAngle_Imu
    ,-lowfilter_pih.Filter(vision_pkt.offset_pitch), PihTarget,  mi_motor[0].Angle);
//    usart_printf("%f,%f\r\n",vision_pkt.offset_yaw,KalmanFilter(&ZIMIAO_Yaw,vision_pkt.offset_yaw));
    //滤波打印//
//    lowfilter.Init(50,0.005);
//    usart_printf("%f,%f,%f,%f\r\n",vision_pkt.offset_yaw,lowfilter.Filter(vision_pkt.offset_yaw)
//                 ,vision_pkt.offset_pitch,lowfilter.Filter(vision_pkt.offset_pitch));
    //按键打印//
//    usart_printf("%d,%d\r\n",RC_GetDatas().key.A.Now_State,RC_GetDatas().key.Q.Is_Click_Once);
    //IMU打印//
//    usart_printf("%f,%f,%f\r\n", IMU_Angle(PIH_ANGLE),IMU_Angle(YAW_ANGLE),IMU_Angle(ROLL_ANGLE));
//    usart_printf("%f,%f\r\n",motors[YawMotor].RealAngle_Imu,motors[PihMotor].RealAngle_Imu);
    //遥控器打印//
//    usart_printf("%f,%f\r\n",YawTarget,PihTarget);
//    usart_printf("%d,%d\r\n",RC_GetDatas().rc.s[0],RC_GetDatas().rc.s[1]);
//    usart_printf("%f,%f,%f,%f\r\n",portSetYawSpeed(),MouseYaw,portSetPihSpeed(),MousePih);

//    usart_printf("%f,%f,%f,%f,%f,%f\r\n", IMU_Angle_CH100(1), IMU_Angle_CH100(2),IMU_Angle_CH100(3),
//                 IMU_Speed_CH100(1),IMU_Speed_CH100(2),IMU_Speed_CH100(3));
    //热量打印//
//    usart_printf("%d,%d,%d,%d\r\n",gimbal.shoot.heat_limit,gimbal.shoot.heat_now,gimbal.shoot.heat_now_user,gimbal.shoot.cool_spd);
//    usart_printf("%d\r\n",gimbal.GimbalPower);
}