/**
 * @brief   实时步态算法
 */

#include "Leg_action.h"

#define LEG_LEN_1 10.0f // 腿部长度1，单位：厘米
#define LEG_LEN_2 14.0f // 腿部长度2，单位：厘米
#define DISTANCE_BETWEEN_LEGS 38.0f // 腿部之间的距离，单位：厘米

GaitMode gaitmode = TROT; // 步态模式枚举变量，初始为对角步态

SpeedParams speedparams={0.0f}; // 当前角度校正参数，初始为0

static LegServo left_front={{7,6,5},{0,0,0}}; // 左前腿舵机id与偏移
static LegServo right_front={{3,2,1},{0,0,0}}; // 右前腿舵机id与偏移
static LegServo left_back={{11,10,9},{0,0,0}}; // 左后腿舵机id与偏移
static LegServo right_back={{15,14,13},{0,0,0}}; // 右后腿舵机id与偏移

static GaitParams left_front_params={0,1500,0.5f,{486,736,256},17.0f,10.0f,-9.8f,1}; // 左前腿步态参数
static GaitParams right_front_params={0,1500,0.5f,{482,745,273},17.0f,10.0f,-9.8f,1}; // 右前腿步态参数
static GaitParams left_back_params={0,1500,0.5f,{498,730,195},17.0f,10.0f,-9.8f,1}; // 左后腿步态参数
static GaitParams right_back_params={0,1500,0.5f,{483,729,213},17.0f,10.0f,-9.8f,1}; // 右后腿步态参数

extern volatile Flag flag; // 外部声明的标志结构体，用于控制步态算法的运行
/**
 * @brief   通过行走方向推出每条腿摆动方向
 * @param flag 
 */
static void Choose_Leg_Direction(Flag flag)
{
    if(flag.walk_forward==1)
    {
        left_front_params.derection=-1;
        right_front_params.derection=1;
        left_back_params.derection=-1;
        right_back_params.derection=1;
    }
    else if(flag.walk_backward==1)
    {
        left_front_params.derection=1;
        right_front_params.derection=-1;
        left_back_params.derection=1;
        right_back_params.derection=-1;
    }
    else if(flag.move_to_left==1)
    {
        left_front_params.derection=1;
        right_front_params.derection=1;
        left_back_params.derection=-1;
        right_back_params.derection=-1;
    }
    else if(flag.move_to_right==1)
    {
        left_front_params.derection=-1;
        right_front_params.derection=-1;
        left_back_params.derection=1;
        right_back_params.derection=1;
    }
    else if(flag.turn_left==1)
    {
        left_front_params.derection=1;
        right_front_params.derection=1;
        left_back_params.derection=1;
        right_back_params.derection=1;
    }
    else if(flag.turn_right==1)
    {
        left_front_params.derection=-1;
        right_front_params.derection=-1;
        left_back_params.derection=-1;
        right_back_params.derection=-1;
    }
}

static float target_w=0.0f;
/**
 * @brief   设定线速度v，角速度w,周期T，返回左右侧腿的步长
 * @param v 线速度，单位：米/秒
 * @param w 角速度，单位：弧度/秒
 */
void Get_Step_Length(float v,float w,float T)
{
    left_front_params.period=
    left_back_params.period=
    right_front_params.period=
    right_back_params.period=
    T*1000.0f;

    target_w=w;

    w=w+speedparams.correct_w;

    if(w==0.0f)//直线运动
    {
        left_front_params.step_length=
        right_front_params.step_length=
        left_back_params.step_length=
        right_back_params.step_length=(v/T)*100;
    }
    else
    {
        float r=v/w;//圆半径，单位：米

        float left_v=(r+DISTANCE_BETWEEN_LEGS/200.0f)*w;//左腿的线速度，单位：米/秒
        left_front_params.step_length=
        left_back_params.step_length=(left_v/T)*100;//左腿的步长，单位转化为厘米

        float right_v=(r-DISTANCE_BETWEEN_LEGS/200.0f)*w;
        right_front_params.step_length=
        right_back_params.step_length=(right_v/T)*100;//右腿的步长，单位转化为厘米
    }
    printf("w=%f\n",w);
}

/**
 * @brief   单条腿的轨迹计算
 * @param params 步态算法参数结构体指针
 * @return LegAngles* 腿部角度结构体指针
 */
static LegAngles Leg_Trajectory(GaitParams* params)
{
    float height_t;//随时间变化的当前高度，单位：厘米
    float len_t;//随时间变化的步长，单位：厘米
    LegAngles angles;
    //判断是支撑相还是摆动相
    if(params->phase<params->duty)
    {
        //支撑相
        float phase_support = params->phase/params->duty;//支撑相的相位，单位：弧度

        height_t=params->z_ground;
        len_t=(params->step_length / 2.0f-params->step_length * 
            (phase_support*phase_support*phase_support*
                (10 - 15 * phase_support + 6 * phase_support*phase_support)))*params->derection;
    }
    else
    {
        //摆动相
        float phase_swings = (params->phase-params->duty)/(1.0f-params->duty);//摆动相的相位，单位：弧度

        height_t=params->z_ground+params->step_height*sinf(phase_swings*PI);
        len_t=((-1)*(params->step_length / 2.0f- params->step_length * 
            (phase_swings*phase_swings*phase_swings*
                (10 - 15 * phase_swings + 6 * phase_swings*phase_swings))))*params->derection;
    }
    float sin_q1=height_t/LEG_LEN_1;
    angles.q[1]=asinf(sin_q1);
    angles.q[2]=-angles.q[1];

    float sin_q0=len_t/2/LEG_LEN_2;
    angles.q[0]=asinf(sin_q0);

    return angles;
}

/**
 * @brief   将弧度转换为角度并转化为舵机0-1000的值
 * @param   
 */
static int Radian_To_Angle(float radian)
{
    return (int)(radian/(PI*2.0f)*1000.0f);
}

/**
 * @brief   相位限制
 * @param phase 相位
 * @return float 相位限制后的值
 */
static float Wrap_Phase(float phase)
{
    while (phase >= 1.0f) {
        phase -= 1.0f;
    }

    while (phase < 0.0f) {
        phase += 1.0f;
    }

    return phase;
}

/**
 * @brief   相位偏移
 * @param gaitmode 步态模式
 * @return float[4] 相位偏移数组
 */
static GaitPhaseOffset* Phase_Offset(GaitMode gaitmode)
{
    static GaitPhaseOffset four_leg;
    switch(gaitmode)
    {
        case TROT:
        four_leg.phase_offset[0]=0.0f;
        four_leg.phase_offset[1]=0.5f;
        four_leg.phase_offset[2]=0.5f;
        four_leg.phase_offset[3]=0.0f;
            return &four_leg;
        case TURTLE:
        four_leg.phase_offset[0]=0.0f;
        four_leg.phase_offset[1]=0.0f;
        four_leg.phase_offset[2]=0.0f;
        four_leg.phase_offset[3]=0.0f;
            return &four_leg;
        case BOUND:
        four_leg.phase_offset[0]=0.0f;
        four_leg.phase_offset[1]=0.0f;
        four_leg.phase_offset[2]=0.0f;
        four_leg.phase_offset[3]=0.0f;
            return &four_leg;
        case WALK:
        four_leg.phase_offset[0]=0.0f;
        four_leg.phase_offset[1]=0.25f;
        four_leg.phase_offset[2]=0.5f;
        four_leg.phase_offset[3]=0.75f;
            return &four_leg;
        default:
            four_leg.phase_offset[0]=0.0f;
            four_leg.phase_offset[1]=0.0f;
            four_leg.phase_offset[2]=0.0f;
            four_leg.phase_offset[3]=0.0f;
        return &four_leg;
    }
}

/**
 * @brief   四条腿相位计算
 * @param period 步态周期，单位：毫秒
 */
static void Gait_Phase_Calc(uint32_t period)
{
    GaitPhaseOffset* four_leg=Phase_Offset(gaitmode);

    uint32_t current_time = HAL_GetTick();
    static uint32_t last_time = 0;
    if(last_time==0||current_time-last_time>150)
    {
        last_time=current_time;
        return;
    }

    static float base_phase = 0.0f;
    base_phase = Wrap_Phase(base_phase+(float)(current_time-last_time) / (float)period);
    left_front_params.phase=Wrap_Phase(
    base_phase+four_leg->phase_offset[0]);
    right_front_params.phase=Wrap_Phase(
    base_phase+four_leg->phase_offset[1]);
    left_back_params.phase=Wrap_Phase(
    base_phase+four_leg->phase_offset[2]);
    right_back_params.phase=Wrap_Phase(
    base_phase+four_leg->phase_offset[3]);
    last_time=current_time;

    LegAngles left_front_angles=Leg_Trajectory(&left_front_params);
    LegAngles right_front_angles=Leg_Trajectory(&right_front_params);
    LegAngles left_back_angles=Leg_Trajectory(&left_back_params);
    LegAngles right_back_angles=Leg_Trajectory(&right_back_params);

    for(uint8_t i=0;i<3;i++)
    {
        Servo_Write(left_front.id[i],
            (uint16_t)(left_front_params.original_angle[i]+
                Radian_To_Angle(left_front_angles.q[i])+left_front.offset[i]),
                25);
        Servo_Write(right_front.id[i],
            (uint16_t)(right_front_params.original_angle[i]+
                Radian_To_Angle(right_front_angles.q[i])+right_front.offset[i]),
                25);
        Servo_Write(left_back.id[i],
            (uint16_t)(left_back_params.original_angle[i]+
                Radian_To_Angle(left_back_angles.q[i])+left_back.offset[i]),
                25);
        Servo_Write(right_back.id[i],
            (uint16_t)(right_back_params.original_angle[i]+
                Radian_To_Angle(right_back_angles.q[i])+right_back.offset[i]),
                25);
    }
}

/**
 * @brief   角度限制
 * @param angle 角度
 * @return float 角度限制后的值
 */
static float Wrap_Angle(float angle)
{
    while(angle > 180.0f)
    {
        angle -= 360.0f;
    }

    while(angle < -180.0f)
    {
        angle += 360.0f;
    }

    return angle;
}

/**
 * @brief   计算一个步态周期的角度偏移值
 * @param gait_period 步态周期，单位：毫秒
 * @param target_angle 目标角度
 * @param current_angle 当前角度
 * @return float 角度偏移值
 */
static float Angle_Correct(float gait_period,float target_angle,float current_angle,uint8_t* update_flag)
{
    static float error_sum = 0.0f;
    static uint32_t count = 0;
    static uint32_t start_time = 0;
    static float angle_correct = 0.0f;

    if(update_flag != NULL)
    {
    *update_flag = 0U;
    }
    uint32_t current_time = HAL_GetTick();
    if(start_time==0)
    {
        start_time=current_time;
        return 0.0f;
    }
    
    float error=Wrap_Angle(target_angle-current_angle);
    error_sum += error;
    count++;
    if(current_time-start_time>=gait_period)
    {
        angle_correct = error_sum/count;
        error_sum = 0.0f;
        count = 0;
        start_time = current_time;
        *update_flag = 1;
    }
 
    return angle_correct;
}

/**
 * @brief   用PID调整角速度
 * @param gait_period 步态周期，单位：毫秒
 * @param target_angle 目标角度
 * @param current_angle 当前角度
 * @return float 角度偏移值
 */
static float Angle_Correct_PID(float gait_period,float target_angle,float current_angle)
{
    static float kp=0.01f;
    static float ki=0.0f;
    static float kd=0.0f;
    static float last_error = 0.0f;
    static float error_sum = 0.0f;
    static float error = 0.0f;
    static float correct_w=0.0f;
    static uint32_t last_time = 0;

    uint32_t current_time = HAL_GetTick();
    uint8_t angle_correct_update = 0;
    float dt=(float)(current_time-last_time)/1000.0f;

    error=Angle_Correct(gait_period,target_angle,current_angle,&angle_correct_update);
    if(angle_correct_update==1)
    {
        if(last_time == 0 || dt <= 0.0f)
    {
        last_time = current_time;
        last_error = error;
        return correct_w;
    }
        error_sum += error*dt;
        correct_w = kp*error+ki*error_sum+kd*(error-last_error)/dt;
        last_error = error;
        last_time = current_time;
    }
    correct_w = correct_w>0.3f?0.3f:correct_w<-0.3f?-0.3f:correct_w;
    return correct_w;
}

/**
 * @brief   计算角度偏差，在task.c中调用，每10ms执行一次
 */
void Angle_Correct_Process(void)
{
    static float target_yaw=0.0f;
    static uint32_t last_time = 0;

    IMU_Data_t *imu=IMU_GetData();

    if(flag.angle_correct == 1)
    {
    target_yaw = imu->yaw;
    speedparams.correct_w = 0.0f;
    flag.angle_correct = 0;
    last_time = HAL_GetTick();
    }
    else
    {
    target_yaw=(target_w*(float)(HAL_GetTick()-last_time)/1000.0f)/PI*180.0f+target_yaw;
    
    speedparams.correct_w=Angle_Correct_PID(left_front_params.period,target_yaw,imu->yaw);
    }
    last_time=HAL_GetTick();
    printf("correct_w=%f\n",speedparams.correct_w);
    printf("imu->yaw=%f\n",imu->yaw);
}

/**
 * @brief   实时步态任务,25ms执行一次
 * @retval 无
 */
void Realtime_Gait_Process(void)
{
    Choose_Leg_Direction(flag);
    Gait_Phase_Calc(left_front_params.period);
}
