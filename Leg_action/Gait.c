/**
 * @brief   实时步态算法
 */

#include "Leg_action.h"

#define LEG_LEN_1 10.0f // 腿部长度1，单位：厘米
#define LEG_LEN_2 14.0f // 腿部长度2，单位：厘米
#define DISTANCE_BETWEEN_LEGS 38.0f // 腿部之间的距离，单位：厘米

GaitMode gaitmode = TROT; // 步态模式枚举变量，初始为对角步态

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
    float base_phase = (float)(current_time % period) / (float)period;
    left_front_params.phase=Wrap_Phase(base_phase+four_leg->phase_offset[0]);
    right_front_params.phase=Wrap_Phase(base_phase+four_leg->phase_offset[1]);
    left_back_params.phase=Wrap_Phase(base_phase+four_leg->phase_offset[2]);
    right_back_params.phase=Wrap_Phase(base_phase+four_leg->phase_offset[3]);

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
 * @brief   实时步态任务,25ms执行一次
 * @retval 无
 */
void Realtime_Gait_Process(void)
{
    Choose_Leg_Direction(flag);
    Gait_Phase_Calc(left_front_params.period);
}
