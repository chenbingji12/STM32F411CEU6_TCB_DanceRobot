/**
  * @file    Single_action.c
  * @brief   单一动作控制函数实现（命令表）
  */

#include "Single_action.h"

extern GaitMode gaitmode;
extern SpeedParams speedparams;
extern uint32_t dance_start_time;

#define FLOWER_MIN_SWITCH_INTERVAL_MS 200U

static uint8_t flower_bpm_active=0U;
static uint8_t flower_target_open=0U;
static uint8_t flower_have_last_beat=0U;
static uint16_t flower_next_beat_index=0U;
static uint32_t flower_beat_start_time=0U;
static uint32_t flower_last_beat_time=0U;
static uint32_t flower_last_switch_tick=0U;

/**
 * @brief   向舵机发送动作数据
 */
static void Send_Data_To_Servo(const uint16_t data[][3],uint8_t count)
{
    for(uint8_t i=0;i<count;i++)
    {
        Servo_Write((uint8_t)data[i][0],(uint16_t)data[i][1],(uint16_t)data[i][2]);
    }
}

/**********************上位机获取与直接设置舵机指令*****************/
/**
  * @brief  设置舵机角度
  * @param  param: 参数字符串
  * @retval 无
  */
void Set_Servo_Pos(char *param)
{
    int servo_id,servo_angle,servo_time;
    if(sscanf(param,"%d %d %d",&servo_id,&servo_angle,&servo_time)==3)
    {
        Servo_Write((uint8_t)servo_id,(uint16_t)servo_angle,servo_time);
        printf("Set_Servo_Pos success:%d %d %d\n",servo_id,servo_angle,servo_time);
    }
    else{
        printf("Set_Servo_Pos error:%s\n",param);
    }
}

/**
  * @brief  读取舵机角度
  * @param  param: 参数字符串
  * @retval 舵机角度
  */
void Read_Servo_Pos(char *param)
{
    int servo_id;
    if(sscanf(param,"%d",&servo_id)==1)
    {
        uint16_t angle=Servo_ReadPos((uint8_t)servo_id);
        printf("Read_Servo_Pos success:%d\n",angle);
    }
    else{
        printf("Read_Servo_Pos error:%s\n",param);
    }
}

/**
  * @brief  读取所有舵机角度
  * @retval 无
  */
void ReadAllPos(char *param)
{
    int servo_id;
    for(servo_id=1;servo_id<=19;servo_id++)
    {
        uint16_t angle=Servo_ReadPos((uint8_t)servo_id);
        printf("Read_Servo_Pos success:%d %d\n",servo_id,angle);
        HAL_IWDG_Refresh(&hiwdg);   // 喂独立看门狗，防止复位,2048ms
    }
    printf("ReadAllPos success\n");
}

/**
 * @brief   重置所有舵机角度为默认
 */
void Reset_Whole(char *param)
{
    Send_Data_To_Servo(Reset_Whole_Data,Reset_Whole_Count);
    printf("Reset_Whole success\n");
}

/**
 * @brief   停止所有舵机动作
 */
void StopAll(char *param)
{
    for(uint8_t id=1;id<=19;id++)
    {
        Servo_Stop(id);
    }
    flag.walk_forward=0U;
    flag.walk_backward=0U;
    flag.move_to_left=0U;
    flag.move_to_right=0U;
    flag.turn_left=0U;
    flag.turn_right=0U;
    flag.moving=0U;
    flag.angle_correct=0U;
    speedparams.target_v=0.0f;
    speedparams.target_w=0.0f;
    speedparams.correct_w=0.0f;
    Location_deal_CancelTurn();
    Arm_Action_StopAll();
    flower_bpm_active=0U;
    printf("StopAll success\n");
}

extern IMU_Data_t *imu;
/**
 * @brief   软件归零（设置当前yaw为0度）
 * @retval  无
 */
void Zero_Yaw(char *param)
{
    Location_deal_ZeroYaw(imu);//软件归零
    flag.zero_yaw = 1;    //设置软件归零标志为已归零
    printf("Zero_Yaw success\n");
}

/**
 * @brief   取消软件归零（设置当前yaw角度为默认值）
 * @retval  无
 */
void Off_Zero_Yaw(char *param)
{
    flag.zero_yaw = 0;    //设置软件归零标志为未归零
    Location_deal_ClearYawError();//清空软件归零与yaw角度偏差
    printf("Off_Zero_Yaw success\n");
}

/**
  * @brief   开始IMU静止校准
  * @retval  无
  */
void IMU_Calibrate(char *param)
{
    IMU_SpeedDisplacement_StartCalibration();
    printf("IMU calibration start, keep robot still\n");
}

/**
  * @brief   清空IMU速度与位移
  * @retval  无
  */
void IMU_Reset(char *param)
{
    IMU_SpeedDisplacement_Reset();
    printf("IMU reset success\n");
}

/**
 * @brief  设置平移方向并启动航向角保持
 * @param  direction: 0=前进, 1=后退, 2=左移, 3=右移
 */
static void Start_Directional_Move(uint8_t direction)
{
    uint8_t direction_changed=0U;

    if((flag.moving==0U) ||
       ((direction==0U) && (flag.walk_forward==0U)) ||
       ((direction==1U) && (flag.walk_backward==0U)) ||
       ((direction==2U) && (flag.move_to_left==0U)) ||
       ((direction==3U) && (flag.move_to_right==0U)))
    {
        direction_changed=1U;
    }

    Location_deal_CancelTurn();

    flag.walk_forward=0U;
    flag.walk_backward=0U;
    flag.move_to_left=0U;
    flag.move_to_right=0U;

    if(direction==0U)
    {
        flag.walk_forward=1U;
    }
    else if(direction==1U)
    {
        flag.walk_backward=1U;
    }
    else if(direction==2U)
    {
        flag.move_to_left=1U;
    }
    else if(direction==3U)
    {
        flag.move_to_right=1U;
    }

    /* 平移时不继承之前的旋转目标，保持当前yaw。 */
    speedparams.target_w=0.0f;
    if(direction_changed!=0U)
    {
        speedparams.correct_w=0.0f;
        flag.angle_correct=1U;
    }
    flag.moving=1U;
}

/**
 * @brief   游戏手柄远程遥控数据处理
  * @param  param: 参数字符串
 * @retval  无
 */
void Gamepad_Control(char *param)
{
    int lx,ly,rx,ry,lt,rt,btns;
    char d;
    uint8_t current_moving;
    static uint8_t btns_flag1=0;//按钮状态标志位，用于判断是否需要读取舵机角度
    static uint8_t btns_flag2=0;//按钮状态标志位，用于判断是否需要改变摇杆比例系数
    static uint8_t change_gaitmode=0;//按钮状态标志位，用于判断是否需要改变gaitmode
    static float p=0.0002f;//摇杆比例系数
    static float T=1.5f;//周期，单位：秒
    static float servo_17=500.0f;
    static float servo_18=500.0f;
    static float servo_19=500.0f;
    static float servo_20=500.0f;
    static float pwm_servo=90.0f;//舵机PWM角度
    if(sscanf(param,"%d,LY:%d,RX:%d,RY:%d,LT:%d,RT:%d,BTNS:%d,D:%c",&lx,&ly,&rx,&ry,&lt,&rt,&btns,&d)==8)
    {
      if(btns%100/20==1)//右侧按键按下
      {
        if(btns%100/30==1)//左侧按键按下
        {
        T=1.5f;
        speedparams.target_w=0.0f;
        speedparams.target_v=0.0f;
        }
        if(lt>50)//左扳机按下
        {
          if(change_gaitmode==0)
          {
            change_gaitmode=1;
          switch(gaitmode)
          {
            case TROT:
              gaitmode=TURTLE;
              break;
            case TURTLE:
              gaitmode=BOUND;
              break;
            case BOUND:
              gaitmode=WALK;
              break;
            case WALK:
              gaitmode=TROT;
              break;
            default:
              break;
          }
        }
        }
        else if(lt==0)
        {
          change_gaitmode=0;
        }

        if(abs(lx)>260)
        {
          pwm_servo=pwm_servo+lx*p*0.4f;
          pwm_servo=pwm_servo>100?100:pwm_servo<60?60:pwm_servo;
          Servo_pwm_SetAngle(pwm_servo);
        }
      }
      else{//右侧按键松开
      if((btns_flag1==0)&&(btns/100==1))//左摇杆按键按下
      {
        btns_flag1=1;//按钮按下，不允许重复读取舵机角度
        servo_17=Servo_ReadPos(17);
        servo_18=Servo_ReadPos(18);
        servo_19=Servo_ReadPos(19);
        servo_20=Servo_ReadPos(20);
      }
      if(btns/200==1)//右摇杆按键按下
      {
        for(uint8_t id=17;id<=20;id++)
        {
          Servo_Write(id,500,1000);
        }
        servo_17=500.0f;
        servo_18=500.0f;
        servo_19=500.0f;
        servo_20=500.0f;
      }
      if(btns/100==0)
      {
        btns_flag1=0;//按钮松开，允许读取舵机角度
      }

      if((btns_flag2==0)&&(btns%100/40==1))//减号按键按下
      {
        btns_flag2=1;//按钮按下，不允许改变摇杆比例系数
        //p=p*0.8f;
        T=T-0.2f;
      }
      if((btns_flag2==0)&&(btns%100/80==1))//加号按键按下
      {
        btns_flag2=1;//按钮按下，不允许改变摇杆比例系数
        //p=p*1.25f;
        T=T+0.2f;
      }
      if(btns%100/10==0)
      {
        btns_flag2=0;//按钮松开，允许改变摇杆比例系数
      }

      if(btns%100/10==1)//左侧按键按下
      {
        flag.half_stand_to_init=1;
      }

      if(btns%10/1==1)//A按键按下
      {
        speedparams.target_v=speedparams.target_v-0.002f;
      }
      if(btns%10/2==1)//B按键按下
      {
        speedparams.target_w=speedparams.target_w+0.02f;
      }
      else if(btns%10/2==0)
      {
        flag.turn_right=0;
      }
      if(btns%10/4==1)//Y按键按下
      {
        speedparams.target_v=speedparams.target_v+0.002f;
      }
      if(btns%10/8==1)//X按键按下
      {
        speedparams.target_w=speedparams.target_w-0.02f;
      }
      else if(btns%10/8==0)
      {
        flag.turn_left=0;
      }

      if(lt>50)//左扳机按下
      {
        flag.init_to_half_stand=1;
      }
      else if(lt==0)
      {

      }
      if(rt>50)//右扳机按下
      {
        flag.half_stand_to_full_stand=1;
      }
      else if(rt==0)
      {

      }

      switch(d)//方向键按下
      {
        case 'U':
        Start_Directional_Move(0U);
          break;
        case 'D':
        Start_Directional_Move(1U);
          break;
        case 'L':
        Start_Directional_Move(2U);
          break;
        case 'R':
        Start_Directional_Move(3U);
          break;
        default:
        flag.walk_forward=0;
        flag.walk_backward=0;
        flag.move_to_left=0;
        flag.move_to_right=0;
          break;
      }

      if(abs(lx)>260)
      {
      servo_17=servo_17+lx*p;
      servo_17=servo_17>1000?1000:servo_17<0?0:servo_17;//限制舵机角度在0-1000之间
      Servo_Write(17, (uint16_t)servo_17, 0);
      }
      if(abs(ly)>260)
      {
      servo_18=servo_18+ly*p;
      servo_18=servo_18>1000?1000:servo_18<0?0:servo_18;//限制舵机角度在0-1000之间
      Servo_Write(18, (uint16_t)servo_18, 0);
      }
      if(abs(rx)>260)
      {
      servo_19=servo_19+rx*p;
      servo_19=servo_19>1000?1000:servo_19<0?0:servo_19;//限制舵机角度在0-1000之间
      Servo_Write(19, (uint16_t)servo_19, 0);
      }
      if(abs(ry)>260)
      {
      servo_20=servo_20+ry*p;
      servo_20=servo_20>1000?1000:servo_20<0?0:servo_20;//限制舵机角度在0-1000之间
      Servo_Write(20, (uint16_t)servo_20, 0);
      }
      }
      current_moving =
      (d=='U' || d=='D' || d=='L' || d=='R') ? 1U : 0U;

      if((flag.moving==0U) && (current_moving==1U))
      {
        /* 运动状态从停止变为运动时，只触发一次角度校正 */
        flag.angle_correct=1U;
      }
      else if((flag.moving==1U) && (current_moving==0U))
      {
        /* 手柄回到中立时，清除步态状态并保留目标速度 */
        flag.walk_forward=0U;
        flag.walk_backward=0U;
        flag.move_to_left=0U;
        flag.move_to_right=0U;
        flag.turn_left=0U;
        flag.turn_right=0U;
        flag.angle_correct=0U;
        speedparams.correct_w=0.0f;
      }

      /* 持续运动期间不重复触发角度校正 */
      flag.moving=current_moving;
    }

    //Get_Step_Length(speedparams.target_v,speedparams.target_w,T);
}

/**
 * @brief   设置舵机PWM角度与运行时间
 * @param   angle  角度值
 * @param   run_time  运行时间值
 * @retval  None
 * */
void Set_pwm_Servo_TargetAngle(char *param)
{
    float angle,run_time;
    if(sscanf(param,"%f %f",&angle,&run_time)==2)
    {
        Set_Servo_pwm_TargetAngle(angle, (uint16_t)run_time);
    }
}

extern BeatData beat_data;
/**
 * @brief   解析从上位机传输的节拍数据
 * @param  param: 参数字符串
 * @retval  无
 */
void Receive_Bmp(char *param)
{
    int bmp[10];
    if(sscanf(param,"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",&bmp[0],&bmp[1],
      &bmp[2],&bmp[3],&bmp[4],&bmp[5],&bmp[6],&bmp[7],&bmp[8],&bmp[9])==10)
      {
        for(uint8_t i=0;i<10;i++)
        {
          if((bmp[i]>=0) && (beat_data.beat_count<500U))
          {
            beat_data.beat_data[beat_data.beat_count]=(uint32_t)bmp[i];
            beat_data.beat_count++;
          }
        }
        (g_mode==DEBUG) && printf("[Receive_Bmp] %d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
          bmp[0],bmp[1],bmp[2],bmp[3],bmp[4],bmp[5],bmp[6],bmp[7],bmp[8],bmp[9]);
      }
    }

/**
 * @brief  启动上位机节拍时间轴
 */
static void Beat_Start_Cmd(char *param)
{
    Arm_Action_StopAll();
    flower_bpm_active=0U;
    Beat_Start();
    if(Beat_IsRunning()!=0U)
    {
        printf("Beat_Start success,count=%u\n",beat_data.beat_count);
    }
    else
    {
        printf("Beat_Start error: need at least 2 timestamps\n");
    }
}

/**
 * @brief   移动机器人
 * @param  param: 参数字符串
 * @retval  无
 */
void Move(char *param)
{
    float v,w,T;
    int derection;
    if(sscanf(param,"%f %f %f %d",&v,&w,&T,&derection)==4)
    {
        Location_deal_CancelTurn();

        speedparams.target_v=v;
        speedparams.target_w=w;

        /* 开始运动时，重新设置角度校正状态。 */
        flag.angle_correct=1;
        flag.walk_forward=1;
        flag.moving=1;
    }
    printf("Move success:v=%f,w=%f,T=%f\n",v,w,T);

    switch(derection)
    {
        case 0:
            flag.walk_forward=1;
            flag.walk_backward=0;
            flag.move_to_left=0;
            flag.move_to_right=0;
            break;
        case 1:
            flag.walk_backward=1;
            flag.walk_forward=0;
            flag.move_to_left=0;
            flag.move_to_right=0;
            break;
        case 2:
            flag.move_to_left=1;
            flag.move_to_right=0;
            flag.walk_forward=0;
            flag.walk_backward=0;
            break;
        case 3:
            flag.move_to_right=1;
            flag.move_to_left=0;
            flag.walk_forward=0;
            flag.walk_backward=0;
            break;
        default:
            break;
    }
}

/**
 * @brief   停止移动
  */
static void StopMove(char *param)
{
    speedparams.target_v=0.0f;
    speedparams.target_w=0.0f;
    flag.walk_forward=0U;
    flag.walk_backward=0U;
    flag.move_to_left=0U;
    flag.move_to_right=0U;
    flag.turn_left=0U;
    flag.turn_right=0U;
    flag.angle_correct=0U;
    speedparams.correct_w=0.0f;
    flag.moving=0;
    Location_deal_CancelTurn();
    printf("StopMove success\n");
}

/**
 * @brief   手势动作
  */
static void Gesture_Control(char *param)
{
    int gesture_id;
    if(sscanf(param,"%d",&gesture_id)==1)
    {
        switch(gesture_id)
        {
            case 0:
                
                break;
            case 1:
                
                break;
            case 2:
                
                break;
            case 3:
                
                break;
            case 4:
                
                break;
            case 5:

                break;
            default:
                break;
        }
    }
}

/**
* @brief   向左转(相对角度)
*/
static void Turn_Left(char *param)
{
    float angle;
    if(sscanf(param,"%f",&angle)==1)
    {
        Location_deal_StartRelTurn(angle);
    }
  }

/*** @brief   向右转(相对角度)
*/
static void Turn_Right(char *param)
{
    float angle;
    if(sscanf(param,"%f",&angle)==1)
    {
        Location_deal_StartRelTurn(-angle);
    }
  }

  extern volatile float battery_voltage;
/**
 * @brief  获取电池电压
 */ 
static void Get_Battery_Voltage(char *param)
{
  printf("Battery Voltage: %.2f V\n",battery_voltage+0.35f);
}

extern GaitMode gaitmode;
/**
 * @brief  切换步态
 */
static void Switch_GaitMode_To_TROT(char *param)
{
    gaitmode=TROT;
    printf("Switch_GaitMode_To_TROT success\n");
}
static void Switch_GaitMode_To_WALK(char *param)
{
    gaitmode=WALK;
    printf("Switch_GaitMode_To_WALK success\n");
}
static void Switch_GaitMode_To_TURTLE(char *param)
{
    gaitmode=TURTLE;
    printf("Switch_GaitMode_To_TURTLE success\n");
}

/**
 * @brief  从初始状态到半起立
 */
static void Init_To_Half_Stand_cmd(char *param)
{
    flag.init_to_half_stand=1;
    printf("Init_To_Half_Stand_cmd success\n");
}
static void Half_Stand_To_Init_cmd(char *param)
{
    flag.half_stand_to_init=1;
    printf("Half_Stand_To_Init_cmd success\n");
}
static void Inverted_Pendulum_Prepare_cmd(char *param)
{
    flag.inverted_pendulum_prepare=1;
    printf("Inverted_Pendulum_Prepare_cmd success\n");
}
static void Inverted_Pendulum_Finish_cmd(char *param)
{
    flag.inverted_pendulum_pid_adjust=0;
    printf("Inverted_Pendulum_Finish_cmd success\n");
}
static void Left_Depend_cmd(char *param)
{
    flag.left_depend=1;
    printf("Left_Depend_cmd success\n");
}
static void Right_Depend_cmd(char *param)
{
    flag.right_depend=1;
    printf("Right_Depend_cmd success\n");
}

/**
 * @brief  获取当前有效花瓣节拍到下一个有效节拍的间隔
 */
static uint16_t Flower_GetNextMoveTime(uint16_t beat_index)
{
    uint16_t i;
    uint32_t current_time=beat_data.beat_data[beat_index];
    uint32_t next_time;
    uint32_t interval=FLOWER_MIN_SWITCH_INTERVAL_MS;

    for(i=(uint16_t)(beat_index+1U);i<beat_data.beat_count;i++)
    {
        next_time=beat_data.beat_data[i];
        if((next_time>current_time) &&
           ((next_time-current_time)>=FLOWER_MIN_SWITCH_INTERVAL_MS))
        {
            interval=next_time-current_time;
            break;
        }
    }

    if(interval>0xFFFFU)
    {
        interval=0xFFFFU;
    }
    return (uint16_t)interval;
}

/**
 * @brief  等待花瓣按上位机时间戳交替开合
 */
static void Flower_Beat_Cmd(char *param)
{
    uint16_t i;
    uint32_t elapsed_time;

    (void)param;
    if((Beat_IsRunning()==0U) || (beat_data.beat_count<2U))
    {
        flower_bpm_active=0U;
        printf("flower_bpm error: send beat_start first\n");
        return;
    }

    for(i=1U;i<beat_data.beat_count;i++)
    {
        if(beat_data.beat_data[i]<=beat_data.beat_data[i-1U])
        {
            flower_bpm_active=0U;
            printf("flower_bpm error: timestamps must increase\n");
            return;
        }
    }

    flower_beat_start_time=dance_start_time;
    elapsed_time=HAL_GetTick()-flower_beat_start_time;
    flower_next_beat_index=0U;
    while((flower_next_beat_index<beat_data.beat_count) &&
          (beat_data.beat_data[flower_next_beat_index]<=elapsed_time))
    {
        flower_next_beat_index++;
    }

    if(flower_next_beat_index>=beat_data.beat_count)
    {
        flower_bpm_active=0U;
        printf("flower_bpm error: no upcoming timestamp\n");
        return;
    }

    flower_have_last_beat=0U;
    flower_last_beat_time=0U;
    flower_last_switch_tick=0U;
    flower_bpm_active=1U;
    printf("flower_bpm success\n");
}

/**
 * @brief  按有效节拍间隔非阻塞更新花瓣舵机
 */
void Flower_Beat_Process(void)
{
    uint32_t now_time;
    uint32_t elapsed_time;
    uint32_t beat_time;
    uint16_t move_time=FLOWER_MIN_SWITCH_INTERVAL_MS;
    uint16_t current_index;
    uint8_t move_requested=0U;

    if(flower_bpm_active==0U)
    {
        return;
    }

    now_time=HAL_GetTick();
    elapsed_time=now_time-flower_beat_start_time;
    while(flower_next_beat_index<beat_data.beat_count)
    {
        beat_time=beat_data.beat_data[flower_next_beat_index];
        if(elapsed_time<beat_time)
        {
            break;
        }

        current_index=flower_next_beat_index;
        if((flower_have_last_beat==0U) ||
           ((beat_time>flower_last_beat_time) &&
            ((beat_time-flower_last_beat_time)>=FLOWER_MIN_SWITCH_INTERVAL_MS) &&
            ((now_time-flower_last_switch_tick)>=FLOWER_MIN_SWITCH_INTERVAL_MS)))
        {
            flower_target_open=(uint8_t)(flower_target_open==0U);
            flower_last_beat_time=beat_time;
            flower_last_switch_tick=now_time;
            flower_have_last_beat=1U;
            move_time=Flower_GetNextMoveTime(current_index);
            move_requested=1U;
        }
        flower_next_beat_index++;
    }

    if(move_requested!=0U)
    {
        Set_Servo_pwm_TargetAngle(flower_target_open!=0U?100.0f:60.0f,
                                  move_time);
    }

    if((flower_next_beat_index>=beat_data.beat_count) ||
       (Beat_IsRunning()==0U))
    {
        flower_bpm_active=0U;
    }
}

/**
 * @brief  打开花并退出节拍开合
 */
static void Flower_Open(char *param)
{
    (void)param;
    flower_bpm_active=0U;
    flower_target_open=1U;
    Set_Servo_pwm_TargetAngle(100.0f,1000U);
}

/**
 * @brief  关闭花并退出节拍开合
 */
static void Flower_Close(char *param)
{
    (void)param;
    flower_bpm_active=0U;
    flower_target_open=0U;
    Set_Servo_pwm_TargetAngle(60.0f,1000U);
}

/**
 * @brief  向左移动
 */
static void MoveLeft(char *param)
{
  Start_Directional_Move(2U);
}
static void MoveRight(char *param)
{
  Start_Directional_Move(3U);
}
static void MoveBack(char *param)
{
  Start_Directional_Move(1U);
}

/**
 * @brief  获取当前IMU偏航角
 */
static void Get_IMU_Yaw(char *param)
{
    IMU_Data_t *imu_data = IMU_GetData();
    printf("Current IMU Yaw: %.2f degrees\n", imu_data->yaw);
}
/**
 * @brief   设置绝对目标yaw角度并启动转向
 */
static void Set_Target_Yaw(char *param)
{
    float yaw;
    if(sscanf(param,"%f",&yaw)==1)
    {
        Location_deal_StartAbsTurn(yaw);
        printf("target_yaw success:%.2f\n",yaw);
    }
}

/***********************动作数组定义*********************/
static const Action action[] = {
    {"reset_whole", 0, 0,Reset_Whole},
    {"read_all_pos",0,0,ReadAllPos},
    {"set ",1,0,Set_Servo_Pos},
    {"read ",1,0,Read_Servo_Pos},
    {"stopall",0,0,StopAll},
    {"zero_yaw",0,0,Zero_Yaw},
    {"off_zero_yaw",0,0,Off_Zero_Yaw},
    {"imu_calibrate",0,0,IMU_Calibrate},
    {"imu_reset",0,0,IMU_Reset},
    {"LX:",1,0,Gamepad_Control},
    {"arm_action_1_bmp",0,0,Arm_Action_1_Request},
    {"arm_action_2_bmp",0,0,Arm_Action_2_Request},
    {"arm_action_1",0,0,Arm_Action_1},
    {"arm_action_2",0,0,Arm_Action_2},
    {"beat_start",0,0,Beat_Start_Cmd},
    {"init_to_half_stand",0,0,Init_To_Half_Stand_cmd},
    {"half_stand_to_init",0,0,Half_Stand_To_Init_cmd},
    {"inverted_pendulum_prepare",0,0,Inverted_Pendulum_Prepare_cmd},
    {"inverted_pendulum_finish",0,0,Inverted_Pendulum_Finish_cmd},
    {"left_depend",0,0,Left_Depend_cmd},
    {"right_depend",0,0,Right_Depend_cmd},
    {"led_test",0,0,WS2812_TestCmd},
    {"led_fill ",1,0,WS2812_FillCmd},
    {"led_off",0,0,WS2812_OffCmd},
    {"led_anim ", 1, 0, WS2812_AnimationCmd},
    {"pwm_servo_set ",1,0,Set_pwm_Servo_TargetAngle},
    {"opticalflow_data_reset",0,0,OpticalFlow_Data_Reset},
    {"[",1,0,Receive_Bmp},
    {"move ",1,0,Move},
    {"stopmove",0,0,StopMove},
    {"gesture:",1,0,Gesture_Control},
    {"turn_left:",1,0,Turn_Left},
    {"turn_right:",1,0,Turn_Right},
    {"get_battery_voltage",0,0,Get_Battery_Voltage},
    {"switch_gaitmode_to_trot",0,0,Switch_GaitMode_To_TROT},
    {"switch_gaitmode_to_walk",0,0,Switch_GaitMode_To_WALK},
    {"switch_gaitmode_to_turtle",0,0,Switch_GaitMode_To_TURTLE},
    {"get_imu_yaw",0,0,Get_IMU_Yaw},
    {"target_yaw:",1,0,Set_Target_Yaw},
    {"flower_bpm",0,0,Flower_Beat_Cmd},
    {"flower_open",0,0,Flower_Open},
    {"flower_close",0,0,Flower_Close},
    {"moveleft",0,0,MoveLeft},
    {"moveright",0,0,MoveRight},
    {"moveback:",1,0,MoveBack},
}; // 动作表，存放所有动作的名称和对应的函数指针

/**************查找动作名称字符串在动作表中的位置***********/
/**
  * @brief  根据动作名称执行相应动作的命令表
  * @param  name: 动作名称字符串
  * @retval 无
  */
void Single_Action(char *name) // 根据传入的动作名称字符串，在动作表中查找对应的函数指针并执行相应动作
{
  for (int i = 0; i < ACTION_COUNT; i++) {
    if (action[i].is_circular == 0) // 如果是单次动作
    {
      if (action[i].is_prefix == 0) // 如果是完全匹配
      {
        if (strcmp(action[i].name, name) == 0) {
          action[i].handler(NULL);
          return;
        }
      } else if (action[i].is_prefix == 1) // 如果是前缀匹配
      {
        uint8_t len = strlen(action[i].name);
        if (strncmp((const char *)action[i].name, name, len) == 0) {
          action[i].handler(name + len); // 将指令前缀之后的“参数部分”传递给业务函数
          return;
        }
      }
    } 
  }
  printf("unknown single action:%s\n", name);
}
