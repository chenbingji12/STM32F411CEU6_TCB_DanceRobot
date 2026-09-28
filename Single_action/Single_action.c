/**
  * @file    Single_action.c
  * @brief   单一动作控制函数实现（命令表）
  */

#include "Single_action.h"

extern GaitMode gaitmode;
extern SpeedParams speedparams;

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
void Stop(char *param)
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
    printf("Stop success\n");
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
    static float p=0.00005f;//摇杆比例系数
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
          pwm_servo=pwm_servo>100?100:pwm_servo<65?65:pwm_servo;
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
        flag.walk_forward=1;
          break;
        case 'D':
        flag.walk_backward=1;
          break;
        case 'L':
        flag.move_to_left=1;
          break;
        case 'R':
        flag.move_to_right=1;
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

    Get_Step_Length(speedparams.target_v,speedparams.target_w,T);
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
          beat_data.beat_data[beat_data.beat_count]=bmp[i];
          beat_data.beat_count++;
          if(beat_data.beat_count>=500)
          {
            beat_data.beat_count=500;
          }
        }
        (g_mode==DEBUG) && SEGGER_RTT_printf(0,"[Receive_Bmp] %d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
          bmp[0],bmp[1],bmp[2],bmp[3],bmp[4],bmp[5],bmp[6],bmp[7],bmp[8],bmp[9]);
      }
    }

/**
 * @brief   移动机器人
 * @param  param: 参数字符串
 * @retval  无
 */
void Move(char *param)
{
    float v,w;
    if(sscanf(param,"%f %f",&v,&w)==2)
    {
        speedparams.target_v=v;
        speedparams.target_w=w;

        /* 开始运动时，重新设置角度校正状态。 */
        flag.angle_correct=1;
        flag.walk_forward=1;
        flag.moving=1;
    }
}

/***********************动作数组定义*********************/
static const Action action[] = {
    {"reset_whole", 0, 0,Reset_Whole},
    {"read_all_pos",0,0,ReadAllPos},
    {"set ",1,0,Set_Servo_Pos},
    {"read ",1,0,Read_Servo_Pos},
    {"stop",0,0,Stop},
    {"zero_yaw",0,0,Zero_Yaw},
    {"off_zero_yaw",0,0,Off_Zero_Yaw},
    {"LX:",1,0,Gamepad_Control},
    {"arm_action_1",0,0,Arm_Action_1},
    {"arm_action_2",0,0,Arm_Action_2},
    {"led_test",0,0,WS2812_TestCmd},
    {"led_fill ",1,0,WS2812_FillCmd},
    {"led_off",0,0,WS2812_OffCmd},
    {"pwm_servo_set ",1,0,Set_pwm_Servo_TargetAngle},
    {"opticalflow_data_reset",0,0,OpticalFlow_Data_Reset},
    {"[",1,0,Receive_Bmp},
    {"move ",1,0,Move},
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
    } else if (action[i].is_circular == 1) // 如果是循环动作
    {
    }
  }
  printf("unknown single action:%s\n", name);
}
