/* Includes ------------------------------------------------------------------*/
#include "Motor.h"
#include "Sensors.h"
#include "extra_built_in_functions.h"
#include "stm32f10x_it.h"


/* Global typedef -----------------------------------------------------------*/
TIM_TimeBaseInitTypeDef time2, time7;
NVIC_InitTypeDef  NVIC_InitStructure;
/* Global define ------------------------------------------------------------*/
/* Global macro -------------------------------------------------------------*/
/* Global variables ---------------------------------------------------------*/
struct Sensors m0,m0_old;
int basespeed = 3000 , escapeSpeed, obstacleThreshold = 2000, SensorThreshold = 2400;
char state = 'd', parkingState=0;
int obstacleFlag=0, correction_val, k = 1000;

/* Global Variables for PID */
float old_error_distance = 0, old_error_direction = 0;
float i_Temp_direction   = 0, i_Temp_distance     = 0;

/* Global function prototypes -----------------------------------------------*/
/* Global functions ---------------------------------------------------------*/
int getCorrectionValue();
void lowPassFilter(int *newVal, int *oldVal,int length);
void turnByAngle(char direction, int angle);
char detectParkingLine();
void Delay_Init();
void MyDelay(uint16_t milisec);
void Timer7_Init();
float PID(int signal, int des_signal, float Kp, float Kd, float Ki, float * old_error, float * i_Temp);


int main(void)
{
  //אתחולים
  Blue_button_Init();
  Motor_Init();
  Sensor_Reading_Init();
  Blue_Led_Init(); //Initialize timer4-OC1  to synchronize blue led
  TIM_Cmd(TIM4, ENABLE); //Turn blue led ON
  Delay_Init();
  Timer7_Init();

  /******* Stuck here until button is pushed *******************/
  while (!GPIO_ReadInputDataBit (GPIOA, GPIO_Pin_0));
  while (GPIO_ReadInputDataBit (GPIOA, GPIO_Pin_0));
  /*************************************************************/  
  TIM_Cmd(TIM4, DISABLE); //Turn blue led OFF
  GPIO_WriteBit (GPIOC,GPIO_Pin_8, Bit_RESET); //Turn blue led OFF

  //קריאת חיישנים ראשונית
  m0=Read_Sensors();
  //הפעלה של טיימר 7 לזיהוי מכשולים
  TIM_Cmd(TIM7, ENABLE);

  while (1)
  {
    //שמירת ערך המדידה הקודם
    m0_old=m0;
    //קריאת החיישנים וביצוע סינון לוו פס
    m0=Read_Sensors();
    lowPassFilter((int*)m0.array_sensor,(int*)m0_old.array_sensor, 8);
    lowPassFilter((int*)&m0.distance,(int*)&m0_old.distance, 1);
    switch(state) 
    {
      case 'd':  // מצב נסיעה רגיל
        if(obstacleFlag) //אם זיהית מכשול תשמור על מרחק ביטחון ממנו
        {
          escapeSpeed = PID(m0.distance, obstacleThreshold, 1, 1, 0.01, &old_error_distance, &i_Temp_distance);
          Motor_Drive(MOTOR1,escapeSpeed);  
          Motor_Drive(MOTOR2,escapeSpeed); 
        }
        else //אחרת עקוב אחרי הקו כרגיל
        {
          //בדיקת קו חניה
          parkingState=detectParkingLine();
          if (parkingState!=0) 
            state = 'p';  
          else 
          {
            //חישוב ערך תיקון
            correction_val = getCorrectionValue();
            correction_val=PID(correction_val,  0,  1 , 2, 0.01, &old_error_direction, &i_Temp_direction) *-1;
            //מתן פקודות למנועים
            //motor1=מנוע שמאל, motor2 = מנוע ימין
            Motor_Drive(MOTOR1,basespeed+correction_val*k);  
            Motor_Drive(MOTOR2,basespeed-correction_val*k);
          }
        }
        break;

      case 'p':  // מצב  חניה
        //נסיעה קצרה קדימה
        Motor_Drive(MOTOR1,basespeed);  
        Motor_Drive(MOTOR2,basespeed); 
        MyDelay(1000);

        // סיבוב כולל של 90 מעלות שבמהלכו הרובוט בודק אם קיימים מכשולים
        turnByAngle(parkingState,50);
        MyDelay(500);
        for(int i=0;i<2;i++)
        {
          Motor_Stop(MOTOR1);
          Motor_Stop(MOTOR2);
          MyDelay(100);
          m0=Read_Sensors();
          MyDelay(500);
          //אם מצאת מכשול תעבור למצב יציאה (exit)
          if(obstacleFlag)
            state='e';
          turnByAngle(parkingState,20);
          MyDelay(500);
        }
        Motor_Stop(MOTOR1);
        Motor_Stop(MOTOR2);

       //אם נקלט מכשול יש לצאת מהחניה ולהמשיך בנסיעה
        if(state=='e')
        {
          //סיבוב 90 מעלות
          if(parkingState=='r')
            turnByAngle('l',90);
          else if(parkingState=='l')
            turnByAngle('r',90);
          //נסיעה קלה לאחור
          Motor_Drive(MOTOR1,-basespeed);  
          Motor_Drive(MOTOR2,-basespeed); 
          MyDelay(800);
          //חזרה למצב נסיעה
          state='d';
          break;
        }

        //אם אין מכשול תתחיל רצף חניה
        //סיבוב 180 מעלות
        if(parkingState=='r')
          turnByAngle('l',180);
        else if(parkingState=='l')
          turnByAngle('r',180);
        MyDelay(500);
        //סע לאחור
        Motor_Drive(MOTOR1,-basespeed);  
        Motor_Drive(MOTOR2,-basespeed); 
        MyDelay(2000);
        //חכה 5 שניות
        Motor_Stop(MOTOR1);
        Motor_Stop(MOTOR2);
        MyDelay(5000);
        //צא מהחניה
        Motor_Drive(MOTOR1,basespeed);  
        Motor_Drive(MOTOR2,basespeed); 
        MyDelay(2000);
        turnByAngle(parkingState,90);
        MyDelay(500);
        //נסיעה קלה לאחור
        Motor_Drive(MOTOR1,-basespeed);  
        Motor_Drive(MOTOR2,-basespeed); 
        MyDelay(800);
        Motor_Stop(MOTOR1);
        Motor_Stop(MOTOR2);
        MyDelay(100);
        //חזרה למצב נסיעה
        state='d';
        break;
    }   
  }
}

//מסנן מעביר נמוך למערך באורך length
void lowPassFilter(int *newVal, int *oldVal,int length)
{
  float a=0.8;
  for (int i = 0; i < length; i++)
  {
    newVal[i]=a*newVal[i]+(1-a)*oldVal[i];
  }
  return;
}

//חישוב ערך תיקון החיישנים
int getCorrectionValue()
{
  int val=0;
  int sensor_map[8]={-2,-2,-1.5,-1,1,1.5,2,2};
  for (int i = 0; i < 8; i++)
  {
      if(m0.array_sensor[i]>SensorThreshold)
        val+=sensor_map[i];
  }
  return val;
}

//זיהוי קו החניה
char detectParkingLine() {
  // בודק אם יש קו חניה באמצעות החיישנים הקיצוניים
  if ((m0.array_sensor[0] > SensorThreshold || m0.array_sensor[1] > SensorThreshold) 
    && m0.array_sensor[2] > SensorThreshold && m0.array_sensor[3] > SensorThreshold) 
      return 'l';
  if ((m0.array_sensor[6] > SensorThreshold || m0.array_sensor[7] > SensorThreshold) 
    && m0.array_sensor[4] > SensorThreshold && m0.array_sensor[5] > SensorThreshold) 
      return 'r';
  return 0;
}

//מסתובב בזווית לכיוון מסויים
//direction= 'r'/'l', angle>0
void turnByAngle(char direction, int angle)
{
  //בדיקת קלט תקין
  if(angle<0 || (direction!='r' && direction!='l'))
    return;

  //הגדרות כיוון
  int dir=0;
  if(direction=='r')
    dir=1;
  else if(direction=='l')
    dir=-1; 

  //מתן הוראת פניה למנועים בכיוון הנכון, משך הדיליי קובע את הזווית בה הרובוט מסתובב
  Motor_Drive(MOTOR1, dir* basespeed);
  Motor_Drive(MOTOR2, dir* -basespeed);
  MyDelay(5750*angle/360.0); //when voltage is exactly 7.0
  Motor_Stop(MOTOR1);
  Motor_Stop(MOTOR2);
}

//אתחול טיימר 2 לטובת פונקציית הדיליי
void Delay_Init()
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
  time2.TIM_ClockDivision = 0;
  time2.TIM_CounterMode = TIM_CounterMode_Up;
  time2.TIM_Period=0xFFFF;
  time2.TIM_Prescaler = 24000-1;
  TIM_TimeBaseInit(TIM2,&time2);
  TIM_Cmd(TIM2,ENABLE);
}

//waits milisec[ms] before continues
void MyDelay(uint16_t milisec)
{
  TIM_SetCounter(TIM2, 0);
  while(TIM_GetCounter(TIM2)<milisec);
}

// בקרת פי איי דיי לסיגנל כלשהו.
float PID(int signal, int des_signal, float Kp, float Kd, float Ki, float *old_error ,float* i_Temp)
{
  //Local variables for PID
  float P,I,D,iMax = 7, iMin = -7 , error;
  
  //חישוב שגיאה
  error = (des_signal - signal); 
  //P
  P = Kp * error; 
  //I
  *i_Temp += error;
  if (*i_Temp > iMax)
    *i_Temp = iMax;
  else if (*i_Temp < iMin)
    *i_Temp = iMin;
  I = Ki * *i_Temp;
  //D
  D = Kd * (*old_error - error);  
  *old_error = error;
  //סכימה של שלושת הרכיבים
  return P + I + D;
}

void TIM7_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM7,TIM_FLAG_Update)==SET)
  {
    //אם זוהה מכשול
    if(m0.distance > obstacleThreshold)
    {
      obstacleFlag=1;
      GPIO_WriteBit(GPIOC,GPIO_Pin_8,(BitAction) 1) ;
    }
    else //אחרת
    {
      obstacleFlag=0;
      GPIO_WriteBit(GPIOC,GPIO_Pin_8,(BitAction) 0) ;
    }
    TIM_ClearITPendingBit( TIM7, TIM_FLAG_Update);
  }
}

//אתחול טיימר 7
void Timer7_Init()
{
  RCC_APB1PeriphClockCmd  (RCC_APB1Periph_TIM7,ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 

  time7.TIM_Period = 2000;
  time7.TIM_Prescaler = 5000;
  time7.TIM_ClockDivision = TIM_CKD_DIV1;
  time7.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM7, &time7);

  TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
}


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif