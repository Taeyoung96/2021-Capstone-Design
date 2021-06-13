/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2021 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "arm_math.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart5_rx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
// 일단 메트릭스 연산 제거하기!!!!
// ##################### Define #####################
//#define ARM_MATH_CM4
#define RX_BUF_SIZE             256
#define TX_BUF_SIZE             15

#define PULSE                       1404
#define PULSE_PER_RAD       0.0045

#define MAF_SIZE 10

// ##################### Basic Setup #####################
float32_t a2 = 2*PI/3, a3 = 4*PI/3; // angle gap between wheels
float32_t R = 0.400 , r = 0.0635;
float32_t theta = 0;

// ##################### Robot Parameter #####################
double La = 0.0023, Ra = 11.0, Ea = 0.1, J = 0.0043, B_m = 0.0019;
double Kt = 0.4307, Ke = 0.4307;

// ##################### Timer #####################
int cnt = 0, cnt2 = 0;

// ##################### Serial Comm #####################
uint8_t rx_dma_buffer[RX_BUF_SIZE];
uint8_t rx_buf[12];
//HAL_StatusTypeDef DMA_;
bool flag = false;

uint8_t tx_buf[TX_BUF_SIZE] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e};
uint8_t tx_buf_[TX_BUF_SIZE] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

volatile unsigned char head_pos; 
volatile unsigned char tail_pos = 255;
volatile unsigned char PacketMode;
volatile unsigned char checkSize;
uint8_t checkSum;

uint32_t Go_cnt = 0, Back_cnt = 0, Stop_cnt = 0; 

// ##################### IMU  #####################
uint8_t imu_rx_buf[16];
uint8_t imu_tx_buf[1] = {'*'};
uint16_t imu_Chk = 0, Chk = 0;

volatile double Roll_d, Pitch_d, Yaw_d;
int16_t Roll, Pitch, Yaw, vel_x, vel_y, vel_z;

// ##################### Motor Setup #####################
uint16_t Motor_CCR[4], CCR_pre[4];
int32_t En_cnt_cur[3],  En_cnt_pre[3], En_cnt_dif[3];
double w[3];
double dt = 0.005;

//##################### PI Speed Controller Setup  #####################
int16_t overflow = 0;

double target_[3] = {0.0, 0.0, 0.0}; 
double err_cur[3] = {0, 0, 0}, err_pre[3] = {0, 0, 0}, err_dif[3] = {0, 0, 0}, err_sum[3] = {0, 0, 0} ;

// Matlab Params
uint16_t Wcs = 2000;
//double Kps = 19.9675, Kis = 0.0441; //Kis = 4.4114  * dt(0.005)                               //Kps = (J*Wcs)/Kt, Kis = Kps*(B_m/J) 
double Kps[3] = {20.0, 20.0, 20.0}, Kis[3] = {0.0002, 0.0002, 0.0002}; //Kis = 0.00005 

int16_t j_target_[3], j_w_[3], direc[3] = {1, 1, 1}; // j = jscope

//##################### Limit Switch Setup #####################
bool up_exti_flag = true, down_exti_flag = false;
uint16_t Elevation_mode = 0, Elevation_mode_ = 0;                                             // 0 : Stop, 1 : Up , 2 : Down
int16_t Elevation_CCR;

uint16_t up_exti_cnt = 0, down_exti_cnt = 0;
uint16_t ele_direc = 0;

uint32_t elevation_cnt = 0, Rise_cnt = 0, Fall_cnt = 0 ;
bool Rise_bool = false, Fall_bool = false;

int ele_motor_dir_check=0;

//##################### PI Current Cointroll#####################
uint32_t Current_ADC[3] = {0,0,0};
uint16_t Wcc = 200;

double Voltage_to_Current[3] = {0.0, };
int Current_ADC_Jscope1;
int Current_ADC_Jscope2;
int Current_ADC_Jscope3;

//double Kpc = 0.4660, Kic = 1.1, Ka = 1.4306; //Kic = Ki * dt(0.0005)
// Kpc[3] = {0.22 , 0.22, 0.22}, Kic[3] = {0.02, 0.02, 0.02}, Ka = 1.4306; //Kic = 0.001 //2차 : Kpc = 0.3 Kic = 0.02
double Kpc[3] = {0.01 , 0.22, 0.3}, Kic[3] = {0.105, 0.02, 0.15}, Ka = 1.4306;
double I_ref[3] = {0.0, 0.0, 0.0};
double I_err_cur[3] = {0.0, }, I_err_pre[3] = {0.0, }, I_err_sum[3] = {0.0, }, I_err_a[3]= {0.0, };
double Motor_volt[3] = {0.0, }, Motor_volt_cur[3] = {0.0, };
double Volt_max = 12.0, Volt_min = -12.0, I_min = -2.5, I_max = 2.5;

//##################### MAF #####################

int rawData[3][MAF_SIZE] = {0, };
double SUM_Data[3] = {0.0, };
double MAF_Data[3] = {0.0, };
int MAF_cnt[3] = {0, };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM1_Init(void);
static void MX_UART5_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM8_Init(void);
static void MX_ADC1_Init(void);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void Uart_rx_dma_handler();
void IMU_Parsing();
double CalcMAF(int data, int n);

void Jscope_Callback();
void Get_Motor_Speed();
void SpeedCtrl();
void CurrentCtrl();
void Motor_ON();

void Elevation();
void Elevation_Cnt();

void up_elevation();
void down_elevation();
void stop_elevation();

void Elevation_Mode(uint16_t mode);
void Go_Straight_Test();

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* ############# Todo List #############
 *
 * 
 */

/* ############ Motor Control ############
 *
 * CCR1 ~ 3        : Three whelel  controlling Mootr_CCR of Omni direction robot
 * CCR4              : The Motor_CCR of elevation
 * En_cnt1 ~ 3     : The Encoder Sensor Count of Motor 1 ~ 3
 *
 * Encoder Diff(OverFlow)
 *              It must be Positive Value
 *              if Encoder Count Overflow, Minus 60000 and Multiply -1.
 * 
 */

/* ############ Motor Control ############
 * 
 *      Currnet Control(PI) : 0.5ms(2000Hz)
 *      Speed Control(PI)   : 5ms(200Hz)
 *
 */

/* ############ Timer OverFLow ############
 * 
 *      OverFlow                                 : 0.5ms(2000Hz)
 *      10-count Timer OverFLow         : 5ms(200Hz)
 *
 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if(htim == &htim7){                   //2000Hz(0.5ms)
    cnt++;
    
    Uart_rx_dma_handler();
    
    Get_Motor_Speed();
    
    //PI Currrent Controll 
    CurrentCtrl();
    
    if(cnt == 10){                         //200Hz(5ms)
      cnt = 0;
      
      Elevation_Mode(Elevation_mode);
      Elevation_Cnt();
      
      // PI Speed Controll(With Anti-Windup)
      SpeedCtrl();
      
    }
    
    // Motor Direc & CCR To Move
    Motor_ON();
    
    // For Data Visualization With Jscope
    Jscope_Callback();
  }
}

// ########## EXTI Callback For Limit Swtich ##########

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  
  if(GPIO_Pin == GPIO_PIN_0){
    up_exti_flag = false;
    down_exti_flag = true;
    
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
    up_exti_cnt++;
    
  }
  
  else if(GPIO_Pin == GPIO_PIN_8){
    down_exti_flag = false;
    down_exti_cnt++;
    
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
    
  }
  
}

/* ###################################
 * UART Communication Protocol ( 10 Bit ( Hex data ) )
 *
 * Start Bit                      : 0xFF                     >>  0, 1

 * Sign Bit                                                    >>  2, 4, 6
                                        + : 0x01
                                        - : 0x00
 
 * Data Bit                     : One Bit                  >>  3, 5, 7 

 * Elevation Mode bit      : One Bit                  >>  8
                                        0 : Stop
                                        1 : Up
                                        2 : Down

 * End Bit                      : CheckSum[2 ~ 8]  >>  9
 
 * CCR = RAD_PER_PULSE
 * ###################################
 */

void Uart_rx_dma_handler(){
  head_pos = huart5.RxXferSize - huart5.hdmarx->Instance->NDTR -1;
  while(tail_pos != head_pos) {
    switch (PacketMode) {
      case 0:
        
        if (rx_dma_buffer[tail_pos] == 0xFF) {         //Header check
          rx_buf[checkSize++] = rx_dma_buffer[tail_pos];
          if(checkSize == 2){
            PacketMode = 1;
          }
        }
        else{
          checkSize = 0;
        }
        break;
        
    case 1:
      
      rx_buf[checkSize++] = rx_dma_buffer[tail_pos];
      
      if(checkSize == 10){
        
        for(int i = 2 ; i < 9 ; i++)
          checkSum += rx_buf[i];
        
        if(rx_buf[9] == checkSum){
        
          // Go forward : 1
          if(rx_buf[8] == 1)
              Elevation_mode = (uint16_t)rx_buf[8];
          else{
            direc[0] = rx_buf[2];
            direc[1] = rx_buf[4];
            direc[2] = rx_buf[6];
          
            target_[0] = (double)rx_buf[3] / 10.0;
            target_[1] = (double)rx_buf[5] / 10.0;
            target_[2] = (double)rx_buf[7] / 10.0;
          }
          
          flag = true;
        }
        
        else{
          flag = false;
          
          target_[0] = 0.0;
          target_[1] = 0.0;
          target_[2] = 0.0;
          
          Elevation_mode = 0;
        }
        
        checkSum =  0;
        checkSize = 0;
        PacketMode = 0;
      }
      
      else if(checkSize > 9){
        checkSum =  0;
        checkSize = 0;
        PacketMode = 0;
      }
      
    }
    
    tail_pos++;
  }
}

double CalcMAF(int data, int n){
  MAF_cnt[n] = (MAF_cnt[n] + 1)%MAF_SIZE;
  rawData[n][MAF_cnt[n]] = data;
  SUM_Data[n] = 0;
  
  for(int i = 0; i < MAF_SIZE; i++){
    SUM_Data[n] += rawData[n][i];
   }
  
  MAF_Data[n] = SUM_Data[n]/MAF_SIZE;
  
  return MAF_Data[n];
}

void Get_Motor_Speed(){
  
  // Encoder Pulse of  Each Motors
  En_cnt_cur[0] = TIM8 -> CNT;
  En_cnt_cur[1] = TIM3 -> CNT;
  En_cnt_cur[2] = TIM4 -> CNT;
      
  for(int j = 0 ; j < 3 ; j++){
    
    // Encoder Pulse count for Positive Value
    if(direc[j] == 1){
      En_cnt_dif[j] =     ( En_cnt_cur[j] - En_cnt_pre[j] );
    }
    else{
      En_cnt_dif[j] =  - ( En_cnt_cur[j] - En_cnt_pre[j] );
    }
  
    // Encoder OverFlow
    if(En_cnt_dif[j] > 50000){
      En_cnt_dif[j] = - ( En_cnt_dif[j] - 60000 ); 
    }
    
    // Motor Speed(w) for Each Motors
    w[j] = ( En_cnt_dif[j] * 0.0748);
    
    // Keep Encoder ount for calculate speed 
    En_cnt_pre[j] = En_cnt_cur[j];
  }
  
}

void Motor_ON(){
  if(direc[0] == 1){
     HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
  }
  else if(direc[0] == 0){
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
  }
    
  if(direc[1] == 1){
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
  }
  else if(direc[1] == 0){
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
  }
        
  if(direc[2] == 1){
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);
  }
  else if(direc[2] == 0){
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);
  }
  
  for(int i = 0 ; i < 3 ; i++){
    
    if(target_[i] == 0){
      Motor_CCR[i] = 0;
      I_err_sum[i] = 0; 
    }
    
  }
  TIM1 -> CCR1 = Motor_CCR[0];
  TIM1 -> CCR2 = Motor_CCR[1];
  TIM1 -> CCR3 = Motor_CCR[2];
}
/*
 * Direc : 1 = Current ++
 * Direc : 0 = Current --
 *
 */
void CurrentCtrl(){
  
  for(int i = 0 ; i < 3 ; i++){
    // Get Current Value By ADC
    Voltage_to_Current[i] =   -(( CalcMAF(Current_ADC[i], i )/ 4096. ) * 3.3 - 2.46 ) * 10.0;
    
    // Calculate Current Error for PI Controll
    I_err_cur[i]    = I_ref[i] - Voltage_to_Current[i];
    I_err_sum[i] += I_err_cur[i];
    
    // PI Controll for Motor Voltage(With AntiWindup)
    Motor_volt[i] = (Kpc[i] * I_err_cur[i]) + (Kic[i] * I_err_sum[i]);
    
    // Anti Windup
    //##############################################
    
    if(Motor_volt[i] > Volt_max){
      I_err_sum[i] -= ( Motor_volt[i] - Volt_max ) / Kpc[i] / 3.0;
      Motor_volt[i] = Volt_max;
    }
    
    //##############################################
    //##############################################
    
    else if(Motor_volt[i] < Volt_min){
      I_err_sum[i] -= ( Motor_volt[i] - Volt_min) / Kpc[i] / 3.0;
      Motor_volt[i] = Volt_min;
    }
    
    //##############################################
    //##############################################
    
    Motor_CCR[i] = (Motor_volt[i] / 12.0) * 2100.0 + 2100.0;
  }
  
}

void SpeedCtrl(){
  
  //Get_Motor_Speed();
  
  for(int i = 0 ; i < 3 ; i++){
    err_cur[i]     = target_[i] - w[i];
    err_sum[i]  += err_cur[i];
    
    I_ref[i] = (Kps[i] * err_cur[i]) + (Kis[i] * err_sum[i]);
    
    if(I_ref[i] > I_max){
      err_sum[i] -= ( I_ref[i] - I_max ) / Kps[i] / 3.0;
      I_ref[i] = I_max;
    }
    
    else if(I_ref[i] < I_min){
      err_sum[i] -= ( I_ref[i] - I_min) / Kps[i] / 3.0;
      I_ref[i] = I_min;
    }
    
  }
}

void Jscope_Callback(){
  for(int i = 0 ; i < 3 ; i++){
    j_target_[i] = target_[i] * 100;
    j_w_[i] = w[i] * 100;
  }
}


void up_elevation(){
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);
  TIM1 -> CCR4 = 3000;
  Elevation_CCR = 3000;
  ele_motor_dir_check=1;
  // CCW
}
void down_elevation(){
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
  TIM1 -> CCR4 = 3000;
  ele_motor_dir_check=0;
  Elevation_CCR = -4200;
  // CW
}
void stop_elevation(){
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
  TIM1 -> CCR4 = 0;
  Elevation_CCR = 0;
}

void Elevation(){
  if(Elevation_mode == 1 && up_exti_flag){
    up_elevation();
  }
  else  if(Elevation_mode_ == 2 && down_exti_flag){
    down_elevation();
  }
  else{
    stop_elevation();
  }
}
void Elevation_Cnt(){
  if(down_exti_flag){
    elevation_cnt++;
    if(elevation_cnt == 4000){
      Elevation_mode_ = 2;
    }
  }
}

void Elevation_Mode(uint16_t mode){
  if(mode == 1){
    Go_cnt++;
    if(Go_cnt >= 1400){
      Stop_cnt++;
      target_[0] = target_[1] = target_[2] = 0;
      Elevation();
      
      if(Elevation_mode_ == 2 && !down_exti_flag){
        Back_cnt++;
        
        if(Back_cnt >= 1400){
          target_[0] = target_[1] = target_[2] = 0;
        }
        
        else{
          target_[0] = 4.7;
          target_[2] = 4.0;
          direc[0] = 1;
          direc[2] = 0;
        }
        
      }
      
    }
    else{
      target_[0] = 4.0;
      target_[2] = 4.9;
      direc[0] = 0;
      direc[2] = 1;
    }
  }
}


void Go_Straight_Test(){
  Go_cnt++;
  if(Go_cnt >= 2000){
    Stop_cnt++;
    
    if(Stop_cnt >= 1000){
      Back_cnt++;
      
      if(Back_cnt >= 2000){
        target_[0] = target_[1] = target_[2] = 0;
      }
      
      else{
        target_[0] = target_[2] = 4.0;
        direc[0] = 1;
        direc[2] = 0;
      }
      
    }
    
    else{
      target_[0] = target_[1] = target_[2] = 0;
    }
    
  }
  else{
    target_[0] = target_[2] = 4.0;
    direc[0] = 0;
    direc[2] = 1;
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM7_Init();
  MX_TIM1_Init();
  MX_UART5_Init();
  MX_UART4_Init();
  MX_TIM8_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  
  HAL_TIM_Base_Start_IT(&htim7);
  
  HAL_UART_Receive_DMA(&huart5, rx_dma_buffer, RX_BUF_SIZE);
  
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_4);
  
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
  
  HAL_ADC_Start_DMA(&hadc1, Current_ADC, 3);
  
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4200-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.Pulse = 2;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.Pulse = 3;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.Pulse = 4;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 60000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 60000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM7 init function */
static void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 840 - 1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 50 - 1;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM8 init function */
static void MX_TIM8_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 60000;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* UART4 init function */
static void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* UART5 init function */
static void MX_UART5_Init(void)
{

  huart5.Instance = UART5;
  huart5.Init.BaudRate = 38400;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Relay_Output_GPIO_Port, Relay_Output_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, Motor1_Pin_Pin|Motor2_Pin_Pin|Motor3_Pin_Pin|Motor4_Pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Relay_Output_Pin */
  GPIO_InitStruct.Pin = Relay_Output_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Relay_Output_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DOWN_SWITCH_Pin */
  GPIO_InitStruct.Pin = DOWN_SWITCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DOWN_SWITCH_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Motor1_Pin_Pin Motor2_Pin_Pin Motor3_Pin_Pin Motor4_Pin_Pin */
  GPIO_InitStruct.Pin = Motor1_Pin_Pin|Motor2_Pin_Pin|Motor3_Pin_Pin|Motor4_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : UP_SWITCH_Pin */
  GPIO_InitStruct.Pin = UP_SWITCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(UP_SWITCH_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
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
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
