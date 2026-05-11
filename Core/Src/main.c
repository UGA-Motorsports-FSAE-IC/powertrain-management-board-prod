/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32c092xx.h"
#include "stm32c0xx_hal.h"
#include "stm32c0xx_hal_fdcan.h"
#include "stm32c0xx_hal_gpio.h"
#include "stm32c0xx_hal_tim.h"
#include "stm32c0xx_hal_uart.h"
#include "string.h"
#include "stdlib.h"
#include <stdint.h>
#include <stdio.h>
#include <sys/_intsup.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MAX_FORWARD_POWER 1200
#define MAX_BACKWARD_POWER -1200


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

FDCAN_HandleTypeDef hfdcan1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t onebyte;
volatile uint8_t gotcommand = 0;
volatile int fulllen = 40;
char intbuffer[40] = {0};

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  intbuffer[40 - fulllen] = onebyte;

  if (onebyte == '!') {
    intbuffer[40 - fulllen] = '\0';
    gotcommand = 1;
    return;
  } else {
    fulllen--;
  }
  
  HAL_UART_Receive_IT(&huart1, &onebyte, 1);
}

volatile uint8_t do_start_piston_actuation = 0;
volatile uint8_t canframe_missing = 0; //use tim1 channel 1  //ACTUALLY TIM2
volatile uint8_t tps_implausibility = 0; //use tim1 channel 3
volatile uint8_t apps_implausibility = 0; //use tim1 channel 4
volatile uint8_t bse_implausibility = 0; //use tim1 channel 5
volatile uint8_t throttle_return = 0;
volatile uint8_t throttle_return_delay_passed = 0;
volatile uint8_t throttle_stuck = 0; //use tim3 channel 1
volatile uint8_t safe_to_enable_fuel_throttle = 0; //use tim3 channel 3
volatile uint8_t maximum_shift_time_passed = 0;
uint32_t apps1 = 0;
uint32_t apps2 = 0;
uint32_t tps1 = 0;
uint32_t tps2 = 0;
uint32_t bs1 = 0;
uint32_t bs2 = 0;
volatile uint8_t downshift_commanded = 0;
uint8_t set_throttle_to_0 = 0;
uint32_t throttle_intended = 0;
uint32_t tps1_actuation = 0;
uint32_t tps2_actuation = 0;
uint32_t tps_difference = 0;
uint32_t gps = 0;
uint32_t apps1_actuation = 0;
uint32_t apps2_actuation = 0;
uint32_t apps_difference = 0;
uint32_t bse1_actuation = 0;
uint32_t bse2_actuation = 0;
uint8_t doing_downshift = 0;
uint32_t throttle_flap_actuation = 0;
volatile uint8_t allow_throttle_blip = 0;
uint32_t gearpositionsensor = 0;
uint32_t actualgear = 1;

FDCAN_FilterTypeDef canfilter;

Canqueue canqueue = {0};

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {

    uint8_t next = (canqueue.head + 1) % CAN_QUEUE_SIZE; 

    if (next != canqueue.tail) {
      HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &(canqueue.messagequeue[next].rxheader), canqueue.messagequeue[next].canrxdata);
      canqueue.head = next;
    }   
  }
}

int getcommand(uint8_t * buffer, int maxlen) {
  memset(buffer, 0, maxlen);
  uint8_t singlebyte = 0;
  while (maxlen) {
    HAL_UART_Receive(&huart1, &singlebyte, 1, 0xFFFF);
    buffer[40 - maxlen] = singlebyte;

    if (singlebyte == '!') {
      buffer[40 - maxlen] = '\0';
      break;
    }

    maxlen--;
  }

  return maxlen;
}


uint8_t sensorflags = 0;

int gp = 80;
int gi = 20;
int gd = 70;
int integral = 0;

int gpr = 80;
int gir = 20;
int gdr = 70;

int targetvalue = 2000;
int previouserror = 0;
int dt = 1;

int pid_output_value() {
  int error = targetvalue - (int)tps1;

  int proportion;
  int derivative;

  if (error > 0) {
    proportion = ((error * gp) / 100);
    integral = integral + ((dt * error * gi) / 100);
    derivative = (((error - previouserror) * gd) / 100);    
  } else {
    proportion = ((error * gpr) / 100);
    integral = integral + ((dt * error * gir) / 100);
    derivative = (((error - previouserror) * gdr) / 100);    
  }

  previouserror = error;

  int finalresult = proportion + integral + derivative;
  if (finalresult > MAX_FORWARD_POWER) {
    return (int)MAX_FORWARD_POWER;
  } else if (finalresult < MAX_BACKWARD_POWER) {
    return (int)MAX_BACKWARD_POWER;
  } else {
    return finalresult;
  }
}

volatile uint8_t ocdelayflag = 0;

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM1) {
    ocdelayflag = 1;
    switch (htim->Channel) { //100ms
      case HAL_TIM_ACTIVE_CHANNEL_1:
        canframe_missing = 1;
        break;
      case HAL_TIM_ACTIVE_CHANNEL_3:
        tps_implausibility = 1;
        break;
      case HAL_TIM_ACTIVE_CHANNEL_4:
        apps_implausibility = 1;
        break;
      case HAL_TIM_ACTIVE_CHANNEL_5:
        bse_implausibility = 1;
        break;
      default:
        break;
    }
  }

  if (htim->Instance == TIM15) { //1 second
    switch (htim->Channel) {
      case HAL_TIM_ACTIVE_CHANNEL_1:
        //throttle_stuck = 1;
        throttle_return = 1;
        break;
      case HAL_TIM_ACTIVE_CHANNEL_2:
        throttle_return_delay_passed = 1;
        break;
      default:
        break;
    }
  }
  if (htim->Instance == TIM16) { //1.337 second
    switch (htim->Channel) {
      case HAL_TIM_ACTIVE_CHANNEL_1:
        safe_to_enable_fuel_throttle = 1;
        break;
      default:
        break;
    }
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM17) { //40ms timer for starting shifter pneumatic piston actuation after throttle blip start
    do_start_piston_actuation = 1;
  }
  if (htim->Instance == TIM14) { //200ms timer for canceling the throttle blip if shift still hasn't completed
    maximum_shift_time_passed = 1;
  }
}

void customprint(const char * toprint) {
  HAL_UART_Transmit(&huart1, "safety:\t", strlen("safety\t"), 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)toprint, strlen(toprint), 100);
}



int sendshiftcut() {
  return 0;
}

FDCAN_TxHeaderTypeDef txheader6;
uint8_t ackframe[8] = {0};
uint8_t currentuniqueid = 0;

HAL_StatusTypeDef sendshiftacknowledgement(int uniqueid) {

  ackframe[1] = uniqueid;

  return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txheader6, ackframe);
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  txheader6.Identifier = 10;
  txheader6.IdType = FDCAN_EXTENDED_ID;
  txheader6.TxFrameType = FDCAN_DATA_FRAME;
  txheader6.DataLength = FDCAN_DLC_BYTES_8;
  txheader6.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txheader6.BitRateSwitch = FDCAN_BRS_OFF;
  txheader6.FDFormat = FDCAN_CLASSIC_CAN;
  txheader6.MessageMarker = 0;

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_FDCAN1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_BLUE);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  FDCAN_FilterTypeDef canfilter;
  FDCAN_TxHeaderTypeDef txheader6;

  char buffer[40] = {0};
  TIM2->CCR1 = 100; 

  HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_3);
  HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4);
  HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_5);
  
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
  HAL_Delay(40);
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
  HAL_Delay(40);
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);


  txheader6.Identifier = 166473;
  txheader6.IdType = FDCAN_EXTENDED_ID;
  txheader6.TxFrameType = FDCAN_DATA_FRAME;
  txheader6.DataLength = FDCAN_DLC_BYTES_8;
  txheader6.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txheader6.BitRateSwitch = FDCAN_BRS_OFF;
  txheader6.FDFormat = FDCAN_CLASSIC_CAN;
  txheader6.MessageMarker = 0;

  uint16_t cantxdata[4] = {400, 500, 600, 700};

  
  canfilter.IdType = FDCAN_EXTENDED_ID;
  canfilter.FilterIndex = 0;
  canfilter.FilterType = FDCAN_FILTER_MASK;
  canfilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  canfilter.FilterID1 = 0x000;
  canfilter.FilterID2 = 0x000;  

  HAL_FDCAN_ConfigFilter(&hfdcan1, &canfilter);

  HAL_FDCAN_Start(&hfdcan1);

  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  HAL_UART_Receive_IT(&huart1, &onebyte, 1);

  uint32_t loopcounter = 0;

  uint8_t printcan = 0;

  uint8_t dopid = 1;
  uint8_t printpid = 1;

  uint8_t useapps = 1;

  uint8_t safetyprint = 1;

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET); //pololu direction
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); //throttle relay

  

  while (1)
  {

    loopcounter++;

    /*
    if (useapps) { 
      if (set_throttle_to_0) {
        targetvalue = 680;
      } else {
        targetvalue = (73 * apps2 / 33) - 4279;
      }
    }

    if (set_throttle_to_0) {
      targetvalue = 680;
    }

    if (dopid) {
      if ((loopcounter % 1000) == 0) {
        int value = pid_output_value();
        if (set_throttle_to_0) {
          if (safetyprint) {
            customprint("throttle is set to 0\n");
          }
        }
        if (value < 0) {
          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET); //pololu direction reverse
          TIM2->CCR1 = (value * -1);
        } else {
          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET); //pololu direction forward
          TIM2->CCR1 = value;
        }
        
        if (printpid) {
          char pidbuffer[40] = {0};
          sprintf(pidbuffer, "pid set to %d, intended is %u, actual is %u\np: %u, i: %u, d: %u, pr: %u, ir: %u, dr: %u\n", value, targetvalue, tps1, gp, gi, gd, gpr, gir, gdr);
          HAL_UART_Transmit(&huart1, pidbuffer, strlen(pidbuffer), 100);
        }
      }
    }
    */

    if (canqueue.tail != canqueue.head) {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, __HAL_TIM_GET_COUNTER(&htim1) - 10);
      canframe_missing = 0;
      
      char idbuffer[40] = {0};
      sprintf("the id was %u\n", canqueue.messagequeue[canqueue.tail].rxheader.Identifier);
      customprint(idbuffer);
      switch (canqueue.messagequeue[canqueue.tail].rxheader.Identifier) {
        case 210:
          apps1 = ((uint16_t *)(canqueue.messagequeue[canqueue.tail].canrxdata))[0];
          sensorflags |= 0b00000001;
          break;
        case 215:
          apps2 = ((uint16_t *)(canqueue.messagequeue[canqueue.tail].canrxdata))[3];
          tps1 = ((uint16_t *)(canqueue.messagequeue[canqueue.tail].canrxdata))[2];
          tps2 = ((uint16_t *)(canqueue.messagequeue[canqueue.tail].canrxdata))[1];
          bs1 = ((uint16_t *)(canqueue.messagequeue[canqueue.tail].canrxdata))[0];
          sensorflags |= 0b00011110;
          break;
        case 214:
          bs2 = ((uint16_t *)(canqueue.messagequeue[canqueue.tail].canrxdata))[3];
          sensorflags |= 0b00100000;
          break;
        case 10:
          uint8_t uniqueid = (canqueue.messagequeue[canqueue.tail].canrxdata)[1];
          uint8_t shiftcommand = (canqueue.messagequeue[canqueue.tail].canrxdata)[0];

          if (uniqueid <= currentuniqueid) {
            char uniqueidbuffer[40] = {0};
            sprintf(uniqueidbuffer, "the uniqueid is %u, currentuniqueid is %u\n", uniqueid, currentuniqueid);            
            customprint(uniqueidbuffer);
          } else {
            customprint("shifting the gears\n");
            if (shiftcommand == 1 && actualgear > 1) { //downshift
              HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET); //downshift
              HAL_Delay(80);
              HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET); //downshift
            }

            if (shiftcommand == 2 && actualgear < 6) {
              int sendresult = sendshiftcut();
              HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET); //upshift  
              HAL_Delay(80);
              HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); //upshift  
            }
            
            currentuniqueid++;
          }

          sendshiftacknowledgement(uniqueid);
          break;
        default:
          char otherbuffer[40] = {0};
          sprintf(otherbuffer, "recieved can message %u\n", canqueue.messagequeue[canqueue.tail].rxheader.Identifier);
          HAL_UART_Transmit(&huart1, otherbuffer, strlen(otherbuffer), 100);
          break;
      }

      canqueue.tail = (canqueue.tail + 1) % CAN_QUEUE_SIZE;

      if (printcan) {
        if ((loopcounter % 100000) == 0) {
          //HAL_UART_Transmit(&huart1, finalbuffer, strlen(finalbuffer), 100);
          if (sensorflags == 0b00111111) {
            char sensorbuffer[600] = {0};
            sprintf(sensorbuffer, "apps1: %u\napps2: %u\ntps1: %u\ntps2: %u\nbs1: %u\nbs2: %u\n\n", apps1, apps2, tps1, tps2, bs1, bs2);
            HAL_UART_Transmit(&huart1, sensorbuffer, strlen(sensorbuffer), 100);
            sensorflags = 0x00000000;
          } 
        }
        //HAL_UART_Transmit(&huart1, finalbuffer, strlen(finalbuffer), 100);
        if (sensorflags == 0b00111111) {
          char sensorbuffer[600] = {0};
          sprintf(sensorbuffer, "apps1: %u\napps2: %u\ntps1: %u\ntps2: %u\nbs1: %u\nbs2: %u\n\n", apps1, apps2, tps1, tps2, bs1, bs2);
          HAL_UART_Transmit(&huart1, sensorbuffer, strlen(sensorbuffer), 100);
          sensorflags = 0x00000000;
        }
      }
      
    } else if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) == 3) {
      FDCAN_RxHeaderTypeDef tempHeader;
      uint8_t tempData[8]; 
      while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0)) {
        HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &tempHeader, tempData);
      }
      canqueue.head = 0;
      canqueue.tail = 0;
    }



    /*
    if (ocdelayflag) {
      customprint("the oc callback was called\n");
      ocdelayflag = 0;
    }

    tps1_actuation = tps1;                //min value is 410, range is 3276
    tps2_actuation = 3836 - tps2;         //min value is 410, range is 3276 //3384 - tps2
    tps_difference = (abs(((int)tps1_actuation) - ((int)tps2_actuation)) * 1000) / 3200; //difference value will range from 0-1000, representing 0%-100%
    if (tps1 < 3999 && tps2 < 3999 && tps1 > 100 && tps2 > 100 && tps_difference < 200) { //if both tps's have no short circuit or disconnection, and difference between them is less than 8%
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, __HAL_TIM_GET_COUNTER(&htim1) - 10); //then reset timer to 0 since there is no error with tps
      tps_implausibility = 0;
    } else if (safetyprint) {
      customprint("tps implausibility\n");
    }

    apps1_actuation = (uint32_t)(((-9) * ((int)apps1) / 10) + 3830);            
    apps2_actuation = apps2;                        
    apps_difference = (abs(((int)apps1_actuation) - ((int)apps2_actuation)) * 1000) / 1300; //difference value will range from 0-1000, representing 0%-100%
    if (apps1 < 3500 && apps2 < 3500  && apps1 > 100 && apps2 > 100 && apps_difference < 200) { //if both apps's have no short circuit or disconnection, and difference between them is less than 8%
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, __HAL_TIM_GET_COUNTER(&htim1) - 10); //then reset timer to 0 since there is no error with apps
      apps_implausibility = 0;
    } else if (safetyprint) {
      customprint("apps implausibility\n");
      char appsbuffer[40];
      sprintf(appsbuffer, "safety: apps1: %u, apps2: %u\n", apps1, apps2);
      HAL_UART_Transmit(&huart1, appsbuffer, strlen(appsbuffer), 100);
    }

    if (bs1 < 3999 && bs2 < 3999 && bs1 > 100 && bs2 > 100) { //if both bse's have no short circuit or disconnection
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_5, __HAL_TIM_GET_COUNTER(&htim1) - 10); //then reset timer to 0 since there is no error with tps
      bse_implausibility = 0;
    } else if (safetyprint) {
      customprint("bse implausibility\n");
    }    

    if (canframe_missing || tps_implausibility || apps_implausibility || bse_implausibility) {
      //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); //throttle pin
      set_throttle_to_0 = 1;
      if (safetyprint) {
        customprint("throttle relay off\n");
      }
    }

    uint8_t not_braking_and_throttle = !((bs1 < 901 && bs2 < 1556) && (tps1_actuation < 600 && tps2_actuation < 600)); //if either brake sensor measures hard braking, while either tps measures open throttle
    uint8_t throttle_target_near_actual = (abs(((int16_t)tps1_actuation) - ((int16_t)targetvalue)) * 1000) / 3276 < 200; //if difference between desired value and intended throttle position is >8%

    if (throttle_target_near_actual &&  not_braking_and_throttle) {
      __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, __HAL_TIM_GET_COUNTER(&htim15) - 10); //throttle_return set to TRUE if this line not called

      if (safe_to_enable_fuel_throttle) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
        throttle_return = 0;
        throttle_return_delay_passed = 0;
      }
    } else { 
      __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, __HAL_TIM_GET_COUNTER(&htim16) - 10); //safe_to_enable_fuel_throttle set to TRUE if this line not called
    }

    if (!not_braking_and_throttle) {
      set_throttle_to_0 = 1;
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    }
    if (throttle_return) {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
      set_throttle_to_0 = 1;
    } else {
      __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, __HAL_TIM_GET_COUNTER(&htim15) - 10); //throttle_return_delay_passed set to TRUE if this line not called
    }

    if (throttle_return_delay_passed) {
      if (safetyprint) {
        customprint("throttle return delay passe\n");
      }
      safe_to_enable_fuel_throttle = 0;
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    }

    if (throttle_target_near_actual && not_braking_and_throttle && !canframe_missing && !tps_implausibility && !apps_implausibility && !bse_implausibility) {
      //only if throttle target is near intended, no simultaneous throttle and braking, and all sensors are recieved and working, then the throttle is allowed to be powered. 
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
      set_throttle_to_0 = 0;  
    }
    */



    if (gotcommand) {
      HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txheader6, (uint8_t *)(cantxdata));

      HAL_UART_Transmit(&huart1, (uint8_t *)intbuffer, 40 - fulllen, 100); 
      char counterbuffer[40] = {0};
      sprintf(counterbuffer, "\nloop counter: %lu\n", loopcounter);
      HAL_UART_Transmit(&huart1, counterbuffer, strlen(counterbuffer), 100);

      if (!strcmp(intbuffer, "togglecanprint")) {
        printcan = !printcan;
      }

      if (!strcmp(intbuffer, "togglepidprint")) {
        printpid = !printpid;
      }

      if (!strcmp(intbuffer, "pid")) {
        dopid = !dopid;
      }

      if (!strcmp(intbuffer, "safety")) {
        safetyprint = !safetyprint;
      }

      if (!strcmp(intbuffer, "useapps")) {
        useapps = !useapps;
        if (useapps) {
          HAL_UART_Transmit(&huart1, "using apps for tps\n", strlen("using apps for tps\n"), 100);
        } else {
          HAL_UART_Transmit(&huart1, "not using apps for tps\n", strlen("using apps for tps\n"), 100);
        }
      }

      if (!strcmp(intbuffer, "setp")) {
        int receivedbytes = getcommand((uint8_t *)buffer, 40);
        int newgp = atoi(buffer);
        if (newgp >= 0) {
          gp = newgp;
          char pwmbuffer[40] = {0};
          sprintf(pwmbuffer, "p is %d%%", gp);
          HAL_UART_Transmit(&huart1, pwmbuffer, strlen(pwmbuffer), 100);
        }
      }

      if (!strcmp(intbuffer, "seti")) {
        int receivedbytes = getcommand((uint8_t *)buffer, 40);
        int newgi = atoi(buffer);
        if (newgi >= 0) {
          gi = newgi;
          char pwmbuffer[40] = {0};
          sprintf(pwmbuffer, "i is %d%%", gi);
          HAL_UART_Transmit(&huart1, pwmbuffer, strlen(pwmbuffer), 100);
        }
      }

      if (!strcmp(intbuffer, "setd")) {
        int receivedbytes = getcommand((uint8_t *)buffer, 40);
        int newgd = atoi(buffer);
        if (newgd >= 0) {
          gd = newgd;
          char pwmbuffer[40] = {0};
          sprintf(pwmbuffer, "d is %d%%", gd);
          HAL_UART_Transmit(&huart1, pwmbuffer, strlen(pwmbuffer), 100);
        }
      }

      if (!strcmp(intbuffer, "setpr")) {
        int receivedbytes = getcommand((uint8_t *)buffer, 40);
        int newgp = atoi(buffer);
        if (newgp >= 0) {
          gpr = newgp;
          char pwmbuffer[40] = {0};
          sprintf(pwmbuffer, "pr is %d%%", gpr);
          HAL_UART_Transmit(&huart1, pwmbuffer, strlen(pwmbuffer), 100);
        }
      }

      if (!strcmp(intbuffer, "setir")) {
        int receivedbytes = getcommand((uint8_t *)buffer, 40);
        int newgd = atoi(buffer);
        if (newgd >= 0) {
          gir = newgd;
          char pwmbuffer[40] = {0};
          sprintf(pwmbuffer, "ir is %d%%", gir);
          HAL_UART_Transmit(&huart1, pwmbuffer, strlen(pwmbuffer), 100);
        }
      }

      if (!strcmp(intbuffer, "setdr")) {
        int receivedbytes = getcommand((uint8_t *)buffer, 40);
        int newgd = atoi(buffer);
        if (newgd >= 0) {
          gdr = newgd;
          char pwmbuffer[40] = {0};
          sprintf(pwmbuffer, "dr is %d%%", gdr);
          HAL_UART_Transmit(&huart1, pwmbuffer, strlen(pwmbuffer), 100);
        }
      }

      if (!strcmp(intbuffer, "setintended")) {
        int receivedbytes = getcommand((uint8_t *)buffer, 40);
        int newintended = atoi(buffer);
        if (newintended > 0) {
          targetvalue = newintended;
          char pwmbuffer[40] = {0};
          sprintf(pwmbuffer, "intended is %d%%", targetvalue);
          HAL_UART_Transmit(&huart1, pwmbuffer, strlen(pwmbuffer), 100);
        }
      }

      if (!strcmp(intbuffer, "led")) {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
      }

      if (!strcmp(intbuffer, "shift")) {
        int receivedbytes = getcommand((uint8_t *)buffer, 40);
        if (!strcmp(buffer, "up")) {
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET); //upshift  
          HAL_Delay(80);
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); //upshift  
          HAL_UART_Transmit(&huart1, "shifted up\n", strlen("shifted up\n"), 100);          
        } else if (!
          strcmp(buffer, "down")) {
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET); //downshift
          HAL_Delay(80);
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET); //downshift
          HAL_UART_Transmit(&huart1, "shifted down\n", strlen("shifted down\n"), 100);
        }
      } 
      
      if (!strcmp(intbuffer, "shutdown")) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8); //shutdown
        HAL_UART_Transmit(&huart1, "toggled shutdown\n", strlen("toggled shutdown\n"), 100);
      }

      if (!strcmp(intbuffer, "throttle")) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1); //throttlebody
        HAL_UART_Transmit(&huart1, "toggled throttle\n", strlen("toggle throttle\n"), 100);
      }

      if (!strcmp(intbuffer, "dirchange")) {
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_1);
        HAL_UART_Transmit(&huart1, "changed direction\n", strlen("changed direction\n"), 100);
      }

      if (!strcmp(intbuffer, "pwm")) {
        int receivedbytes = getcommand((uint8_t *)buffer, 40);
        int pwmvalue = atoi(buffer);
        if (pwmvalue > 0) {
          TIM1->CCR1 = (unsigned int)pwmvalue;
          int percentvalue = pwmvalue / 20;
          char pwmbuffer[40] = {0};
          sprintf(pwmbuffer, "throttle pwm is %d%%", percentvalue);
          HAL_UART_Transmit(&huart1, pwmbuffer, strlen(pwmbuffer), 100);
        }
      }
      memset(intbuffer, 0, fulllen);
      gotcommand = 0;
      fulllen = 40;
      HAL_UART_Receive_IT(&huart1, &onebyte, 1);
    }

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 3;
  hfdcan1.Init.NominalTimeSeg1 = 20;
  hfdcan1.Init.NominalTimeSeg2 = 3;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 11;
  hfdcan1.Init.DataTimeSeg1 = 12;
  hfdcan1.Init.DataTimeSeg2 = 11;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 99;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 38399;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_6) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 999;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 38399;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 999;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, throttlerelay_Pin|upshift_Pin|downshift_Pin|shutdown_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(dir_GPIO_Port, dir_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : throttlerelay_Pin upshift_Pin downshift_Pin shutdown_Pin */
  GPIO_InitStruct.Pin = throttlerelay_Pin|upshift_Pin|downshift_Pin|shutdown_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : dir_Pin */
  GPIO_InitStruct.Pin = dir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(dir_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : led_Pin */
  GPIO_InitStruct.Pin = led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(led_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
