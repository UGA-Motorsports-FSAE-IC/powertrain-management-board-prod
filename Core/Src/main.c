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

#define SHIFT_CAN_FRAME 172

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

FDCAN_HandleTypeDef hfdcan1;

TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */


uint32_t apps1 = 0;
uint32_t apps2 = 0;

uint32_t tps1 = 0;
uint32_t tps2 = 0;

uint32_t bs1 = 0;
uint32_t bs2 = 0;

uint32_t gearpositionsensor = 0;
uint32_t actualgear = 1; //translation from gearpositionsensor variable

FDCAN_FilterTypeDef canfilter;

Canqueue canqueue = {0};

Canqueue importantcanqueue = {0};

volatile uint8_t shift_direction = 0;
volatile uint8_t shiftcounter = 0; //random number
volatile uint8_t currentshiftcounter = 0; //random number

volatile uint32_t cancounter = 0;

uint8_t sensorflags = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM17_Init(void);
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



void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {

  cancounter += 1;

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
    FDCAN_RxHeaderTypeDef tempHeader = {0};
    uint8_t tempData[8] = {0}; 

    HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &tempHeader, tempData);

    switch (tempHeader.Identifier) {
      case SHIFT_CAN_FRAME: 
        shift_direction = tempData[0];
        shiftcounter = tempData[1];
        return;
        break;
      default:
        break;
    }

    uint8_t next = (canqueue.head + 1) % CAN_QUEUE_SIZE; 
    if (next != canqueue.tail) {
      canqueue.messagequeue[next].rxheader = tempHeader;
      memcpy(canqueue.messagequeue[next].canrxdata, tempData, sizeof(tempData));
      canqueue.head = next;
    }   
  }
}

void add_message_to_queue(FDCAN_TxHeaderTypeDef * header, uint8_t * datatosend) {
  while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0) {
    continue;
  }
  
  uint32_t errormessage = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, header, (const uint8_t *)datatosend);
  if (errormessage != HAL_OK) {
    char buffer[40] = {0};
    sprintf(buffer, "message send fail, error code %lu", errormessage);
    customprint(buffer);
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


/*
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
*/

void customprint(const char * toprint) {
  HAL_UART_Transmit(&huart1, "safety:\t", strlen("safety\t"), 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)toprint, strlen(toprint), 100);
}

int sendshiftcut() {
  return 0;
}

FDCAN_TxHeaderTypeDef txheader6;
uint8_t ackframe[8] = {0};
uint8_t currentuniqueid = 1;

HAL_StatusTypeDef sendshiftacknowledgement(int uniqueid) {
  ackframe[1] = uniqueid;
  add_message_to_queue(&txheader6, ackframe);
}


void doshift() {

  if (shiftcounter == currentshiftcounter) {
    return;
  } else {
    currentshiftcounter = shiftcounter;
    customprint("doing a shift\n");
  }
  
  switch (shift_direction) {
    case 1:
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
      HAL_Delay(150);
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
      break;
    case 2:
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
      HAL_Delay(150);
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
      break;
    default:
      customprint("error with direction\n");
      break;
  }
}

/*
uint16_t translatestep(uint16_t candidate, uint16_t * inputvalues, uint16_t * outputvalues, uint8_t steps) {
  uint16_t current_min_delta = 65535;
  uint16_t bestfitindex = 0;
  for (int i = 0; i < steps; i++) {
    if (abs(((int)inputvalues[i]) - ((int)candidate)) < current_min_delta) {
      bestfitindex = i;
    }
  }
  return outputvalues[bestfitindex];
}

uint16_t originalvalues[7] = {580, 860, 1140, 1710, 2260, 2800, 3489};
uint16_t realvalues[7] = {1, 7, 2, 3, 4, 5};
uint16_t translatedgps = 8;

volatile uint8_t translate_and_send_sensor_values = 0;
*/




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

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
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
  //HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
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

  //TIM2->CCR1 = 100; 

  //HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
  //HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_3);
  //HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4);
  //HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_5);
  HAL_TIM_Base_Start_IT(&htim17);
  
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
  HAL_Delay(40);
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
  HAL_Delay(90);
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);


  txheader6.Identifier = 50;
  txheader6.IdType = FDCAN_STANDARD_ID;
  txheader6.TxFrameType = FDCAN_DATA_FRAME;
  txheader6.DataLength = FDCAN_DLC_BYTES_8;
  txheader6.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txheader6.BitRateSwitch = FDCAN_BRS_OFF;
  txheader6.FDFormat = FDCAN_CLASSIC_CAN;
  txheader6.MessageMarker = 0;

  uint16_t cantxdata[4] = {400, 500, 600, 700};

  
  canfilter.IdType = FDCAN_STANDARD_ID;
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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); //shutdown;

  FDCAN_TxHeaderTypeDef txheader2;
  txheader2.Identifier = 50;
  txheader2.IdType = FDCAN_STANDARD_ID;
  txheader2.TxFrameType = FDCAN_DATA_FRAME;
  txheader2.DataLength = FDCAN_DLC_BYTES_8;
  txheader2.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txheader2.BitRateSwitch = FDCAN_BRS_OFF;
  txheader2.FDFormat = FDCAN_CLASSIC_CAN;
  txheader2.MessageMarker = 0;
  uint8_t * txheader2data = {0};


  while (1)
  {

    loopcounter++;

    /*
    if (translate_and_send_sensor_values) {
      translatedgps = translatestep(gearpositionsensor, originalvalues, realvalues, sizeof(realvalues));
      char gearbuffer[40] = {0};
      txheader2data[0] = translatedgps;
      sprintf(translatedgps, "the actual gear is %u", translatedgps);
      customprint(gearbuffer);
      add_message_to_queue(&txheader2, txheader2data);
      translate_and_send_sensor_values = 0;
    }
      */
      
    doshift();


    if (canqueue.tail != canqueue.head) { //dequeue from normal queue

      uint32_t identifier = canqueue.messagequeue[canqueue.tail].rxheader.Identifier;

      switch (identifier) {
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
        case 213:
          gearpositionsensor = ((uint16_t *)(canqueue.messagequeue[canqueue.tail].canrxdata))[1];
        default:
          char otherbuffer[40] = {0};
          sprintf(otherbuffer, "misc message from : %u\n", identifier);
          break;
      }

      canqueue.tail = (canqueue.tail + 1) % CAN_QUEUE_SIZE;

      if (printcan) {
        if ((loopcounter % 100000) == 0) {
          //HAL_UART_Transmit(&huart1, finalbuffer, strlen(finalbuffer), 100);
        }
        //HAL_UART_Transmit(&huart1, finalbuffer, strlen(finalbuffer), 100);
        
        if (sensorflags == 0b00111111) {
          char sensorbuffer[600] = {0};
          sprintf(sensorbuffer, "apps1: %u\napps2: %u\ntps1: %u\ntps2: %u\nbs1: %u\nbs2: %u\ngps: %u\n\n", apps1, apps2, tps1, tps2, bs1, bs2, gearpositionsensor);
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

    if (gotcommand) {

      HAL_UART_Transmit(&huart1, (uint8_t *)intbuffer, 40 - fulllen, 100); 
      char counterbuffer[40] = {0};
      sprintf(counterbuffer, "\nloop counter: %lu, cancounter: %lu\n", loopcounter, cancounter);  
      HAL_UART_Transmit(&huart1, counterbuffer, strlen(counterbuffer), 100);


      if (!strcmp(intbuffer, "togglecanprint")) {
        printcan = !printcan;
      }

      /*
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

      */

      if (!strcmp(intbuffer, "led")) {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
      }

      if (!strcmp(intbuffer, "shift")) {
        int receivedbytes = getcommand((uint8_t *)buffer, 40);
        if (!strcmp(buffer, "down")) {
          shiftcounter++;
          shift_direction = 2;
          /*
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET); //upshift  
          HAL_Delay(80);
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); //upshift  
          HAL_UART_Transmit(&huart1, "shifted up\n", strlen("shifted up\n"), 100);          
          */
          
        } else if (!strcmp(buffer, "up")) {
          shiftcounter++;
          shift_direction = 1;
          /*
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET); //downshift
          HAL_Delay(80);
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET); //downshift
          HAL_UART_Transmit(&huart1, "shifted down\n", strlen("shifted down\n"), 100);
          */
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
      
      /*
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
      */

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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV4;

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
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 7;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 65535;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

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

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
