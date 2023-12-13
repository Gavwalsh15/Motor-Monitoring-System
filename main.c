/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  *
  * Demo application to connect to a cloud based MQTT broker
  * Data communication is un-encrypted using port 1883
  *
  * Before running the code: -
  * 1. open /Core/Inc/WiFi_Credentials.h and enter WiFi access point SSID and password
  * 2. open /Core/Inc/CloudBrokerCredentials.h and enter your broker credentials
  * 3. Enter the subscribe topic name in line 376
  * 4. Enter the publish topic name in line 426
  *
  * Application Functionality
  * 1. Fetches the epoch time from st.com and uses it to start the RTC with a 1-second wake-up interrupt
  * 2. Connects to wireless access point and then to cloud DNS
  * 3. Connects to MQTT broker
  * 4. Subscribes to a single topic
  * 5. Prints date and time every second
  * 6. Publishes to a topic every 5 seconds (uses Tim6 periodic interrupt)
  * 7. Subscribe callback function runs every time a publish message is received from the broker
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <string.h>
#include "wifi.h"
#include "net_internal.h"
#include "..\..\Common\Shared\Inc\net.h"
#include "stm32l475e_iot01_tsensor.h"
#include "stm32l475e_iot01_psensor.h"
#include "WiFi_Credentials.h"
#include "CloudBrokerCredentials.h"
#include "timedate.h"
#include <time.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CONNECTION_TRIAL_MAX          10

#define WIFI_WRITE_TIMEOUT 10000
#define WIFI_READ_TIMEOUT  10000

#define CONNECTION_TRIAL_MAX          10

#define MQTT_SEND_BUFFER_SIZE             600
#define MQTT_READ_BUFFER_SIZE             600
#define MQTT_TOPIC_BUFFER_SIZE            100  /**< Maximum length of the application-defined topic names. */
#define MQTT_MSG_BUFFER_SIZE              MQTT_SEND_BUFFER_SIZE /**< Maximum length of the application-defined MQTT messages. */
#define MQTT_CMD_TIMEOUT                  5000
#define MODEL_MAC_SIZE                    13
#define MODEL_DEFAULT_MAC                 "0102030405"

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DFSDM_Channel_HandleTypeDef hdfsdm1_channel1;

I2C_HandleTypeDef hi2c2;

QSPI_HandleTypeDef hqspi;

RNG_HandleTypeDef hrng;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
uint8_t timeDisplay = 0, readTempature = 0, setLevel = 1, timeOut = 0, stopMotorFlag = 1;
uint8_t balanceError = 0;
net_hnd_t hnet;
uint32_t RPM, RPM_Count = 0;
uint8_t readRPM = 0, sliderValue = 0;

typedef struct {
  char *HostName;
  char *HostPort;
  char *ConnSecurity;
  char *MQClientId;
  char *MQUserName;
  char *MQUserPwd;
#ifdef LITMUS_LOOP
  char *LoopTopicId;
#endif
} device_config_t;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_I2C2_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM6_Init(void);
static void MX_RNG_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
int network_wr(Network* n, unsigned char* buffer, int len, int timeout_ms);
int network_rd(Network* n, unsigned char* buffer, int len, int timeout_ms);

extern int net_if_init(void * if_ctxt);
extern int net_if_deinit(void * if_ctxt);
extern int net_if_reinit(void * if_ctxt);
extern int wifi_net_if_init(void * if_ctxt);

void SPI_WIFI_ISR(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
	return len;
}

/**-------------------------------------------------------------------------------
  * @brief  EXTI line detection callback.
  * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line.
  * @retval None
  ------------------------------------------------------------------------------*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
	case (GPIO_PIN_1):
	{
		SPI_WIFI_ISR();
		break;
	}

	case BUTTON_EXTI13_Pin:
		{
			setLevel = 1;
			break;
		}
	case OPA_Pin:
	{
		RPM_Count++;
		break;
	}

    default:
    {
      break;
    }
  }
}

/*------------------------------------------------------------------
 * RTC timer event callback function
 * Runs every second
 ------------------------------------------------------------------*/
void HAL_RTCEx_WakeUpTimerEventCallback (RTC_HandleTypeDef * hrtc) {
	timeDisplay = 1;
}

/*--------------------------------------------------------------
 * Timer 6 overflow callback function
 --------------------------------------------------------------*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim) {
	if(htim->Instance == TIM6) {
		RPM = RPM_Count*30;
		RPM_Count = 0;
		readRPM = 1;
		readTempature = 1;
	}
	//used to throtle the MQTT messages set at 12 seconds
	if(htim->Instance == TIM7) {
			timeOut = 0;
	}
}

/*--------------------------------------------------------------------------
 * Subscribe message callback function
 * Called every time a publish meassage is received from a subscribed topic
 --------------------------------------------------------------------------*/
void subscribeMessageHandler(MessageData* data)
{
	static char mqtt_msg[MQTT_MSG_BUFFER_SIZE], mqtt_topic[MQTT_TOPIC_BUFFER_SIZE];
	snprintf(mqtt_msg, data->message->payloadlen+1, "%s", (char *)data->message->payload);
	snprintf(mqtt_topic, data->topicName->lenstring.len+1, "%s", data->topicName->lenstring.data);
	printf("\r\nPublished message from MQTT broker\r\n");
	printf("Topic: %s, Payload: %s\r\n\n", mqtt_topic, mqtt_msg);

	if(strstr(mqtt_topic, "motor-status")){
		if(atoi(mqtt_msg) == 1 && balanceError == 0) {
				printf("Motor On\r\n");
				motorForward();
				stopMotorFlag = 0;
			}
			else if(atoi(mqtt_msg) == 0){
				stopMotor();
			}else if (atoi(mqtt_msg) == 1 && balanceError == 1){
				//Force Puts off the motor switch on the MQTT Dash
				printf("Can't Start Motor, Please Balance\r\n");
				stopMotor();
			}
		}

	if(strstr(mqtt_topic, "slider")){
		sliderValue = atoi(mqtt_msg);
		//kicks up to full whack for 1ms so it will continue
		TIM2->CCR3 = (TIM2->ARR);
		HAL_Delay(1);
		printf("Slider Changed to %d\r\n", sliderValue);
		TIM2->CCR3 = (TIM2->ARR/100) * sliderValue;
	}

	if(strstr(mqtt_topic, "direction")){
			if(strstr(mqtt_msg, "F")) {
				motorForward();
			}else if(strstr(mqtt_msg, "R")){
				motorReverse();
			}
		}


	//cause it hangs on to the message
	memset(mqtt_msg, 0, sizeof(mqtt_msg));
}

//Motor Control Functions
	void motorForward() {
		printf("Motor Set Forward\r\n");

		HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
		HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
	}

	void motorReverse() {
		printf("Motor Reverse\r\n");

		HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);
		HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 1);
	}

	void stopMotor() {
		printf("Motor is Stopped\r\n");

		HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);
		HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
		stopMotorFlag = 1;
		sliderValue = 0;
	}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	char tempatureMsg[10];
	char speedMsg[10];
	char acceleroMsg[2];
	char motorState[2];
	char sliderMsg[4];
	char dirMsg[2];
	int32_t ret;
	float tempF;
	uint32_t tempI;
	int16_t accelero_XYZ[3];
	int16_t level_UpperLim_XYZ[3];
	int16_t level_LowerLim_XYZ[3];
	uint8_t i, command = 's', setTimout = 0, updateDir;
	int16_t deviation;


	//Network and MQTT variables
	device_config_t MQTT_Config;
	MQTTMessage mqmsg;
	MQTTClient client;
	Network network;
	MQTTPacket_connectData options = MQTTPacket_connectData_initializer;
	static unsigned char mqtt_send_buffer[MQTT_SEND_BUFFER_SIZE];
	static unsigned char mqtt_read_buffer[MQTT_READ_BUFFER_SIZE];
	net_sockhnd_t socket;
	net_ipaddr_t ipAddr;
	net_macaddr_t macAddr;

	//RTC variables
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	char timeBuffer[40];

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DFSDM1_Init();
  MX_I2C2_Init();
  MX_QUADSPI_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_RTC_Init();
  MX_TIM6_Init();
  MX_RNG_Init();
  MX_TIM7_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  printf("IoT Project\r\n\n");

  //LED off at initialisation
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  //Initialise MQTT broker structure
  //Fill in this section with MQTT broker credentials from header file
  MQTT_Config.HostName = CloudBroker_HostName;
  MQTT_Config.HostPort = CloudBroker_Port;
  MQTT_Config.ConnSecurity = "0";	//plain TCP connection with no security
  MQTT_Config.MQUserName = CloudBroker_Username;
  MQTT_Config.MQUserPwd = CloudBroker_Password;
  MQTT_Config.MQClientId = CloudBroker_ClientID;

  //Initialise WiFi network
  if (net_init(&hnet, NET_IF, (wifi_net_if_init)) != NET_OK) {
	  printf("\n\rError");
  }
  else {
	  printf("\n\rOK");
  }
  HAL_Delay(500);

  printf("\n\rRetrieving the IP address.");

  if (net_get_ip_address(hnet, &ipAddr) != NET_OK) {
	  printf("\n\rError 2");
  }
  else
  {
    switch(ipAddr.ipv) {
      case NET_IP_V4:
        printf("\n\rIP address: %d.%d.%d.%d\n\r", ipAddr.ip[12], ipAddr.ip[13], ipAddr.ip[14], ipAddr.ip[15]);
        break;
      case NET_IP_V6:
      default:
    	  printf("\n\rError 3");
    }
  }

  if (net_get_mac_address(hnet, &macAddr) == NET_OK) {
      printf("\n\rMac Address: %02x:%02x:%02x:%02x:%02x:%02x\r\n",
               macAddr.mac[0], macAddr.mac[1], macAddr.mac[2], macAddr.mac[3], macAddr.mac[4], macAddr.mac[5]);
  }

  /*
   * Fetch the epoch time from st.com and use it to set the RTC time
   */
  if (setRTCTimeDateFromNetwork(true) != TD_OK) {
	  printf("Fail setting time\r\n");
  }
  else {
	  printf("Time set, Starting RTC\r\n");
	  //RTC started with a 1-second wake-up interrupt
	  HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 2047, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
  }

  printf("Connecting to MQTT Broker\r\n\n");
  //Create network socket
  ret = net_sock_create(hnet, &socket, NET_PROTO_TCP);
  if (ret != NET_OK)
  {
    printf("\n\rCould not create the socket.\r\n");
  }
  else
  {
	  ret |= net_sock_setopt(socket, "sock_noblocking", NULL, 0);
  }

  ret = net_sock_open(socket, MQTT_Config.HostName, 1883, 0);
  if (ret != NET_OK)
	{
	  printf("\n\rCould not open the socket.");
	  HAL_Delay(1000);
	}
  else {
	  printf("\r\nConnected to server");
	  HAL_Delay(1000);
  }

  network.my_socket = socket;
  network.mqttread = (network_rd);
  network.mqttwrite = (network_wr);

  MQTTClientInit(&client, &network, MQTT_CMD_TIMEOUT, mqtt_send_buffer, MQTT_SEND_BUFFER_SIZE,
	 mqtt_read_buffer, MQTT_READ_BUFFER_SIZE);

  /* MQTT connect */
  options.clientID.cstring = MQTT_Config.MQClientId;
  options.username.cstring = MQTT_Config.MQUserName;
  options.password.cstring = MQTT_Config.MQUserPwd;

  HAL_Delay(1000);

  ret = MQTTConnect(&client, &options);
  if (ret != 0)
  {
	printf("\n\rMQTTConnect() failed: %ld\n", ret);
  }
  else
  {
	printf("\n\rConnected to MQTT Broker");
	HAL_Delay(1000);
  }
  HAL_Delay(1000);

  //Add code here to subscribe to topics
  ret = MQTTSubscribe(&client, "Gavwalsh11/feeds/slider", QOS0, (subscribeMessageHandler));
    if (ret != MQSUCCESS) {
  	printf("\n\rSubscribe failed: %ld\n\r", ret);
    }
    else {
  	printf("\n\rSubscribed to Slider topic \n\r");
  	ret = MQTTYield(&client, 500);
    }

  ret = MQTTSubscribe(&client, "Gavwalsh11/feeds/motor-status", QOS0, (subscribeMessageHandler));
	if (ret != MQSUCCESS) {
	printf("\n\rSubscribe failed: %ld\n\r", ret);
	}
	else {
	printf("\n\rSubscribed to Motor-Status topic \n\r");
	ret = MQTTYield(&client, 500);
	}

  ret = MQTTSubscribe(&client, "Gavwalsh11/feeds/direction", QOS0, (subscribeMessageHandler));
	 if (ret != MQSUCCESS) {
	 printf("\n\rSubscribe failed: %ld\n\r", ret);
	 }
	 else {
	 printf("\n\rSubscribed to direction topic \n\r");
	 ret = MQTTYield(&client, 500);
	 }




  HAL_Delay(1000);
  __HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);
  HAL_NVIC_GetPendingIRQ(TIM6_DAC_IRQn);
  HAL_TIM_Base_Start_IT(&htim6);	//start timer 6. Used for periodic interrupt
  __HAL_TIM_CLEAR_IT(&htim7, TIM_IT_UPDATE);
  HAL_TIM_Base_Start_IT(&htim7);	//start timer 7. Used for limiting the amount of messages sent this is 10 secs
  BSP_TSENSOR_Init();
  BSP_ACCELERO_Init();

  //turn off motor pins
  stopMotor();
  TIM3->CCR4 = 0;	//0% dutycycle
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

  //just so the slider always is 0 at the start with dutyCycle
  sprintf(sliderMsg, "%d", sliderValue);
  memset(&mqmsg, 0, sizeof(MQTTMessage));
  mqmsg.qos = QOS0;
  mqmsg.payload = (char *) sliderMsg;
  mqmsg.payloadlen = strlen(sliderMsg);
  MQTTPublish(&client, "Gavwalsh11/feeds/slider", &mqmsg);

  printf("\tPress H for help\r\nb:\tBalance Motor\r\nf:\tMotor Clockwise\r\nr:\tMotor Anti-Clockwise"
 		  "\r\n+:\tIncrease Speed\r\n-:\tReduce Speed\r\ns:\tMotor Stop\r\n\n\n");
   HAL_Delay(2000);
   /* USER CODE END 2 */

   /* Infinite loop */
   /* USER CODE BEGIN WHILE */
   while (1)
   {
     /* USER CODE END WHILE */

     /* USER CODE BEGIN 3 */
		/*
		 * Need to yield to test for a received publish message from broker
		 */
		ret = MQTTYield(&client, 500);


 	  if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) != 0) {
 		  HAL_UART_Receive(&huart1, &command, 1, 10);
 		  printf("Command Received %c\n\r", command);
 		  switch(command) {
 		  case 'h':
 		  case 'H':
 			  printf("\tPress H for help\r\nb:\tBalance Motor\r\nf:\tMotor Clockwise\r\nr:\tMotor Anti-Clockwise"
 					  "\r\n+:\tIncrease Speed\r\n-:\tReduce Speed\r\ns:\tMotor Stop\r\n\n\n");

 			  HAL_Delay(2000);
 			  break;
 		  case 'b':
 		  case 'B':
 			  setLevel = 1;
			  break;
		  case 's':
		  case 'S':
			  stopMotor();
			  break;
		  case 'f':
		  case 'F':
			  motorForward();
			  sprintf(dirMsg, "%s", "F");
			  updateDir = 1;
			  break;
		  case 'r':
		  case 'R':
			  motorReverse();
			  sprintf(dirMsg, "%s", "R");
			  updateDir = 1;
			  break;
		  case '+':
			  if(sliderValue < 100) {
				  sliderValue += 10;
				  printf("\r\nIncreasing Motor Speed: %d%\r\n", sliderValue);
				  sprintf(sliderMsg, "%d", sliderValue);
				  memset(&mqmsg, 0, sizeof(MQTTMessage));
				  mqmsg.qos = QOS0;
				  mqmsg.payload = (char *) sliderMsg;
				  mqmsg.payloadlen = strlen(sliderMsg);
				  MQTTPublish(&client, "Gavwalsh11/feeds/slider", &mqmsg);
				  TIM2->CCR3 = (TIM2->ARR/100) * sliderValue;
			  }
			  break;
		  case '-':
			  if(sliderValue > 0) {
				  sliderValue -= 10;
				  printf("\r\nDecreasing Motor Speed: %d%\r\n", sliderValue);
				  sprintf(sliderMsg, "%d", sliderValue);
				  memset(&mqmsg, 0, sizeof(MQTTMessage));
				  mqmsg.qos = QOS0;
				  mqmsg.payload = (char *) sliderMsg;
				  mqmsg.payloadlen = strlen(sliderMsg);
				  MQTTPublish(&client, "Gavwalsh11/feeds/slider", &mqmsg);
				  TIM2->CCR3 = (TIM2->ARR/100) * sliderValue;
			  }
			  break;
		  default: printf("Invalid Charater\r\n"); break;
		  }
	  }


	if(setLevel){
		setLevel = 0;
		balanceError = 0;

		printf("Reset Level\r\n");
		BSP_ACCELERO_AccGetXYZ(accelero_XYZ);
		for (i = 0; i < 3; i++) {
			// Calculate upper and lower limits
			deviation = abs(accelero_XYZ[i] + 500) / 10;
			level_UpperLim_XYZ[i] = accelero_XYZ[i] + deviation;
			level_LowerLim_XYZ[i] = accelero_XYZ[i] - deviation;
		}

		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0);

		sprintf(acceleroMsg, "%d", balanceError);
		printf("%s\r\n",acceleroMsg);
		memset(&mqmsg, 0, sizeof(MQTTMessage));
		mqmsg.qos = QOS0;
		mqmsg.payload = (char *) acceleroMsg;
		mqmsg.payloadlen = strlen(acceleroMsg);
		MQTTPublish(&client, "Gavwalsh11/feeds/accelero", &mqmsg);
		printf("Publishing Reset Balance Error\r\n");
	}


	/*
	 * Display date/time every second
	 */
	if(timeDisplay) {
		timeDisplay = 0;

		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
		sprintf(timeBuffer, "%02d/%02d/%02d %02d:%02d:%02d\r\n", sDate.Date, sDate.Month, sDate.Year, sTime.Hours, sTime.Minutes, sTime.Seconds);
		//printf("%s", timeBuffer);
	}


	if(readTempature && timeOut == 0){
		readTempature = 0;
		setTimout = 1;

		tempF = BSP_TSENSOR_ReadTemp();
		tempI = tempF*10;
		sprintf(tempatureMsg, "%d.%d", tempI/10,tempI%10);
		memset(&mqmsg, 0, sizeof(MQTTMessage));
		mqmsg.qos = QOS0;
		mqmsg.payload = (char *) tempatureMsg;
		mqmsg.payloadlen = strlen(tempatureMsg);
		MQTTPublish(&client, "Gavwalsh11/feeds/tempature", &mqmsg);
		printf("Publishing motor temp: %d.%d\r\n", tempI/10,tempI%10);

		if(tempF > 29){
			stopMotor();
			printf("Motor Over Heated\r\n");
		}
	}

	BSP_ACCELERO_AccGetXYZ(accelero_XYZ);
	for (i = 0; i < 3; i++) {
	    if (accelero_XYZ[i] > level_UpperLim_XYZ[i] || accelero_XYZ[i] < level_LowerLim_XYZ[i]) {
	        balanceError = 1;
	        printf("Board out of balance\r\n");
	        stopMotor();
	    }
	}



	if(balanceError && timeOut == 0){
		setTimout = 1;

		sprintf(acceleroMsg, "%d", balanceError);
		memset(&mqmsg, 0, sizeof(MQTTMessage));
		mqmsg.qos = QOS0;
		mqmsg.payload = (char *) acceleroMsg;
		mqmsg.payloadlen = strlen(acceleroMsg);
		MQTTPublish(&client, "Gavwalsh11/feeds/accelero", &mqmsg);
		printf("Publishing Balance Error\r\n");
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
		printf("Motor is off Balance\r\n");
	}


	if(stopMotorFlag && timeOut == 0){
		setTimout = 1;
		sliderValue = 0;

		sprintf(motorState, "%d", 0);
		memset(&mqmsg, 0, sizeof(MQTTMessage));
		mqmsg.qos = QOS0;
		mqmsg.payload = (char *) motorState;
		mqmsg.payloadlen = strlen(motorState);
		MQTTPublish(&client, "Gavwalsh11/feeds/motor-status", &mqmsg);
		printf("Publishing Motor is Stopped\r\n");
	}

	if(updateDir && timeOut == 0){
		updateDir = 0;
		setTimout = 1;

		memset(&mqmsg, 0, sizeof(MQTTMessage));
		mqmsg.qos = QOS0;
		mqmsg.payload = (char *) dirMsg;
		mqmsg.payloadlen = strlen(dirMsg);
		MQTTPublish(&client, "Gavwalsh11/feeds/direction", &mqmsg);
		memset(&dirMsg, 0, sizeof(dirMsg));


		sprintf(motorState, "%d", 1);
		memset(&mqmsg, 0, sizeof(MQTTMessage));
		mqmsg.qos = QOS0;
		mqmsg.payload = (char *) motorState;
		mqmsg.payloadlen = strlen(motorState);
		MQTTPublish(&client, "Gavwalsh11/feeds/motor-status", &mqmsg);
		printf("Publishing Motor is Stopped\r\n");
	}

	 if(readRPM && timeOut == 0) {
		readRPM = 0;
		setTimout = 1;

		sprintf(speedMsg, "%lu", RPM);
		memset(&mqmsg, 0, sizeof(MQTTMessage));
		mqmsg.qos = QOS0;
		mqmsg.payload = (char *) speedMsg;
		mqmsg.payloadlen = strlen(speedMsg);
		MQTTPublish(&client, "Gavwalsh11/feeds/motor-speed", &mqmsg);
		printf("Publishing Motor is Speed: %lu\r\n", RPM);
		RPM = 0;
	}



	 if(setTimout){
		 //this is kinda redundant cant think of a situation where there wont be a timeout
		 //Many have TIM7 be dynmanic?? to always match 30 pubs a min
		 setTimout = 0;
		 timeOut = 1;
	 }
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_RNG;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_channel1.Instance = DFSDM1_Channel1;
  hdfsdm1_channel1.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel1.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel1.Init.OutputClock.Divider = 2;
  hdfsdm1_channel1.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel1.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel1.Init.Input.Pins = DFSDM_CHANNEL_FOLLOWING_CHANNEL_PINS;
  hdfsdm1_channel1.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel1.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel1.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel1.Init.Awd.Oversampling = 1;
  hdfsdm1_channel1.Init.Offset = 0;
  hdfsdm1_channel1.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00000E14;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 2;
  hqspi.Init.FifoThreshold = 4;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize = 23;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */

  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 2047, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  htim2.Init.Period = 79999;
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
  sConfigOC.Pulse = 40000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 7999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 49999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 15999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 59999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, IN1_Pin|IN2_Pin|VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USB_OTG_FS_PWR_EN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPBTLE_RF_SPI3_CSN_GPIO_Port, SPBTLE_RF_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPBTLE_RF_RST_Pin|ARD_D9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPSGRF_915_SPI3_CSN_GPIO_Port, SPSGRF_915_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : M24SR64_Y_RF_DISABLE_Pin M24SR64_Y_GPO_Pin ISM43362_RST_Pin ISM43362_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin|ISM43362_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_OVRCR_EXTI3_Pin SPSGRF_915_GPIO3_EXTI5_Pin SPBTLE_RF_IRQ_EXTI6_Pin ISM43362_DRDY_EXTI1_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVRCR_EXTI3_Pin|SPSGRF_915_GPIO3_EXTI5_Pin|SPBTLE_RF_IRQ_EXTI6_Pin|ISM43362_DRDY_EXTI1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_EXTI13_Pin */
  GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_EXTI13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OPA_Pin VL53L0X_GPIO1_EXTI7_Pin LSM3MDL_DRDY_EXTI8_Pin */
  GPIO_InitStruct.Pin = OPA_Pin|VL53L0X_GPIO1_EXTI7_Pin|LSM3MDL_DRDY_EXTI8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : IN1_Pin IN2_Pin VL53L0X_XSHUT_Pin LED3_WIFI__LED4_BLE_Pin */
  GPIO_InitStruct.Pin = IN1_Pin|IN2_Pin|VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_A2_Pin ARD_A1_Pin ARD_A0_Pin */
  GPIO_InitStruct.Pin = ARD_A2_Pin|ARD_A1_Pin|ARD_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D1_Pin ARD_D0_Pin */
  GPIO_InitStruct.Pin = ARD_D1_Pin|ARD_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D7_Pin */
  GPIO_InitStruct.Pin = ARD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D13_Pin ARD_D12_Pin ARD_D11_Pin */
  GPIO_InitStruct.Pin = ARD_D13_Pin|ARD_D12_Pin|ARD_D11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D6_Pin */
  GPIO_InitStruct.Pin = ARD_D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D8_Pin ISM43362_BOOT0_Pin ISM43362_WAKEUP_Pin LED2_Pin
                           SPSGRF_915_SDN_Pin ARD_D5_Pin SPSGRF_915_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin|SPSGRF_915_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LPS22HB_INT_DRDY_EXTI0_Pin LSM6DSL_INT1_EXTI11_Pin ARD_D2_Pin HTS221_DRDY_EXTI15_Pin */
  GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI0_Pin|LSM6DSL_INT1_EXTI11_Pin|ARD_D2_Pin|HTS221_DRDY_EXTI15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_PWR_EN_Pin SPBTLE_RF_SPI3_CSN_Pin PMOD_RESET_Pin STSAFE_A100_RESET_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin|SPBTLE_RF_SPI3_CSN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : SPBTLE_RF_RST_Pin ARD_D9_Pin */
  GPIO_InitStruct.Pin = SPBTLE_RF_RST_Pin|ARD_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PMOD_SPI2_SCK_Pin */
  GPIO_InitStruct.Pin = PMOD_SPI2_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PMOD_SPI2_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PMOD_UART2_CTS_Pin PMOD_UART2_RTS_Pin PMOD_UART2_TX_Pin PMOD_UART2_RX_Pin */
  GPIO_InitStruct.Pin = PMOD_UART2_CTS_Pin|PMOD_UART2_RTS_Pin|PMOD_UART2_TX_Pin|PMOD_UART2_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D15_Pin ARD_D14_Pin */
  GPIO_InitStruct.Pin = ARD_D15_Pin|ARD_D14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Led_SetState(bool on)
{
  if (on == true)
  {
    BSP_LED_On(LED_GREEN);
  }
  else
  {
    BSP_LED_Off(LED_GREEN);
  }
}


/**
 * @brief Blink LED for 'count' cycles of 'period' period and 'duty' ON duration.
 * duty < 0 tells to start with an OFF state.
 */
void Led_Blink(int period, int duty, int count)
{
  if ( (duty > 0) && (period >= duty) )
  {
    /*  Shape:   ____
                  on |_off__ */
    do
    {
      Led_SetState(true);
      HAL_Delay(duty);
      Led_SetState(false);
      HAL_Delay(period - duty);
    } while (count--);
  }
  if ( (duty < 0) && (period >= -duty) )
  {
    /*  Shape:         ____
                __off_| on   */
    do
    {
      Led_SetState(false);
      HAL_Delay(period + duty);
      Led_SetState(true);
      HAL_Delay(-duty);
    } while (count--);
  }
}



void SPI3_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&hspi);
}/** Function to read data from the socket opened into provided buffer
 * @param - Address of Network Structure
 *        - Buffer to store the data read from socket
 *        - Expected number of bytes to read from socket
 *        - Timeout in milliseconds
 * @return - Number of Bytes read on SUCCESS
 *         - -1 on FAILURE
 **/
int network_rd(Network* n, unsigned char* buffer, int len, int timeout_ms)
{
  int bytes;

  bytes = net_sock_recv((net_sockhnd_t) n->my_socket, buffer, len);
  if(bytes < 0)
  {
    msg_error("net_sock_recv failed - %d\n", bytes);
    bytes = -1;
  }

   return bytes;
}

/** Function to write data to the socket opened present in provided buffer
 * @param - Address of Network Structure
 *        - Buffer storing the data to write to socket
 *        - Number of bytes of data to write to socket
 *        - Timeout in milliseconds
 * @return - Number of Bytes written on SUCCESS
 *         - -1 on FAILURE
 **/
int network_wr(Network* n, unsigned char* buffer, int len, int timeout_ms)
{
  int rc;

  rc = net_sock_send((net_sockhnd_t) n->my_socket, buffer, len);
  if(rc < 0)
  {
    msg_error("net_sock_send failed - %d\n", rc);
    rc = -1;
  }

  return rc;
}

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

#ifdef  USE_FULL_ASSERT
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
