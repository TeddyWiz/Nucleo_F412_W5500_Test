/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "loopback.h"
#include "wizchip_conf.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ETH_MAX_BUF_SIZE 2048
#define True_STD
#define SEND_SIZE 41820
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
wiz_NetInfo defaultNetInfo = { .mac = {0x00,0x08,0xdc,0xff,0xee,0xdd},
							.ip = {192,168,15,111},
							.sn = {255,255,255,0},
							.gw = {192,168,15,1},
							.dns = {168, 126, 63, 1},
							.dhcp = NETINFO_STATIC};

unsigned char gServer_IP[4] = {192,168,15,7};
unsigned char ethBuf0[ETH_MAX_BUF_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
uint8_t rxData;
uint8_t send_flag = 0;
uint8_t ETH_INT_Flag = 0;

uint8_t rx_buffer[2048]= {0,};
int rx_index = 0;
uint8_t rx_flag =0;
#ifdef KEIL
     #ifdef __GNUC__
     //With GCC, small printf (option LD Linker->Libraries->Small printf
     //set to 'Yes') calls __io_putchar()
         #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
		#else
				 #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
		#endif /* __GNUC__*/
#endif

#ifdef True_STD
	int _write(int fd, char *str, int len)
	{
		for(int i=0; i<len; i++)
		{
			HAL_UART_Transmit(&huart3, (uint8_t *)&str[i], 1, 0xFFFF);
		}
		return len;
	}
#endif

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

    /*
        This will be called once data is received successfully,
        via interrupts.
    */

     /*
       loop back received data
     */
     HAL_UART_Receive_IT(&huart3, &rxData, 1);
     if (rxData == '\n')
	 {
	   if (rx_buffer[rx_index - 1] == '\r')
	   {
		 rx_index--;
		 rx_flag = 1;
		 rx_buffer[rx_index] = 0;
	   }
	   else
	   {
		 rx_index = 0;
		 HAL_UART_Transmit(&huart3, "not support format\r\n", 20, 1000);
	   }
	 }
	 else if (rxData == 0x08) // back space
	 {
	   rx_index--;
	 }
	 else
	 {
	   rx_buffer[rx_index++] = rxData;
	 }
     HAL_UART_Transmit(&huart3, &rxData, 1, 1000);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == W5500_INTn_Pin)
	{
		ETH_INT_Flag = 1;
	}
}
void csEnable(void)
{
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
}

void csDisable(void)
{
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}

void spiWriteByte(uint8_t tx)
{
	uint8_t rx;
  while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
	HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 10);
  while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
  
}

uint8_t spiReadByte(void)
{
	uint8_t rx = 0, tx = 0xFF;
  while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
	HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 10);
  while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_RX);
	return rx;
}
static void wizchip_read_burst(uint8_t *pBuf, uint16_t len)
{
#if 1
	HAL_SPI_Receive_DMA(&hspi1, pBuf, len);
  while (HAL_DMA_GetState(hspi1.hdmarx) == HAL_DMA_STATE_BUSY);
  while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_RX);
#else
  uint8_t tx[SEND_SIZE] = {0,};
    while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
  	HAL_SPI_TransmitReceive(&hspi1, &tx, pBuf, len, 2);
    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_RX);
#endif
  
	//return;
}

static void wizchip_write_burst(uint8_t *pBuf, uint16_t len)
{
#if 1
  HAL_SPI_Transmit_DMA(&hspi1, pBuf, len);
  while (HAL_DMA_GetState(hspi1.hdmatx) == HAL_DMA_STATE_BUSY);
  while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
#else
  uint8_t rx[SEND_SIZE] = {0,};
    while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
  	HAL_SPI_TransmitReceive(&hspi1, pBuf, &rx, len, 2);
    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_RX);
#endif
  
	//return;
}
void print_network_information()
{
	wiz_NetInfo gWIZNETINFO;
	wizchip_getnetinfo(&gWIZNETINFO);

	printf("Mac address: %02x:%02x:%02x:%02x:%02x:%02x\r\n",gWIZNETINFO.mac[0],gWIZNETINFO.mac[1],gWIZNETINFO.mac[2],gWIZNETINFO.mac[3],gWIZNETINFO.mac[4],gWIZNETINFO.mac[5]);
	printf("IP address : %d.%d.%d.%d\r\n",gWIZNETINFO.ip[0],gWIZNETINFO.ip[1],gWIZNETINFO.ip[2],gWIZNETINFO.ip[3]);
	printf("SM Mask	   : %d.%d.%d.%d\r\n",gWIZNETINFO.sn[0],gWIZNETINFO.sn[1],gWIZNETINFO.sn[2],gWIZNETINFO.sn[3]);
	printf("Gate way   : %d.%d.%d.%d\r\n",gWIZNETINFO.gw[0],gWIZNETINFO.gw[1],gWIZNETINFO.gw[2],gWIZNETINFO.gw[3]);
	printf("DNS Server : %d.%d.%d.%d\r\n",gWIZNETINFO.dns[0],gWIZNETINFO.dns[1],gWIZNETINFO.dns[2],gWIZNETINFO.dns[3]);
}

void Display_Net_Conf()
{
	uint8_t tmpstr[6] = {0,};
	wiz_NetInfo gWIZNETINFO;

	ctlnetwork(CN_GET_NETINFO, (void*) &gWIZNETINFO);
	ctlwizchip(CW_GET_ID,(void*)tmpstr);

	// Display Network Information
	if(gWIZNETINFO.dhcp == NETINFO_DHCP) printf("\r\n===== %s NET CONF : DHCP =====\r\n",(char*)tmpstr);
		else printf("\r\n===== %s NET CONF : Static =====\r\n",(char*)tmpstr);

	printf("\r\nMAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n", gWIZNETINFO.mac[0], gWIZNETINFO.mac[1], gWIZNETINFO.mac[2], gWIZNETINFO.mac[3], gWIZNETINFO.mac[4], gWIZNETINFO.mac[5]);
	printf("IP: %d.%d.%d.%d\r\n", gWIZNETINFO.ip[0], gWIZNETINFO.ip[1], gWIZNETINFO.ip[2], gWIZNETINFO.ip[3]);
	printf("GW: %d.%d.%d.%d\r\n", gWIZNETINFO.gw[0], gWIZNETINFO.gw[1], gWIZNETINFO.gw[2], gWIZNETINFO.gw[3]);
	printf("SN: %d.%d.%d.%d\r\n", gWIZNETINFO.sn[0], gWIZNETINFO.sn[1], gWIZNETINFO.sn[2], gWIZNETINFO.sn[3]);
	printf("DNS: %d.%d.%d.%d\r\n", gWIZNETINFO.dns[0], gWIZNETINFO.dns[1], gWIZNETINFO.dns[2], gWIZNETINFO.dns[3]);
}
void W5500_Initialze(void)
{
	intr_kind temp;
	//unsigned char W5100S_AdrSet[2][4] = {{2,2,2,2},{2,2,2,2}};
  //unsigned char W5100S_AdrSet[2][4] = {{8,0,0,0},{8,0,0,0}};
	unsigned char _AdrSet[2][8] = {{8,0,0,0,0,0,0,0},{8,0,0,0,0,0,0,0}};
  uint8_t ver= 0;
	csDisable();
	/*
	 */
	reg_wizchip_cs_cbfunc(csEnable, csDisable);
	reg_wizchip_spi_cbfunc(spiReadByte,spiWriteByte);
  reg_wizchip_spiburst_cbfunc(wizchip_read_burst, wizchip_write_burst);
  //reg_wizchip_spi_cbfunc(spiReadByte, spiWriteByte, wizchip_read_burst, wizchip_write_burst);
	temp = IK_DEST_UNREACH;

	if(ctlwizchip(CW_INIT_WIZCHIP,(void*)_AdrSet) == -1)
	{
		printf("W5500 initialized fail.\r\n");
	}

  ver = getVERSIONR();
  printf("ver = 0x%02x\r\n", ver);
  
	if(ctlwizchip(CW_SET_INTRMASK,&temp) == -1)
	{
		printf("W5500 interrupt\r\n");
	}

	do{//check phy status.
		if(ctlwizchip(CW_GET_PHYLINK,(void*)&temp) == -1){
			printf("Unknown PHY link status.\r\n");
		}
	}while(temp == PHY_LINK_OFF);

}
uint32_t get_time(void)
{
    return HAL_GetTick();
}
uint16_t CLI_Process(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint8_t u_sn = 0, t_sn = 1;
  int32_t ret = 0;
  uint8_t temp_ret1 = 0, temp_ret2 = 0;
  uint16_t port = 5001;
  uint8_t send_data[SEND_SIZE]={0,};
  uint8_t recv_data[SEND_SIZE] = {0,};
  uint16_t cnt = 0;
  unsigned char destip[4] = {192,168,15,7};
  uint16_t destport = 6000, t_d_port = 6001;
  uint8_t repeat_cnt = 3;
  uint32_t time1 = 0, time2 = 0;
  uint16_t send_size_1 = 0, recv_size = 0;
  uint16_t max_buf_size = 0, sentsize = 0, temp_send_size = 0;
  uint8_t d_MAC[6];
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
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  printf("W5500 test \r\n");
  HAL_UART_Receive_IT(&huart3, &rxData, 1);

  W5500_Initialze();
  wizchip_setnetinfo(&defaultNetInfo);
  //Display_Net_Conf();
  print_network_information();

  temp_ret1 = getSIR();
  temp_ret2 = getSIMR();
  printf("1. IR Reg %02x IMR Reg %02x\r\n", temp_ret1, temp_ret2);
  //setIMR(0x01); //socket 0 enable
  setSIMR(0x01); //socket 0 enable
  temp_ret1 = getSIR();
  temp_ret2 = getSIMR();
  printf("2. IR Reg %02x IMR Reg %02x\r\n", temp_ret1, temp_ret2);
  temp_ret1 = getSn_IR(u_sn);
  temp_ret2 = getSn_IMR(u_sn);
  printf("1. sn : %d IR Reg %02x IMR Reg %02x\r\n",u_sn, temp_ret1, temp_ret2);
  
  setSn_IMR(u_sn, 0x04); //->RECV INT,  SENDOK|TIMEOUT|RECV|DISCON|CON|
  setSn_IR(u_sn, 0x04);
  temp_ret1 = getSn_IR(u_sn);
  temp_ret2 = getSn_IMR(u_sn);
  printf("2. sn : %d IR Reg %02x IMR Reg %02x\r\n",u_sn, temp_ret1, temp_ret2);
  
  if((ret = socket(u_sn, Sn_MR_UDP, port, 0x00)) != u_sn)
    printf("socket open Error 0x%02x\r\n", ret);
  send_data[0] ='z';
  send_data[1] ='0';
  printf("socket open port : %d \r\n", port);
  printf("Dest IP : %d.%d.%d.%d Port : %d\r\n", destip[0], destip[1], destip[2], destip[3], destport);
  for(cnt = 2; cnt < SEND_SIZE; cnt ++)
  {
    send_data[cnt] = 0x41 + (cnt % 25);
  }
  printf("UDP Send Ready port %d \r\n", port);
  printf("plead input 1 send start\r\n");
  max_buf_size = getSn_TxMAX(u_sn);
  send_size_1 = max_buf_size - (max_buf_size % 1472);
  printf("1 send buf size = %d\r\n", send_size_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(rx_flag == 1)
    {
      CLI_Process();
      rx_flag = 0;
    }
    if(ETH_INT_Flag == 1)
    {
    	ETH_INT_Flag = 0;
      temp_ret1 = getSn_IR(u_sn);
      printf("1 sn: %d snIR:%02x \r\n", u_sn, temp_ret1);

      setSn_IR(u_sn, temp_ret1);
      temp_ret1 = getSn_IR(u_sn);
      printf("2 sn: %d snIR:%02x \r\n", u_sn, temp_ret1);
      getSn_DHAR(u_sn, d_MAC);
      printf("Recv1 Dest MAC = %02x:%02x:%02x:%02x:%02x:%02x \r\n", d_MAC[0], d_MAC[1], d_MAC[2], d_MAC[3], d_MAC[4], d_MAC[5]);
      if((recv_size = getSn_RX_RSR(u_sn)) > 0)
      {
        if(recv_size > SEND_SIZE) recv_size = SEND_SIZE;
        ret = recvfrom(u_sn, recv_data, recv_size, destip, (uint16_t*)&destport);
        if(ret <= 0)
        {
          printf("%d: recvfrom error. %ld\r\n",u_sn,ret);
          return ret;
        }
        recv_data[recv_size] = 0;
        printf("recv[%d]:%s\r\n",recv_size, recv_data);
        printf("recv size: %d , Dest IP : %d.%d.%d.%d Port : %d\r\n",recv_size , destip[0], destip[1], destip[2], destip[3], destport);
        getSn_DHAR(u_sn, d_MAC);
        printf("Recv2 Dest MAC = %02x:%02x:%02x:%02x:%02x:%02x \r\n", d_MAC[0], d_MAC[1], d_MAC[2], d_MAC[3], d_MAC[4], d_MAC[5]);

        recv_size = (uint16_t) ret;
        sentsize = 0;
        while(sentsize != recv_size)
        {
          ret = sendto(u_sn, recv_data+sentsize, recv_size-sentsize, destip, destport);
          if(ret < 0)
          {
            printf("%d: sendto error. %ld\r\n",u_sn,ret);
            return ret;
          }
          sentsize += ret; // Don't care SOCKERR_BUSY, because it is zero.
        }
        getSn_DHAR(u_sn, d_MAC);
        printf("send Dest MAC = %02x:%02x:%02x:%02x:%02x:%02x\r\n", d_MAC[0], d_MAC[1], d_MAC[2], d_MAC[3], d_MAC[4], d_MAC[5]);

      }
    }
    if(send_flag == 1)
    {
      time1 = get_time();
      printf("UDP Send start \r\n");
      for(cnt = 0; cnt < repeat_cnt; cnt++)
      {
        send_data[1] ='0'+ cnt;
        #if 0
        ret = sendto(u_sn, send_data, SEND_SIZE, destip, destport);
        if(ret < 0)
        {
          printf("[%d] : sendto error. %ld\r\n",u_sn ,ret);
        }
        #endif
        sentsize = 0;
        while(sentsize != SEND_SIZE)
        {
            temp_send_size = (SEND_SIZE-sentsize) > send_size_1 ? send_size_1 : (SEND_SIZE-sentsize);
            ret = sendto(u_sn, send_data+sentsize, temp_send_size, destip, destport);
            if(ret < 0)
            {
              printf("%d: sendto error. %ld\r\n",u_sn,ret);
            }
            else
              sentsize += ret; // Don't care SOCKERR_BUSY, because it is zero.
        }
      }
      send_flag = 0;
      time2 = get_time();
      printf("sned finished count : %d time : %ld one cnt time : %ld\r\n", repeat_cnt, (uint16_t)(time2 - time1), (uint16_t)((time2-time1)/repeat_cnt));
    }
	  //loopback_tcps(0,ethBuf0,5000);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : W5500_INTn_Pin */
  GPIO_InitStruct.Pin = W5500_INTn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(W5500_INTn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint16_t CLI_Process(void)
{
  uint16_t mode = 0;
  printf("serial recv : [%d] %s\r\n", rx_index, rx_buffer);
  if (strncmp((const char *)rx_buffer, (const char *)"help", 4) == 0) // spi setting
  {
	  printf("help\r\n");
  }
  else if (strncmp((const char *)rx_buffer, (const char *)"stop", 4) == 0) // spi setting
  {
	  printf("stop\r\n");
	  close(0);
    mode = 4;
  }
  else if (strncmp((const char *)rx_buffer, (const char *)"1", 1) == 0) // spi setting
  {
	  printf("send udp flag\r\n");
    mode = 1;
    send_flag = 1;
  }
  rx_index = 0;
  HAL_UART_Transmit(&huart3, (uint8_t *)">", 1, 0xFFFF);
  return mode;
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
