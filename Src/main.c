
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
  * COPYRIGHT(c) 2020 STMicroelectronics
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
#include "stm32l4xx_hal.h"

/* USER CODE BEGIN Includes */

#include "I2C_lcd16x2.h"
#include "ESP8266_usart_commands.h"
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init();
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint32_t funcao_anos(uint32_t *delta, RTC_TimeTypeDef *horario1, RTC_DateTypeDef *data1);
uint32_t funcao_anos_aux1(RTC_TimeTypeDef *horario, RTC_DateTypeDef *data);
uint32_t funcao_segundos_desde_1jan_agora(RTC_TimeTypeDef *horario, RTC_DateTypeDef *data);
uint32_t funcao_segundos_desde_1jan_fixo(RTC_DateTypeDef *data);
uint32_t funcao_meses(uint32_t *delta, RTC_TimeTypeDef *horario1, RTC_DateTypeDef *data1);
uint32_t funcao_meses_aux(RTC_TimeTypeDef *horario, RTC_DateTypeDef *data);
uint32_t funcao_dias(uint32_t *delta, RTC_TimeTypeDef *horario1, RTC_DateTypeDef *data1);
uint32_t funcao_horas(uint32_t *delta, RTC_TimeTypeDef *horario1);
uint32_t funcao_minutos(uint32_t *delta, RTC_TimeTypeDef *horario1);
uint32_t funcao_segundos(uint32_t *delta, RTC_TimeTypeDef *horario1);


uint32_t diferenca_de_seculos(void);
uint32_t segundos_interno(RTC_TimeTypeDef *horario, RTC_DateTypeDef *data);
void luna_sync(uint32_t *segundos_brasil);
void clock_sync(uint8_t *resposta);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

uint8_t  interrupcao=0, estados=0, flag0=0, flag1=0, flag2=0, flag3=0,
		flag4=0, flag5=0, flag6=0, flag7=0;

uint8_t resposta0[AT_RESPONSE_SIZE];
uint8_t resposta1[ATE_RESPONSE_SIZE];
uint8_t resposta2[ATCWMODE_CUR_RESPONSE_SIZE];
uint8_t resposta3[ATCWJAP_CUR_RESPONSE_SIZE];
uint8_t resposta31[ATCWJAP1_CUR_RESPONSE_SIZE];
uint8_t resposta4[ATCIPSTART_RESPONSE_SIZE];
uint8_t resposta5[ATCIPSEND_RESPONSE_SIZE];
uint8_t resposta6[NTPREQUEST_RESPONSE_SIZE];
uint8_t respostaclose[ATCIPCLOSE_RESPONSE_SIZE];

uint8_t horas, segundos, horario[8], data[8], semana[3], dia_da_semana, anos, meses, dia, fase_lua;
static uint8_t minutos;

uint32_t segundos_brasil, w=0, seculo=1, seculo2;
static uint32_t anos4digitos;

RTC_TimeTypeDef ghorario;
RTC_DateTypeDef gdata;


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t flag_final1=0;
	uint32_t segundos_interno_aux;

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
  MX_I2C1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  LCD1602_i2c_Init();
  LCD1602_special_caracter_Init("ç", "ú", "á", "é", "í", "ã","õ","ô");
  LCD1602_print("Início");

  HAL_UART_MspInit(&huart2);

  HAL_UART_Receive_DMA(&huart2, resposta0, AT_RESPONSE_SIZE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* O objetivo do código é sincronizar o relógio interno do microcontrolador (RTC-Real Time Clock) com o relogio da internet usando o protocolo
   * NTP-Network Time Protocol. Para isso utilizamos o microcontrolador stm32l432kc um modulo wi-fi esp8266 e uma tela de lcd16x2.
   * O codigo envia comandos AT via USART, para o esp8266, e aguarda uma resposta, se a resposta for a resposta esperada envia-se o próximo
   * comando. Dado um ultimo comando teremos uma ultima resposta dessa se extrai informações para o relógio.
   * Nunca deixe o relógio sem sincronizar por mais de um século pois as fases da lua estarão erradas*/

  while (1) // esse while representa um loop infinito
  {
	  if(interrupcao==0) //Quando não acontece uma interrupção ou se retorna de uma interrupção essa condição é acionada, entrar neste if representa o fluxo normal do código
	  {
		  if(estados==0) //Aqui se tem varias instruções if-else-if para enviar as instruçoes AT em ordem. A variavel estados é incrementada na interrupção.
		  {
			  LCD1602_clear();
			  LCD1602_print("Enviando 0");
			  if(flag0==0)
			  {
				  HAL_UART_Transmit(&huart2, AT, AT_COMMAND_SIZE, HAL_MAX_DELAY);
			  }
			  flag0=1;
			  HAL_Delay(1000);
			  if(strcmp(resposta0, ERRO_0)==0)/*Este if trata de um erro relacionado ao fato de que o comando ATE0 ja foi dado*/
			  {
				  HAL_UART_DMAStop(&huart2);

				  estados=2;//Pulamos para o estado 2
				  flag0=2;//Essa flag avisa que ocorreu um erro e prepara para a corrente de erros que esta por vir
				  HAL_Delay(1000);
				  interrupcao=0;
				  HAL_UART_Receive_DMA(&huart2, resposta2, ATCWMODE_CUR_RESPONSE_SIZE);
			  }
		  }
		  else if(estados==1)
		  {
			  LCD1602_clear();
			  LCD1602_print("Enviando 1");
			  if(flag1==0)
			  {
				  HAL_UART_Transmit(&huart2, ATE, ATE_COMMAND_SIZE, HAL_MAX_DELAY);
			  }
			  flag1=1;
			  HAL_Delay(1000);
		  }
		  else if(estados==2)
		  {
			  LCD1602_clear();
			  LCD1602_print("Enviando 2");
			  if(flag2==0)
			  {
				  HAL_UART_Transmit(&huart2, ATCWMODE_CUR, ATCWMODE_CUR_COMMAND_SIZE, HAL_MAX_DELAY);
			  }
			  flag2=1;
			  HAL_Delay(1000);
		  }
		  else if(estados==3)
		  {
			  LCD1602_clear();
			  LCD1602_print("Enviando 3");
			  if(flag3==0)
			  {
				  HAL_UART_Transmit(&huart2, ATCWJAP_CUR, ATCWJAP_CUR_COMMAND_SIZE, HAL_MAX_DELAY);
			  }
			  flag3=1;
			  HAL_Delay(1000);
		  }
		  else if(estados==4)
		  {
			  LCD1602_clear();
			  LCD1602_print("Enviando 4");
			  if(flag4==0)
			  {
				  HAL_UART_Transmit(&huart2, ATCIPSTART, ATCIPSTART_COMMAND_SIZE, HAL_MAX_DELAY);
			  }
			  flag4=1;
			  HAL_Delay(1000);
		  }
		  else if(estados==5)
		  {
			  LCD1602_clear();
			  LCD1602_print("Enviando 5");
			  if(flag5==0)
			  {
				  HAL_UART_Transmit(&huart2, ATCIPSEND, ATCIPSEND_COMMAND_SIZE, HAL_MAX_DELAY);
			  }
			  flag5=1;
			  HAL_Delay(1000);
		  }
		  else if(estados==6)
		  {
			  LCD1602_clear();
			  LCD1602_print("Enviando 6");
			  if(flag6==0)
			  {
				  HAL_UART_Transmit(&huart2, NTPREQUEST, NTPREQUEST_COMMAND_SIZE, HAL_MAX_DELAY);
			  }
			  flag6=1;
			  HAL_Delay(1000);
			  if(strcmp(resposta6,ERRO_6)==0) /*Dentro deste if tratamos um erro. O erro acontece quando o servidor
			  	  	  	  	  	  	  	  	   	não responde ao comando NTPREQUEST. Ao receber a resposta incompleta
			  	  	  	  	  	  	  	  	   	não é gerada uma interrupção*/
			  {
				  HAL_UART_DMAStop(&huart2); //Pausamos o DMA sem gerar uma interrupção

				  HAL_UART_Receive_DMA(&huart2, respostaclose, ATCIPCLOSE_RESPONSE_SIZE);//Nos preparamos para a resposta do encerramento da conexão

				  for(uint i=0;i<NTPREQUEST_RESPONSE_SIZE;i++)//Limpamos a resposta incompleta recebida
				  {
					  resposta6[i]=0;
				  }
				  HAL_UART_Transmit(&huart2, ATCIPCLOSE, ATCIPCLOSE_COMMAND_SIZE, HAL_MAX_DELAY); //Encerramos a conexão com o servidor
				  estados=4;//voltamos ao estado 4
				  flag4=0;
				  flag5=0;
				  flag6=0;
				  HAL_Delay(1000);
				  interrupcao=0;
				  HAL_UART_Receive_DMA(&huart2, resposta4, ATCIPSTART_RESPONSE_SIZE);
			  }
		  }
		  else if(estados==7)
		  {
			  LCD1602_clear();
			  LCD1602_print("Concluído");

			  if(flag7==0)
			  {
				  HAL_UART_Transmit(&huart2, ATCIPCLOSE, ATCIPCLOSE_COMMAND_SIZE, HAL_MAX_DELAY);
			  }
			  flag7=1;
			  HAL_Delay(1000);
			  interrupcao=2; //Esta condição passa para a proxima parte do programa e encerra a conexao e recebimento de dados
		  }
		  else //Se o programa tentar executar um estado invalido este erro será tratado aqui
		  {
		  	  LCD1602_clear();
		  	  LCD1602_print("Erro de estado");
		  	  HAL_Delay(1000);
		  }

	  }
	  else if (interrupcao==1) //Quando uma interrupção acontece, ou seja os dados estão disponíveis, a placa esp8266 respondeu, esta condição é executada para tratar os dados recebidos
	  {
		  if(estados==0) //Temos varios if-else-if para verificar qual comnando foi dado
		  {
			  if(strcmp(AT_RESPONSE, resposta0)==0) //essa condição verifica se a resposta dada está correta
			  {
				  estados++; // Se a ressposta está correta passa-se para o proximo comando
			  }
			  else //Se a resposta dada não estiver correta o erro será tratado aqui
			  {
				  LCD1602_clear();
				  LCD1602_print("Resposta não");
				  LCD1602_setCursor(2,1);
				  LCD1602_print("esperada 0");
				  while(1)
				  {

				  }
			  }
			  LCD1602_clear();
			  LCD1602_print("Recebimento");
			  LCD1602_setCursor(2,1);
			  LCD1602_print("completo 0");
			  HAL_Delay(1000);
			  interrupcao=0; //Retorna-se da interrupção
			  HAL_UART_Receive_DMA(&huart2, resposta1, ATE_RESPONSE_SIZE); //Prepara-se a usart para a recepção do próximo comando
		  }
		  else if(estados==1)
		  {
			  if(strcmp(ATE_RESPONSE, resposta1)==0)
			  {
				  estados++;
			  }
			  else
			  {
				  LCD1602_clear();
				  LCD1602_print("Resposta não");
				  LCD1602_setCursor(2,1);
				  LCD1602_print("esperada 1");
				  while(1)
				  {

				  }
			  }
			  LCD1602_clear();
			  LCD1602_print("Recebimento");
			  LCD1602_setCursor(2,1);
			  LCD1602_print("completo 1");
			  HAL_Delay(1000);
			  interrupcao=0;
			  HAL_UART_Receive_DMA(&huart2, resposta2, ATCWMODE_CUR_RESPONSE_SIZE);
		  }
		  else if(estados==2)
		  {
			  if(strcmp(ATCWMODE_CUR_RESPONSE, resposta2)==0)
			  {
				  estados++;
			  }
			  else
			  {
				  LCD1602_clear();
				  LCD1602_print("Resposta não");
				  LCD1602_setCursor(2,1);
				  LCD1602_print("esperada 2");
				  while(1)
				  {

				  }
			  }
			  LCD1602_clear();
			  LCD1602_print("Recebimento");
			  LCD1602_setCursor(2,1);
			  LCD1602_print("completo 2");
			  HAL_Delay(1000);
			  interrupcao=0;
			  if(flag0==2)
			  {
				  HAL_UART_Receive_DMA(&huart2, resposta31, ATCWJAP1_CUR_RESPONSE_SIZE);
			  }
			  else
			  {
				  HAL_UART_Receive_DMA(&huart2, resposta3, ATCWJAP_CUR_RESPONSE_SIZE);
			  }
		  }
		  else if(estados==3)
		  {
			  if(strcmp(ATCWJAP_CUR_RESPONSE, resposta3)==0 || strncmp(ATCWJAP1_CUR_RESPONSE, resposta31, ATCWJAP1_CUR_RESPONSE_SIZE)==0) //Erro strcmp(ATCWJAP1_CUR_RESPONSE, resposta31)!=0 deveria ser strcmp(ATCWJAP1_CUR_RESPONSE, resposta31)==0 substitui-se pela instrução strncmp(ATCWJAP1_CUR_RESPONSE, resposta31, ATCWJAP1_CUR_RESPONSE_SIZE)==0 pois a outra instrução nao localiza o caractere NULL \0
			  {
				  estados++;
			  }
			  else
			  {
				  LCD1602_clear();
				  LCD1602_print("Resposta não");
				  LCD1602_setCursor(2,1);
				  LCD1602_print("esperada 3");
				  while(1)
				  {

				  }
			  }
			  LCD1602_clear();
			  LCD1602_print("Recebimento");
			  LCD1602_setCursor(2,1);
			  LCD1602_print("completo 3");
			  HAL_Delay(1000);
			  interrupcao=0;
			  HAL_UART_Receive_DMA(&huart2, resposta4, ATCIPSTART_RESPONSE_SIZE);
		  }
		  else if(estados==4)
		  {
			  if(strcmp(ATCIPSTART_RESPONSE, resposta4)==0)
			  {
				  estados++;
			  }
			  else
			  {
				  LCD1602_clear();
				  LCD1602_print("Resposta não");
				  LCD1602_setCursor(2,1);
				  LCD1602_print("esperada 4");
				  while(1)
				  {

				  }
			  }
			  LCD1602_clear();
			  LCD1602_print("Recebimento");
			  LCD1602_setCursor(2,1);
			  LCD1602_print("completo 4");
			  HAL_Delay(1000);
			  interrupcao=0;
			  HAL_UART_Receive_DMA(&huart2, resposta5, ATCIPSEND_RESPONSE_SIZE);
		  }
		  else if(estados==5)
		  {
			  if(strncmp(ATCIPSEND_RESPONSE, resposta5,ATCIPSEND_RESPONSE_SIZE)==0) //Erro strcmp(ATCIPSEND_RESPONSE, resposta5)!=0 deveria ser strcmp(ATCIPSEND_RESPONSE, resposta5)==0 Substitui-se pela instrução strncmp(ATCIPSEND_RESPONSE, resposta5,ATCIPSEND_RESPONSE_SIZE) pois a outra instrução não localiza ocaractere NULL \0 no final das strings
			  {
				  estados++;
			  }
			  else
			  {
				  LCD1602_clear();
				  LCD1602_print("Resposta não");
				  LCD1602_setCursor(2,1);
				  LCD1602_print("esperada 5");
				  while(1)
				  {

				  }
			  }
			  LCD1602_clear();
			  LCD1602_print("Recebimento");
			  LCD1602_setCursor(2,1);
			  LCD1602_print("completo 5");
			  HAL_Delay(1000);
			  interrupcao=0;
			  HAL_UART_Receive_DMA(&huart2, resposta6, NTPREQUEST_RESPONSE_SIZE);
		  }
		  else if(estados==6)//O ultimo dado recebido contém as informações para o relógio e aquele será tratado aqui
		  {
			  clock_sync(resposta6);
			  interrupcao=0;
			  estados++;
			  HAL_UART_Receive_DMA(&huart2, respostaclose, ATCIPCLOSE_RESPONSE_SIZE);
		  }
	  }
	  else if(interrupcao==2)/*Essa secção do programa encerra o programa*/
	  {
		  while(w<=anos4digitos)
		  {
			  w=w+100;
			  seculo++;
		  }
		  seculo=seculo-1;
		  seculo2=seculo;
		  MX_RTC_Init();
		  interrupcao=3;
	  }
	  else if(interrupcao==3)
	  {
		  HAL_RTC_GetTime(&hrtc, &ghorario, RTC_FORMAT_BIN);
		  HAL_RTC_GetDate(&hrtc, &gdata, RTC_FORMAT_BIN);/* Os comandos HAL_RTC_GetTime e HAL_RTC_GetDate tem que serem dados nessa sequencia mesmo que só desejarmos as horas*/

		  sprintf(horario,"%02d:%02d:%02d",ghorario.Hours, ghorario.Minutes, ghorario.Seconds);
		  sprintf(data,"%02d/%02d/%02d", gdata.Date, gdata.Month, gdata.Year);
		  if(gdata.WeekDay==RTC_WEEKDAY_MONDAY)
		  {
			  sprintf(semana, "Seg");
		  }
		  else if(gdata.WeekDay==RTC_WEEKDAY_TUESDAY)
		  {
			  sprintf(semana, "Ter");
		  }
		  else if(gdata.WeekDay==RTC_WEEKDAY_WEDNESDAY)
		  {
			  sprintf(semana, "Qua");
		  }
		  else if(gdata.WeekDay==RTC_WEEKDAY_THURSDAY)
		  {
			  sprintf(semana, "Qui");
		  }
		  else if(gdata.WeekDay==RTC_WEEKDAY_FRIDAY)
		  {
			  sprintf(semana, "Sex");
		  }
		  else if(gdata.WeekDay==RTC_WEEKDAY_SATURDAY)
		  {
			  sprintf(semana, "Sáb");
		  }
		  else if(gdata.WeekDay==RTC_WEEKDAY_SUNDAY)
		  {
			  sprintf(semana, "Dom");
		  }
		  else
		  {
			  sprintf(semana, "Err");
		  }

		  /*Para sincronizar as fases da lua com o RTC*/
		  if(ghorario.Seconds>=0 && ghorario.Seconds<1)
		  {
			  seculo2=seculo+diferenca_de_seculos();
			  segundos_interno_aux=segundos_interno(&ghorario, &gdata);
			  luna_sync(&segundos_interno_aux);
		  }


		  if(flag_final1==0)
		  {
			  LCD1602_clear();
			  LCD1602_special_caracter_Init("L1", "L2", "L3", "L4", "á", "é", "í", "ó");
			  LCD1602_noCursor();
			  flag_final1=1;
		  }
		  LCD1602_print(horario);
		  LCD1602_setCursor(2,1);
		  LCD1602_print(data);
		  LCD1602_setCursor(2,10);
		  LCD1602_print(semana);
		  if(fase_lua==1)
		  {
			  LCD1602_setCursor(2,15);
			  LCD1602_print("L1");
			  LCD1602_setCursor(2,16);
			  LCD1602_print("L2");
		  }
		  else if(fase_lua==2)
		  {
			  LCD1602_setCursor(2,15);
			  LCD1602_print("L4");
			  LCD1602_setCursor(2,16);
			  LCD1602_print("L2");
		  }
		  else if(fase_lua==3)
		  {
			  LCD1602_setCursor(2,15);
			  LCD1602_print("L4");
			  LCD1602_setCursor(2,16);
			  LCD1602_print("L3");
		  }
		  else if(fase_lua==4)
		  {
			  LCD1602_setCursor(2,15);
			  LCD1602_print("L1");
			  LCD1602_setCursor(2,16);
			  LCD1602_print("L3");
		  }

		  LCD1602_setCursor(1,1); /*Ao invés de dar clear e escrever novamente, o que faria a tela piscar, apenas escrevemos por cima, por isso voltamos o cursor para a posição 1,1*/
	  }
	  else
	  {
		  LCD1602_clear();
		  LCD1602_print("Erro inesperado");
		  HAL_Delay(1000);
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20D22930;//0x00707CBB; //0x00000E14;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* RTC init function */
static void MX_RTC_Init()
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

    /**Initialize RTC Only 
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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initialize RTC and set the Time and Date 
    */
  sTime.Hours = horas;
  sTime.Minutes = minutos;
  sTime.Seconds = segundos;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sDate.WeekDay = dia_da_semana;
  sDate.Month = meses;
  sDate.Date = dia;
  sDate.Year = anos;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
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

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 PA5 
                           PA6 PA7 PA8 PA11 
                           PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB3 PB4 
                           PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PH3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

uint32_t diferenca_de_seculos(void)
{
	uint32_t retorno;

	if(gdata.Year<anos && gdata.Year>=1)
		retorno=1;
	else
		retorno=0;

	return retorno;
}

uint32_t segundos_interno(RTC_TimeTypeDef *horario, RTC_DateTypeDef *data)
{
	uint32_t delta=0;

	delta=funcao_anos(&delta, horario, data);

	return segundos_brasil+delta;
}

uint32_t funcao_anos(uint32_t *delta, RTC_TimeTypeDef *horario1, RTC_DateTypeDef *data1)
{
	if(data1->Year*100*(seculo2-1)-anos4digitos==0)
	{
		*delta=funcao_meses(delta, horario1, data1);
	}
	else
	{
		*delta=funcao_anos_aux1(horario1 ,data1)+funcao_meses(delta, horario1, data1);
	}

	return *delta;
}


uint32_t funcao_anos_aux1(RTC_TimeTypeDef *horario, RTC_DateTypeDef *data)
{
	uint32_t segundos1=0;

	if(data->Month>=1 && funcao_segundos_desde_1jan_agora(horario, data)<funcao_segundos_desde_1jan_fixo(data)/*data->Month<meses*/)
	{
		for(uint32_t i=anos4digitos;i<data->Year+100*(seculo2-1)-1;i++)
		{
			if( ( ( i % 4 == 0 && i % 100 != 0 ) || i % 400 == 0 ))
				segundos1=segundos1+31622400;
			else
				segundos1=segundos1+31536000;
		}
	}
	else
	{
		for(uint32_t i=anos4digitos;i<data->Year+100*(seculo2-1);i++)
		{
			if( ( ( i % 4 == 0 && i % 100 != 0 ) || i % 400 == 0 ))
				segundos1=segundos1+31622400;
			else
				segundos1=segundos1+31536000;
		}
	}

	return segundos1;
}

uint32_t funcao_segundos_desde_1jan_agora(RTC_TimeTypeDef *horario, RTC_DateTypeDef *data)
{
	uint32_t segundos1=0;

	for(uint8_t i=1;i<data->Month;i++)
	{
		if(i==2)
		{
			if( ( ( (data->Year*100*(seculo2-1)) % 4 == 0 && (data->Year*100*(seculo2-1)) % 100 != 0 ) || (data->Year*100*(seculo2-1)) % 400 == 0 ))
				segundos1=segundos1+2505600;
			else
				segundos1=segundos1+2419200;
		}
		else
		{
			if(((i%2==0 && i<=6)||(i%2!=0 && i>=9)))
				segundos1=segundos1+2592000;
			else if(((i%2!=0 && i<=7)||(i%2==0 && i>=8)))
				segundos1=segundos1+2678400;
		}
	}
	segundos1=segundos1+(data->Date-1)*86400+horario->Hours*3600+horario->Minutes*60+horario->Seconds;

	return segundos1;
}

uint32_t funcao_segundos_desde_1jan_fixo(RTC_DateTypeDef *data)
{
	uint32_t segundos1=0;

	for(uint8_t i=1;i<meses;i++)
	{
		if(i==2)
		{
			if( ( (( data->Year*100*(seculo2-1)) % 4 == 0 && (data->Year*100*(seculo2-1)) % 100 != 0 ) || (data->Year*100*(seculo2-1)) % 400 == 0 ))
				segundos1=segundos1+2505600;
			else
				segundos1=segundos1+2419200;
		}
		else
		{
			if(((i%2==0 && i<=6)||(i%2!=0 && i>=9)))
				segundos1=segundos1+2592000;
			else if(((i%2!=0 && i<=7)||(i%2==0 && i>=8)))
				segundos1=segundos1+2678400;
		}
	}

	segundos1=segundos1+(dia-1)*86400+horas*3600+minutos*60+segundos;
	return segundos1;
}

uint32_t funcao_meses(uint32_t *delta, RTC_TimeTypeDef *horario1, RTC_DateTypeDef *data1)
{
	if(data1->Month-meses==0)
		*delta=funcao_dias(delta, horario1, data1);
	else
		*delta=funcao_meses_aux(horario1, data1)+funcao_dias(delta, horario1, data1);

	return *delta;
}

uint32_t funcao_meses_aux(RTC_TimeTypeDef *horario, RTC_DateTypeDef *data)
{
	uint32_t segundos1=0;

	if(data->Date>=1 && (data->Date-1)*86400+horario->Hours*3600+horario->Minutes*60+horario->Seconds<dia*86400+horas*3600+minutos*60+segundos)
	{
		for(uint8_t i=meses;i!=data->Month-1;i++)
		{
			if(i==13)
				i=1;
			if(i==2)
			{
				if( ( ( anos4digitos % 4 == 0 && anos4digitos % 100 != 0 ) || anos4digitos % 400 == 0 ))
					segundos1=segundos1+2505600;
				else
					segundos1=segundos1+2419200;
			}
			else
			{
				if(((i%2==0 && i<=6)||(i%2!=0 && i>=9)))
					segundos1=segundos1+2592000;
				else if(((i%2!=0 && i<=7)||(i%2==0 && i>=8)))
					segundos1=segundos1+2678400;
			}
		}
	}
	else
	{
		for(uint8_t i=meses;i!=data->Month;i++)
		{
			if(i==13)
				i=1;
			if(i==2)
			{
				if( ( ( anos4digitos % 4 == 0 && anos4digitos % 100 != 0 ) || anos4digitos % 400 == 0 ))
					segundos1=segundos1+2505600;
				else
					segundos1=segundos1+2419200;
			}
			else
			{
				if(((i%2==0 && i<=6)||(i%2!=0 && i>=9)))
					segundos1=segundos1+2592000;
				else if(((i%2!=0 && i<=7)||(i%2==0 && i>=8)))
					segundos1=segundos1+2678400;
			}
		}
	}

	return segundos1;
}

uint32_t funcao_dias(uint32_t *delta, RTC_TimeTypeDef *horario1, RTC_DateTypeDef *data1)
{
	if(data1->Date-dia==0)
	{
		*delta=funcao_horas(delta, horario1);
	}
	else if(data1->Date-dia>0)
	{
		*delta=86400*(data1->Date-dia-1)+funcao_horas(delta, horario1);
	}
	else if(data1->Date-dia<0)
	{
		if(meses==2)
		{
			if( ( ( anos4digitos % 4 == 0 && anos4digitos % 100 != 0 ) || anos4digitos % 400 == 0 ))
				*delta=86400*(data1->Date+29-dia-1)+funcao_horas(delta, horario1);
			else
				*delta=86400*(data1->Date+28-dia-1)+funcao_horas(delta, horario1);
		}
		else
		{
			if(((meses%2==0 && meses<=6)||(meses%2!=0 && meses>=9)))
				*delta=86400*(data1->Date+30-dia-1)+funcao_horas(delta, horario1);
			else if(((meses%2!=0 && meses<=7)||(meses%2==0 && meses>=8)))
				*delta=86400*(data1->Date+31-dia-1)+funcao_horas(delta, horario1);
		}
	}

	return *delta;
}

uint32_t funcao_horas(uint32_t *delta, RTC_TimeTypeDef *horario1)
{
	if(horario1->Hours-horas==0)
	{
		*delta=funcao_minutos(delta, horario1);
	}
	else if(horario1->Hours-horas>0)
	{
		*delta=3600*(horario1->Hours-horas-1)+funcao_minutos(delta, horario1);
	}
	else if(horario1->Hours-horas<0)
	{
		*delta=3600*((horario1->Hours+24)-horas-1)+funcao_minutos(delta, horario1);
	}

	return *delta;
}

uint32_t funcao_minutos(uint32_t *delta, RTC_TimeTypeDef *horario1)
{
	if(horario1->Minutes-minutos==0)
	{
		*delta=funcao_segundos(delta, horario1);
	}
	else if(horario1->Minutes-minutos>0)
	{
		*delta=60*(horario1->Minutes-minutos-1)+funcao_segundos(delta, horario1);
	}
	else if(horario1->Minutes-minutos<0)
	{
		*delta=60*((horario1->Minutes+60)-minutos-1)+funcao_segundos(delta, horario1);
	}

	return *delta;
}

uint32_t funcao_segundos(uint32_t *delta, RTC_TimeTypeDef *horario1)
{
	if(horario1->Seconds-segundos==0)
	{
		*delta=0;
	}
	else if(horario1->Seconds-segundos>0)
	{
		*delta=horario1->Seconds-segundos;
	}
	else if(horario1->Seconds-segundos<0)
	{
		*delta=(horario1->Seconds+60)-segundos;
	}

	return *delta;
}

void luna_sync(uint32_t *segundos_brasil)
{
	/*O cálculo das fases da lua está baseado no trabalho "As Variações dos Intervalos de Tempo entre as Fases Principais da Lua" de Fernando Lang da Silveira.
	 * Em 1970 o ano lunar é 582 (lunação) e o ano lunar começa em 7/1/1970 às 18:00 com a lua nova*/

	double tnova, tquarto_crescente, tcheia, tquarto_minguante, ttotal, segundos_da_lua=669785.76, pi=3.141592654, tnovaproxima;
	uint32_t ano_lunar=582;

	while(segundos_da_lua<=*segundos_brasil)
	{
		segundos_da_lua=segundos_da_lua+(0.0866*cos((2*pi/12.3689)*(ano_lunar-953)+0.298)+0.1817*cos((2*pi/13.9444)*(ano_lunar-953)+0.602)+29.5306)*24*60*60;
		ano_lunar++;
	}
	if(segundos_da_lua==*segundos_brasil)
	{

	}
	else
		ano_lunar=ano_lunar-1;
	tnova=10957*24*60*60+(0.1723*cos((2*pi/12.3689)*(ano_lunar-953)-1.526)+0.4067*cos((2*pi/13.94436)*(ano_lunar-953)+5.089)+29.5305888*(ano_lunar-953)+5.4716)*24*60*60;
	tquarto_crescente=10957*24*60*60+(0.1723*cos((2*pi/12.3689)*(ano_lunar-953)-1.401)+0.6280*cos(2*pi/13.94436*(ano_lunar-953)+0.488)+29.5305888*(ano_lunar-953)+12.8571)*24*60*60;
	tcheia=10957*24*60*60+(0.1723*cos((2*pi/12.3689)*(ano_lunar-953)-1.272)+0.4067*cos((2*pi/13.94436)*(ano_lunar-953)+2.172)+29.5305888*(ano_lunar-953)+20.2366)*24*60*60;
	tquarto_minguante=10957*24*60*60+(0.1723*cos((2*pi/12.3689)*(ano_lunar-953)-1.143)+0.6280*cos((2*pi/13.94436)*(ano_lunar-953)+3.856)+29.5305888*(ano_lunar-953)+27.6163)*24*60*60;
	//ttotal=(0.0866*cos((2*pi/12.3689)*(ano_lunar-953)+0.298)+0.1817*cos((2*pi/13.9444)*(ano_lunar-953)+0.602)+29.5306)*24*60*60;
	tnovaproxima=10957*24*60*60+(0.1723*cos((2*pi/12.3689)*(ano_lunar+1-953)-1.526)+0.4067*cos((2*pi/13.94436)*(ano_lunar+1-953)+5.089)+29.5305888*(ano_lunar+1-953)+5.4716)*24*60*60;


	if(*segundos_brasil>=tnova && *segundos_brasil<tquarto_crescente)
		fase_lua=1;
	else if(*segundos_brasil>=tquarto_crescente && *segundos_brasil<tcheia)
		fase_lua=2;
	else if(*segundos_brasil>=tcheia && *segundos_brasil<tquarto_minguante)
		fase_lua=3;
	else if(*segundos_brasil>=tquarto_minguante && *segundos_brasil<tnovaproxima)
		fase_lua=4;
}

/*Esta função trata os dados da ultima resposta recebida convertendo-as para o formato de horas-minutos-segundos*/
void clock_sync(uint8_t *resposta)
{
	double x, y, z, k;
	uint32_t segundos_total=0, segundos_unix, xa, ya, za, ka, ano_count=1970, segundos_dos_anos=0,
			segundos_dos_meses=0, meses_count=1, segundos_dos_dias=0, dia_count=1, ano_count_aux=70;

	/*Extrai-se a quantidade de segundos desde 1/1/1900 às 00:00:00 h
	 * Converte-se para a quantidade de segundos desde 1/1/1970 às 00:00:00 h
	 * Atrasa-se 3 horas para o fuso horario do Brasil*/
	segundos_total=(segundos_total | resposta[78])<<8;
	segundos_total=(segundos_total | resposta[79])<<8;
	segundos_total=(segundos_total | resposta[80])<<8;
	segundos_total=segundos_total | resposta[81];

	segundos_unix=segundos_total-seventyYears;
	segundos_brasil=segundos_unix-3600*3;

	x=(double)segundos_brasil/86400;//O número 86400 representa a quantidade de segundos em um dia
	xa=segundos_brasil/86400;//Divisão inteira retorna um inteiro
	horas=24*(x-xa);//a variavel horas representa a hora local no Brasil sem horario de verão

	y=(double)segundos_brasil/3600;//O número 3600 representa a quantidade de segundos em uma hora
	ya=segundos_brasil/3600;//Divisão inteira retorna um inteiro
	minutos=60*(y-ya);//a variavel minutos representa os minutos

	z=(double)segundos_brasil/60;//O número 60 representa a quantidade de segundos em um minuto
	za=segundos_brasil/60;//Divisão inteira retorna um inteiro
	segundos=60*(z-za);//a variavel segundos representa os segundos

	k=(double)(segundos_brasil+259200)/604800;//O número 604800 representa a quantidade de segundos em uma semana. Desloca-se 259200 s para que a semana comece em uma segunda-feira
	ka=(segundos_brasil+259200)/604800;
	dia_da_semana=7*(k-ka)+1;//a variavel dia_da_semana representa os dias da semana


	/*Verifica em qual dia, mês e ano estamos contando todos os dias desde 1970*/
	while(segundos_dos_dias<=segundos_brasil)
	{
		segundos_dos_dias=segundos_dos_dias+86400;

		dia_count++;
		/*if(Verifique se a contagem esta em 32, se estiver, "zere" o contador || Verifique se a contagem está em 31, se estiver, verifique se o mês possui 30, se tiver, "zere" o contador || Verifique se o contador está em 29 e se é fevereiro se não for ano bissexto "zere" o contador || verifique se é dia 30 de fevereiro se for então "zere" o contador*/
		if(dia_count==32 || (dia_count==31 && ((meses_count%2==0 && meses_count<=6)||(meses_count%2!=0 && meses_count>=9))) || (dia_count==29 && meses_count==2 && !( ( ano_count % 4 == 0 && ano_count % 100 != 0 ) || ano_count % 400 == 0 )) || (dia_count==30 && meses_count==2 ) )
		{
			meses_count++;
			dia_count=1;
			if(meses_count==13) //Virada do ano
			{
				ano_count++;
				ano_count_aux++;
				if(ano_count_aux==100)
					ano_count_aux=0;
				meses_count=1;
			}
		}
	}
	if(segundos_dos_dias==segundos_brasil) //sempre conta-se um dia a mais dentro do loop anterior
		dia=dia_count;
	else
	{
		if(dia_count==1)//Resolve o bug do dia 0 e do mês 0 (esse bug acontece por que os dias e meses são ciclicos e no começo de um novo não se consegue retroceder para o final do ciclo anterior resultando em um zero)
		{
			meses_count=meses_count-1;
			if(meses_count==0)
			{
				meses_count=12;
				ano_count=ano_count-1;
				if(ano_count_aux==0)//Resolve o bug do ano 95 quando deveria ser 99
					ano_count_aux=99;
				else
					ano_count_aux=ano_count_aux-1;
			}
			meses=meses_count;
			if(meses_count==2)
			{
				if( ( ( ano_count % 4 == 0 && ano_count % 100 != 0 ) || ano_count % 400 == 0 ))
					dia=29;
				else
					dia=28;
			}
			else
			{
				if(((meses_count%2==0 && meses_count<=6)||(meses_count%2!=0 && meses_count>=9)))
					dia=30;
				else if(((meses_count%2!=0 && meses_count<=7)||(meses_count%2==0 && meses_count>=8)))
					dia=31;
			}
		}
		else
		{
			dia=dia_count-1;
			meses=meses_count;
		}
	}
	anos4digitos=ano_count;
	anos=ano_count_aux;
	luna_sync(&segundos_brasil);
}



/*Esta função é chamada toda a vez que ocorre uma interrupção na uart configurando uma flag que desviará o fluxo do programa
 * principal*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	interrupcao=1;
}

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
