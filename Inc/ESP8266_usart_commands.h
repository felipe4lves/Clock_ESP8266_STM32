/*
 * ESP8266_usart_commands.h
 *
 *  Created on: 20 de mai de 2020
 *      Author: alves
 */

#ifndef INC_ESP8266_USART_COMMANDS_H_
#define INC_ESP8266_USART_COMMANDS_H_

// List of commands
#define AT 						"AT\r\n" //Verify if the esp8266 is active and working
#define ATE 					"ATE0\r\n" //disable echo command in usart (Disabilita a opção de eco no esp8266)
#define ATCWMODE_CUR			"AT+CWMODE_CUR=1\r\n"
#define ATCWJAP_CUR				"AT+CWJAP_CUR=\"wifi_network\",\"_LazG*$3M+24X@p]\"\r\n"
#define ATCIPSTART				"AT+CIPSTART=\"UDP\",\"a.st1.ntp.br\",123\r\n"
#define ATCIPSEND				"AT+CIPSEND=48\r\n"
#define ATCIPCLOSE				"AT+CIPCLOSE\r\n"
uint8_t NTPREQUEST[]=			{0xE3, 0x00, 0x06, 0xEC, 0xE3, 0x00, 0x06, 0xEC, 0xE3, 0x00, 0x06, 0xEC, 0xE3, 0x00, 0x06, 0xEC, 0xE3, 0x00, 0x06, 0xEC, 0xE3, 0x00, 0x06, 0xEC, 0xE3, 0x00, 0x06, 0xEC, 0xE3, 0x00, 0x06, 0xEC, 0xE3, 0x00, 0x06, 0xEC, 0xE3, 0x00, 0x06, 0xEC, 0xE3, 0x00, 0x06, 0xEC, 0xE3, 0x00, 0x06, 0xEC};

//Lists of response vectors for each command (Para cada comando dado espera-se uma resposta)
uint8_t AT_RESPONSE[]= 			{0x41, 0x54, 0x0d, 0x0d, 0x0a, 0x0d, 0x0a, 0x4f, 0x4b, 0x0d, 0x0a};
uint8_t ATE_RESPONSE[]= 		{0x41, 0x54, 0x45, 0x30, 0x0d, 0x0d, 0x0a, 0x0d, 0x0a, 0x4f, 0x4b, 0x0d, 0x0a};
uint8_t ATCWMODE_CUR_RESPONSE[]={0x0d, 0x0a, 0x4f, 0x4b, 0x0d, 0x0a};
uint8_t ATCWJAP_CUR_RESPONSE[]=	{0x57, 0x49, 0x46, 0x49, 0x20, 0x43, 0x4f, 0x4e, 0x4e, 0x45, 0x43, 0x54, 0x45, 0x44, 0x0d, 0x0a, 0x57, 0x49, 0x46, 0x49, 0x20, 0x47, 0x4f, 0x54, 0x20, 0x49, 0x50, 0x0d, 0x0a, 0x0d, 0x0a, 0x4f, 0x4b, 0x0d, 0x0a};
uint8_t ATCWJAP1_CUR_RESPONSE[]={0x57, 0x49, 0x46, 0x49, 0x20, 0x44, 0x49, 0x53, 0x43, 0x4f, 0x4e, 0x4e, 0x45, 0x43, 0x54, 0x0d, 0x0a, 0x57, 0x49, 0x46, 0x49, 0x20, 0x43, 0x4f, 0x4e, 0x4e, 0x45, 0x43, 0x54, 0x45, 0x44, 0x0d, 0x0a, 0x57, 0x49, 0x46, 0x49, 0x20, 0x47, 0x4f, 0x54, 0x20, 0x49, 0x50, 0x0d, 0x0a, 0x0d, 0x0a, 0x4f, 0x4b, 0x0d, 0x0a};
uint8_t ATCIPSTART_RESPONSE[]=	{0x30, 0x2c, 0x43, 0x4f, 0x4e, 0x4e, 0x45, 0x43, 0x54, 0x0d, 0x0a, 0x0d, 0x0a, 0x4f, 0x4b, 0x0d, 0x0a};
uint8_t ATCIPSEND_RESPONSE[]=	{0x0d, 0x0a, 0x4f, 0x4b, 0x0d, 0x0a, 0x3e, 0x20};
uint8_t ATCIPCLOSE_RESPONSE[]=	{0x43, 0x4c, 0x4f, 0x53, 0x45, 0x44, 0x0d, 0x0a, 0x0d, 0x0a, 0x4f, 0x4b, 0x0d, 0x0a};
uint8_t ERRO_6[]=				{0x0d, 0x0a, 0x52, 0x65, 0x63, 0x76, 0x20, 0x34, 0x38, 0x20, 0x62, 0x79, 0x74, 0x65, 0x73, 0x0d, 0x0a, 0x0d, 0x0a, 0x53, 0x45, 0x4e, 0x44, 0x20, 0x4f, 0x4b, 0x0d, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t ERRO_0[]=				{0x0d, 0x0a, 0x4f, 0x4b, 0x0d, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00};

//List of commands and response sizes
#define AT_COMMAND_SIZE 	4
#define AT_RESPONSE_SIZE 	11

#define ATE_COMMAND_SIZE 	6
#define ATE_RESPONSE_SIZE 	13

#define ATCWMODE_CUR_COMMAND_SIZE	17
#define ATCWMODE_CUR_RESPONSE_SIZE	6

#define ATCWJAP_CUR_COMMAND_SIZE	47
#define ATCWJAP_CUR_RESPONSE_SIZE	35

#define ATCWJAP1_CUR_RESPONSE_SIZE	52

#define ATCIPSTART_COMMAND_SIZE		38
#define ATCIPSTART_RESPONSE_SIZE	17

#define ATCIPSEND_COMMAND_SIZE	15
#define ATCIPSEND_RESPONSE_SIZE	8

#define ATCIPCLOSE_COMMAND_SIZE	13
#define ATCIPCLOSE_RESPONSE_SIZE	14

#define NTPREQUEST_COMMAND_SIZE	48
#define NTPREQUEST_RESPONSE_SIZE	86

//Setenta anos em segundos (para compatibilidade com unix)
const uint32_t seventyYears  = 2208988800;

#endif /* INC_ESP8266_USART_COMMANDS_H_ */
