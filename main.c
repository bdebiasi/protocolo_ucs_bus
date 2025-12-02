/** 
  ******************************************************************************
  * @file main.c
  * @brief Adjustable LED blinking speed using STM8S-DISCOVERY touch sensing key
  * Application example.
  * @author STMicroelectronics - MCD Application Team
  * @version V2.0.0
  * @date 15-MAR-2011
  ******************************************************************************
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2008 STMicroelectronics</center></h2>
  * @image html logo.bmp
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "stm8_tsl_api.h"
#include "stm8s_tim4.h"
#include "stm8s_uart2.h"
#include <stdbool.h>

#define LCD_PORT                              GPIOB
#define LCD_RS                                GPIO_PIN_0     
#define LCD_EN                                GPIO_PIN_1   
#define LCD_DB4                               GPIO_PIN_2       
#define LCD_DB5                               GPIO_PIN_3
#define LCD_DB6                               GPIO_PIN_4 
#define LCD_DB7                               GPIO_PIN_5

#define clear_display                         0x01                
#define goto_home                             0x02
#define cursor_direction_inc                  (0x04 | 0x02)
#define cursor_direction_dec                  (0x04 | 0x00)
#define display_shift                         (0x04 | 0x01) 
#define display_no_shift                      (0x04 | 0x00)
 
#define display_on                            (0x08 | 0x04)
#define display_off                           (0x08 | 0x02) 
#define cursor_on                             (0x08 | 0x02)       
#define cursor_off                            (0x08 | 0x00)   
#define blink_on                              (0x08 | 0x01)   
#define blink_off                             (0x08 | 0x00)   
                                    
#define _8_pin_interface                      (0x20 | 0x10)   
#define _4_pin_interface                      (0x20 | 0x00)      
#define _2_row_display                        (0x20 | 0x08) 
#define _1_row_display                        (0x20 | 0x00)
#define _5x10_dots                            (0x20 | 0x40)      
#define _5x7_dots                             (0x20 | 0x00)
 
#define DAT                                   1
#define CMD                                   0
#define RX_BUFFER_SIZE                        32
#define VERIFICADOR 0x02

#define TAM_MAX_BUFFER 30
#define TIMEOUT 1000

#define PIN_ENABLE_485	GPIOE, GPIO_PIN_2
#define PORT_485 GPIOE
#define ENDERECO_FIXO 0x01

volatile uint8_t rx_buffer[RX_BUFFER_SIZE];

void LCD_GPIO_init(void);
void LCD_init(void);  
void LCD_send(unsigned char value, unsigned char mode);
void LCD_4bit_send(unsigned char lcd_data);              
void LCD_putstr(char *lcd_string);               
void LCD_putchar(char char_data);             
void LCD_clear_home(void);            
void LCD_goto(unsigned char  x_pos, unsigned char  y_pos);
void toggle_EN_pin(void);
void toggle_io(unsigned char lcd_data, unsigned char bit_pos, unsigned char pin_num);

#define MilliSec       1
#define Sec           10

void CLK_Configuration(void);
void GPIO_Configuration(void);
void ExtraCode_Init(void);
void ExtraCode_StateMachine(void);
void Toggle(void);
void delay_ms(uint16_t ms);
void LED_On_For(GPIO_TypeDef* PORT, GPIO_Pin_TypeDef PIN, uint16_t segundos);
void acende_desliga_led(int nro_led, bool liga_led);
unsigned char verificar_status_botao(int botao);
void pisca_led(int nro_led, int piscadas, int tempo_piscada);
void ComandoLCD(unsigned char *buffer, int tamanho_msg);
void processar_bytes(unsigned char *enviado, unsigned int tamanho);
unsigned char CalcularBCC(unsigned char *buffer, unsigned int tamanho);

u8 BlinkSpeed = 6;
int NumberOfStart;
int CheckFlag = 1;


void delay_ms(uint16_t ms)
{
		uint16_t i = 0;
    while (ms--)
    {
        for (i = 0; i < 1000; i++)
            continue;
    }
}

void LED_On_For(GPIO_TypeDef* PORT, GPIO_Pin_TypeDef PIN, uint16_t segundos)
{
    GPIO_WriteHigh(PORT, PIN);
    delay_ms(segundos * 1000);
    GPIO_WriteLow(PORT, PIN);
}

void acende_desliga_led(int nro_led, bool liga_led)
{
    if (liga_led){
        if (nro_led == 1){
            GPIO_WriteHigh(GPIOD, GPIO_PIN_0);
        } else {
            GPIO_WriteHigh(GPIOD, GPIO_PIN_7);
        }
    } else {
        if (nro_led == 1){
            GPIO_WriteLow(GPIOD, GPIO_PIN_0);
        } else {
            GPIO_WriteLow(GPIOD, GPIO_PIN_7);
        }
    }
}

unsigned char verificar_status_botao(int botao)
{
    if (botao == 1){
        if (GPIO_ReadInputPin(GPIOD, GPIO_PIN_4) == RESET){
            return 0x01;
        }
        return 0x00;
    } else {
        if (GPIO_ReadInputPin(GPIOD, GPIO_PIN_3) == RESET){
            return 0x01;
        }
        return 0x00;
    }
}

void pisca_led(int nro_led, int piscadas, int tempo_piscada)
{
    uint16_t intervalo = 300;
    int i;

    if (nro_led == 1) {
        for (i=0; i < piscadas; i++){
            LED_On_For(GPIOD, GPIO_PIN_0, tempo_piscada);
            delay_ms(intervalo);
        }
    } else {
        for (i=0; i < piscadas; i++){
            LED_On_For(GPIOD, GPIO_PIN_7, tempo_piscada);
            delay_ms(intervalo);
        }
    }
}


void ComandoLCD(unsigned char *buffer, int tamanho_msg){
	unsigned char pos_inicial_lcd = buffer[5];
	unsigned char msg[32]; 										
	unsigned int i;
	
	LCD_clear_home();
	
	for(i=0;i<tamanho_msg;i++){
		msg[i] = buffer[6+i]; 
	}
	
	msg[i] = '\0';
	
	LCD_send(pos_inicial_lcd, CMD); 
	if(tamanho_msg > 0){
		LCD_putstr(msg);
	}
}


void GPIO_485_Configuration(void)
{
	GPIO_DeInit(PORT_485);
	GPIO_Init(PIN_ENABLE_485, GPIO_MODE_OUT_PP_LOW_FAST);
}


void AlterarEstado485Enable(unsigned int status)
{
	if(status == 0){
		GPIO_WriteLow(PIN_ENABLE_485);
	}
	else{
		GPIO_WriteHigh(PIN_ENABLE_485);
	}
}

void processar_bytes(unsigned char *enviado, unsigned int tamanho) {
    unsigned char byte4 = enviado[4];
    unsigned char dados_retorno = 0x10;
    unsigned char comando_entendido = 0x06;
		int i;
		int tamanho_retorno;
		
	unsigned char dado_retorno[] = {0, 0, 0, 0, 0, 0, 0, 0};

    if (enviado[0] == VERIFICADOR) {  

        if (4 < tamanho) {

            if (byte4 == 0x01){
                dados_retorno = verificar_status_botao(1);
            }

            else if (byte4 == 0x02){
                dados_retorno = verificar_status_botao(2);
            }
            
            else if (byte4 == 0x03) {
								unsigned char estado = enviado[5];
                bool liga_led = true;
								
                if (estado == 0x00){
                    liga_led = false;
                }

                acende_desliga_led(1, liga_led);
            }

            else if (byte4 == 0x04) {
								unsigned char estado = enviado[5];
                bool liga_led = true;
								
                if (estado == 0x00){
                    liga_led = false;
                }

                acende_desliga_led(2, liga_led);
            }

            else if (byte4 == 0x05) {
                int piscadas = enviado[5];
                int tempo_piscada = enviado[6];
								
                pisca_led(1, piscadas, tempo_piscada);
            }

            else if (byte4 == 0x06) {
                int piscadas = enviado[5];
                int tempo_piscada = enviado[6];
								
                pisca_led(2, piscadas, tempo_piscada);
            }

            else if (byte4 == 0x07) { //até 16 bits
                unsigned char tamanho_msg = tamanho - 6;
                											
                ComandoLCD(enviado, tamanho_msg);
            }

            else {
                comando_entendido = 0x15;
            }

        }
    
    }
    // retorno precisa ser: stx, tamanho do pacote, destino, origem, comando, dados, bcc
				
		dado_retorno[0] = 0x02;
		dado_retorno[2] = enviado[3];
		dado_retorno[3] = ENDERECO_FIXO;
		dado_retorno[4] = byte4;
		dado_retorno[5] = comando_entendido;
		
		
    if (dados_retorno != 0x10){
				dado_retorno[1] = 0x07;
				dado_retorno[6] = dados_retorno;
				tamanho_retorno = 7;
				dado_retorno[7] = CalcularBCC(dado_retorno, tamanho_retorno);
		}
		
		else {
			dado_retorno[1] = 0x06;
			tamanho_retorno = 6;
			dado_retorno[6] = CalcularBCC(dado_retorno, tamanho_retorno);
		}
		
		AlterarEstado485Enable(1); // modo transmissão
		delay_ms(1);

    for (i = 0; i < tamanho_retorno; i++) {
        UART2_SendData8(dado_retorno[i]);
        while (!UART2_GetFlagStatus(UART2_FLAG_TXE));
    }

    while (!UART2_GetFlagStatus(UART2_FLAG_TC)); // final real do envio

    AlterarEstado485Enable(0); // volta para RX
		
				   
}

unsigned char CalcularBCC(unsigned char *buffer, unsigned int tamanho)
{
    unsigned char bcc = 0x00;
    unsigned int i;

    for (i = 0; i < tamanho - 1; i++)
    {
        bcc ^= buffer[i];
    }

    return bcc;
}

unsigned char bufferRX[30];

void main(void)
{
    unsigned long TempoAux;
    unsigned char dado;
    int i;

    unsigned int tam_buffer_entrada = 0; // tamanho do buffer
    unsigned int cont_timeout = 0;       // contador de timeout caso buffer demore para ser atualizado

    /* Configura clocks */
    CLK_Configuration();

    /* Configura GPIOs */
    GPIO_Configuration();

    /* Inicializa LCD */
    LCD_init();  
    LCD_clear_home();
    LCD_goto(3, 0);                       
    LCD_putstr("TESTE"); 

    /* Inicializa UART e 485 */
    UART2_Init(9600, UART2_WORDLENGTH_8D, UART2_STOPBITS_1, UART2_PARITY_NO,
               UART2_SYNCMODE_CLOCK_DISABLE, UART2_MODE_TXRX_ENABLE);
    GPIO_485_Configuration();
		AlterarEstado485Enable(0);

    while(1) {
				AlterarEstado485Enable(0);
        /* Recebe dado da UART */
        if (UART2_GetFlagStatus(UART2_FLAG_RXNE) == TRUE) {
            dado = UART2_ReceiveData8();

            if (tam_buffer_entrada < TAM_MAX_BUFFER) {
                bufferRX[tam_buffer_entrada] = dado;
                tam_buffer_entrada++;
            }
            cont_timeout = 0; // reset timeout
        }

        /* Se já começou a ler, conta timeout */
        if (tam_buffer_entrada > 0) {
            cont_timeout++;
            if (cont_timeout > TIMEOUT) {

                unsigned char tamanho = bufferRX[1];
                unsigned char bcc = bufferRX[tam_buffer_entrada - 1];
                unsigned char bccRecebido = CalcularBCC(bufferRX, tam_buffer_entrada);
                unsigned char endereco_destino = bufferRX[2];
                unsigned char stx = bufferRX[0];

                /* Valida BCC */
								
								if (bufferRX[0] != 0x02) {
										goto limpar_buffer;
								}

                if (bccRecebido != bcc) {
                    LCD_clear_home();
                    LCD_goto(1, 0);                       
                    LCD_putstr("FALHABCC");
                    goto limpar_buffer; // limpa buffer no final da iteração
                }

                /* Valida endereço e STX */
                if (endereco_destino != ENDERECO_FIXO) {
                    LCD_clear_home();
                    LCD_goto(1, 0);                       
                    LCD_putstr("ERROEND");
                    goto limpar_buffer;
                }

                if (stx != 0x02 || (tam_buffer_entrada) != tamanho + 1) {
                    LCD_clear_home();
                    LCD_goto(1, 0);                       
                    LCD_putstr("PAUSTX");
                    goto limpar_buffer;
                }

                /* Pacote válido: processa */
																
                processar_bytes(bufferRX, tamanho);
								
								
								goto limpar_buffer;

                limpar_buffer:
                    // Limpa buffer apenas aqui, após erro ou processamento
                    for (i = 0; i < TAM_MAX_BUFFER; i++) {
                        bufferRX[i] = 0;
                    }
                    tam_buffer_entrada = 0;
                    cont_timeout = 0;
            }
        }
    }
}



void CLK_Configuration(void)
{
  /* Fmaster = 16MHz */
  CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);

}

void GPIO_Configuration(void)
{
    /* GPIOD reset */
    GPIO_DeInit(GPIOD);
		

    /* Configure PD0 (LED1) as output push-pull low (led switched on) */
    GPIO_Init(GPIOD, GPIO_PIN_0, GPIO_MODE_OUT_PP_LOW_FAST);
    GPIO_Init(GPIOD, GPIO_PIN_7, GPIO_MODE_OUT_PP_LOW_FAST);
	//GPIO_Init(GPIOD, GPIO_PIN_3, GPIO_MODE_OUT_PP_LOW_FAST);
	//GPIO_Init(GPIOD, GPIO_PIN_4, GPIO_MODE_OUT_PP_LOW_FAST);
    //GPIO_Init(GPIOD, GPIO_PIN_5, GPIO_MODE_OUT_PP_LOW_FAST);
		//GPIO_Init(GPIOD, GPIO_PIN_6, GPIO_MODE_IN_FL_NO_IT);
		GPIO_Init(GPIOD, GPIO_PIN_3, GPIO_MODE_IN_PU_NO_IT);
		GPIO_Init(GPIOD, GPIO_PIN_4, GPIO_MODE_IN_PU_NO_IT);
	// Acender o LED em PD1
    //GPIO_WriteHigh(GPIOD, GPIO_PIN_3);

    // Apagar o LED em PD1
    //GPIO_WriteLow(GPIOD, GPIO_PIN_3);

    // Ler botão em PD3
    //if (GPIO_ReadInputPin(GPIOD, GPIO_PIN_3) == RESET) {
        // Botão pressionado
    //}
}


void LCD_GPIO_init(void)
{
	unsigned long Tempo_Aux;
    GPIO_Init(LCD_PORT, LCD_RS, GPIO_MODE_OUT_PP_HIGH_FAST);
    GPIO_Init(LCD_PORT, LCD_EN, GPIO_MODE_OUT_PP_HIGH_FAST);
    GPIO_Init(LCD_PORT, LCD_DB4, GPIO_MODE_OUT_PP_HIGH_FAST);
    GPIO_Init(LCD_PORT, LCD_DB5, GPIO_MODE_OUT_PP_HIGH_FAST);
    GPIO_Init(LCD_PORT, LCD_DB6, GPIO_MODE_OUT_PP_HIGH_FAST);
    GPIO_Init(LCD_PORT, LCD_DB7, GPIO_MODE_OUT_PP_HIGH_FAST);
    //delay_ms(10);
	for (Tempo_Aux=0;Tempo_Aux<0xFFFF;Tempo_Aux++) continue;
}

void LCD_init(void)
{                                     
    LCD_GPIO_init();    
    toggle_EN_pin();
            
    GPIO_WriteLow(LCD_PORT, LCD_RS);            
    GPIO_WriteLow(LCD_PORT, LCD_DB7);   
    GPIO_WriteLow(LCD_PORT, LCD_DB6);   
    GPIO_WriteHigh(LCD_PORT, LCD_DB5);   
    GPIO_WriteHigh(LCD_PORT, LCD_DB4);                      
    toggle_EN_pin();
             
    GPIO_WriteLow(LCD_PORT, LCD_DB7);   
    GPIO_WriteLow(LCD_PORT, LCD_DB6);   
    GPIO_WriteHigh(LCD_PORT, LCD_DB5);   
    GPIO_WriteHigh(LCD_PORT, LCD_DB4);  
    toggle_EN_pin();
 
    GPIO_WriteLow(LCD_PORT, LCD_DB7);   
    GPIO_WriteLow(LCD_PORT, LCD_DB6);   
    GPIO_WriteHigh(LCD_PORT, LCD_DB5);   
    GPIO_WriteHigh(LCD_PORT, LCD_DB4);  
    toggle_EN_pin();                  
 
    GPIO_WriteLow(LCD_PORT, LCD_DB7);   
    GPIO_WriteLow(LCD_PORT, LCD_DB6);   
    GPIO_WriteHigh(LCD_PORT, LCD_DB5);        
    GPIO_WriteLow(LCD_PORT, LCD_DB4);  
    toggle_EN_pin();
 
    LCD_send((_4_pin_interface | _2_row_display | _5x7_dots), CMD);
    LCD_send((display_on | cursor_off | blink_off), CMD); 
    LCD_send(clear_display, CMD);         
    LCD_send((cursor_direction_inc | display_no_shift), CMD);
}   
 
 
void LCD_send(unsigned char value, unsigned char mode)
{                               
    switch(mode)
    {
        case DAT:
        {
            GPIO_WriteHigh(LCD_PORT, LCD_RS);   
            break;
        }
        case CMD:
        {
            GPIO_WriteLow(LCD_PORT, LCD_RS);   
            break;
        }
    }
    
    LCD_4bit_send(value);
}  
    
 
void LCD_4bit_send(unsigned char lcd_data)       
{
    toggle_io(lcd_data, 7, LCD_DB7);
    toggle_io(lcd_data, 6, LCD_DB6);
    toggle_io(lcd_data, 5, LCD_DB5);
    toggle_io(lcd_data, 4, LCD_DB4);
    toggle_EN_pin();
    toggle_io(lcd_data, 3, LCD_DB7);
    toggle_io(lcd_data, 2, LCD_DB6);
    toggle_io(lcd_data, 1, LCD_DB5);
    toggle_io(lcd_data, 0, LCD_DB4);
    toggle_EN_pin();
}  
 
 
void LCD_putstr(char *lcd_string)
{
    do
    {
        LCD_send(*lcd_string++, DAT);
    }while(*lcd_string != '\0');
}
 
void LCD_putchar(char char_data)
{
    LCD_send(char_data, DAT);
}
 
void LCD_clear_home(void)
{
    LCD_send(clear_display, CMD);
    LCD_send(goto_home, CMD);
}
 
void LCD_goto(unsigned char  x_pos, unsigned char  y_pos)
{                                                   
    if(y_pos == 0)    
    {                              
        LCD_send((0x80 | x_pos), CMD);
    }
    else 
    {                                              
        LCD_send((0x80 | 0x40 | x_pos), CMD); 
    }
}
 
void toggle_EN_pin(void)
{
	unsigned long Tempo_Aux;
    GPIO_WriteHigh(LCD_PORT, LCD_EN);    
    //delay_ms(2);
		for (Tempo_Aux=0;Tempo_Aux<0x5FF;Tempo_Aux++) continue;
    GPIO_WriteLow(LCD_PORT,LCD_EN);   
}
 
void toggle_io(unsigned char lcd_data, unsigned char bit_pos, unsigned char pin_num)
{
    bool temp = FALSE;
    
    temp = (0x01 & (lcd_data >> bit_pos));  
                                         
    switch(temp)
    {
        case TRUE:
        {
            GPIO_WriteHigh(LCD_PORT, pin_num);   
            break;
        }
 
        default:
        {
            GPIO_WriteLow(LCD_PORT, pin_num);   
            break;
         }
      }
}
