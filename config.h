// ***********************************************************
// Baychi Soft 2013         
// ***       OpenTiny TX Configuration file               **
// ***********************************************************
// Version Number     : 2.1
// Latest Code Update : 2014-05-16

// Версия и номер компиляции. Используется для проверки целостности программы
// При модификации программы необходимо изменить одно из этих чисел 
unsigned char version[] = { 8, 1 };

//####### TX BOARD TYPE #######
// 1 = TX Expert Tiny original Board
// 2 = RX Open/orange v2 Board in TX mode (PPM input on D3 chdnnel (5-th slot)
// 3 = TX Open/orange v2 Board
// 4 = TX Hawkeye от КНА
// 5 = TX Expert 2G 
// 6 = TX DTF UHF Deluxe 

#define TX_BOARD_TYPE 1

// Проверка соответсвия типы платы настройкам Arduino
//
#if (TX_BOARD_TYPE == 6)          // HawkEye DeluxeTX (Atmega32u4) 
#if (__AVR_ATmega32U4__ != 1)
#error Wrong board selected, select Arduino Leonardo board
#endif
#else
#if ((__AVR_ATmega328P__ != 1) && (__AVR_ATmega168__ != 1))  || (F_CPU != 16000000)
#error Wrong board selected, select Arduino Pro/Pro Mini 5V/16MHz w/ ATMega328 or 168
#endif
#endif

// Время для входа в меню
#define MENU_WAIT_TIME 9999

//######### TRANSMISSION VARIABLES ##########

#define CARRIER_FREQUENCY  433075  // 433Mhz startup frequency !!! не менять
#define HOPPING_STEP_SIZE  6 // 60kHz hopping steps
#define HOPE_NUM          8  /* number of hope frequensies */ 

//###### HOPPING CHANNELS #######
//Каналы прыжков (регистры 11-18) Select the hopping channels between 0-255
//Frequency = CARRIER_FREQUENCY + (StepSize(60khz)* Channel_Number) 
static unsigned char hop_list[HOPE_NUM] = {77,147,89,167,109,189,127,209};   // по умолчанию - мои частоты

// Четыре первых регистра настроек (S/N, номер Bind, поправка частоты, разрешение коррекции частоты и детектирования FS ретранслятора
static unsigned char Regs4[7] = {99 ,72, 204, 1, 0, 0, 0 };    // последний бай - уровень отладки
// Регистры управления мощносью (19-23): канал, мошность1 - мощность3 (0-7).
#if(TX_BOARD_TYPE == 5)
static unsigned char  PowReg[5] =  { 8, 0, 2, 7, 128 };       // последне значение - константа для УМ Expert 2G 
#else
static unsigned char  PowReg[4] =  { 8, 0, 2, 7};             // что-бы не терять настройки при переходе с предыдущих версий, не исп. 2G 
#endif

//###### SERIAL PORT SPEED #######
#define SERIAL_BAUD_RATE 38400  // как у Эксперта
#define REGS_NUM 42              // количестов отображаемых регистров настроек

// Параметры пакета
#define RF_PACK_SIZE 16                 /* размер данных в пакете */
#define RC_CHANNEL_COUNT 12             /* количество каналов управления и импульсов на PPM In */

unsigned char RF_Tx_Buffer[RF_PACK_SIZE];  // буфер отсылаемого кадра
unsigned char hopping_channel = 0;
unsigned long time,start_time;   // текущее время в мс и время старта

// unsigned char RF_Mode = 0;  /* для RFMки */
signed char curTemperature=0;        // последняя температура, считанная из RFMки -64 ... +127
signed char freqCorr=0;              // поправка к частоте кварца в зависимости от t (ppm)  
unsigned char lastPower = 0;         // текущий режим мощности
unsigned int maxDif=0;               // для контроля загруженности

#define Available 0
#define Transmit 1
#define Transmitted 2
#define Receive 3
#define Received 4

#define Terminal Serial             /* По умолчанию вся информация идет через стандартный последовательный порт UART */

#if (TX_BOARD_TYPE == 1)           // Expert Tiny module
    //## RFM22BP Pinouts for Tx Tiny Board
    #define SDO_pin 12
    #define SDI_pin 11        
    #define SCLK_pin 13 
    #define SDN_pin 9

    #define IRQ_pin 7
    #define nSel_pin 10
    #define IRQ_interrupt 
    #define PPM_IN 8
    #define USE_ICP1 // Use ICP1 in input capture mode
    #define BUTTON 5
        
    #define  nIRQ_1 (PIND & 0x80)==0x80 //D7
    #define  nIRQ_0 (PIND & 0x80)==0x00 //D7
      
    #define  nSEL_on PORTB |= 0x04  //B2
    #define  nSEL_off PORTB &= 0xFB //B2
    
    #define  SCK_on PORTB |= _BV(5) // B5
    #define  SCK_off PORTB &=~_BV(5) // B5
    
    #define  SDI_on PORTB |= _BV(3)  // B3
    #define  SDI_off PORTB &=~_BV(3) // B3
    
    #define  SDO_1 (PINB & 0x10) == 0x10 //B4
    #define  SDO_0 (PINB & 0x10) == 0x00 //B4

    //#### Other interface pinouts ###
    #define GREEN_LED_pin 6
    #define RED_LED_pin 6
    
    #define Red_LED_ON   PORTD |= _BV(6);  
    #define Red_LED_OFF  PORTD &= ~_BV(6); 
    
    #define Green_LED_ON  PORTD |= _BV(6); // проецируем
    #define Green_LED_OFF PORTD &= ~_BV(6);    

// Аппаратный переключатель мощности
    #define SW1_IN A2  // Power switch 1 on 25 pin
    #define SW2_IN A3  // Power switch 2 on 26 pin  
    #define SW1_IS_ON (PINC & 0x04) == 0x00  // проверка sw1 
    #define SW2_IS_ON (PINC & 0x08) == 0x00  // проверка sw2 

#endif


#if (TX_BOARD_TYPE == 2)              // Orange reciever in transmitter mode
      //### PINOUTS OF OpenLRS Rx V2 Board
      #define SDO_pin A0
      #define SDI_pin A1        
      #define SCLK_pin A2 
      #define IRQ_pin 2
      #define nSel_pin 4
      #define IRQ_interrupt 0
      
      #define PPM_IN 8
      #define USE_ICP1           /* Use ICP1 in input capture mode */
      #define BUTTON 6

      #define  nIRQ_1 (PIND & 0x04)==0x04 //D2
      #define  nIRQ_0 (PIND & 0x04)==0x00 //D2
      
      #define  nSEL_on PORTD |= 0x10 //D4
      #define  nSEL_off PORTD &= 0xEF //D4
      
      #define  SCK_on PORTC |= 0x04 //C2
      #define  SCK_off PORTC &= 0xFB //C2
      
      #define  SDI_on PORTC |= 0x02 //C1
      #define  SDI_off PORTC &= 0xFD //C1
      
      #define  SDO_1 (PINC & 0x01) == 0x01 //C0
      #define  SDO_0 (PINC & 0x01) == 0x00 //C0

      //#### Other interface pinouts ###
      #define GREEN_LED_pin 13
      #define RED_LED_pin A3
    
      #define Red_LED_ON  PORTC |= _BV(3);
      #define Red_LED_OFF  PORTC &= ~_BV(3);
      
      #define Green_LED_ON  PORTB |= _BV(5);
      #define Green_LED_OFF  PORTB &= ~_BV(5);
    
// Аппаратный переключатель мощности
    #define SW1_IN 5  // Power switch 1 on 9 pin
    #define SW2_IN 6  // Power switch 2 on 10 pin  
    #define SW1_IS_ON (PIND & 0x20) == 0x00  // проверка sw1 
    #define SW2_IS_ON (PIND & 0x40) == 0x00  // проверка sw2 

#endif

#if (TX_BOARD_TYPE == 3)              // Orange transmitter  через прерывания
      //### PINOUTS OF OpenLRS Rx V2 Board
      #define SDO_pin 9
      #define SDI_pin 8        
      #define SCLK_pin 7 
      #define IRQ_pin 2
      #define nSel_pin 4
      #define IRQ_interrupt 0
      
      #define PPM_IN 3
      #define BUTTON 11

      #define  nIRQ_1 (PIND & 0x04)==0x04 //D2
      #define  nIRQ_0 (PIND & 0x04)==0x00 //D2
      
      #define  nSEL_on PORTD |= 0x10 //D4
      #define  nSEL_off PORTD &= 0xEF //D4
      
      #define  SCK_on PORTD |= 0x80 // D7
      #define  SCK_off PORTD &= 0x7F //D7
      
      #define  SDI_on PORTB |= 0x01 //B0
      #define  SDI_off PORTB &= 0xFE //B0
      
      #define  SDO_1 (PINB & 0x02) == 0x02 //B1
      #define  SDO_0 (PINB & 0x02) == 0x00 //B1
      
      //#### Other interface pinouts ###
      #define GREEN_LED_pin 13
      #define RED_LED_pin 12
    
      #define Red_LED_ON  PORTB |= _BV(4);
      #define Red_LED_OFF  PORTB &= ~_BV(4);
      
      #define Green_LED_ON  PORTB |= _BV(5);
      #define Green_LED_OFF  PORTB &= ~_BV(5);

     #define PPM_Pin_Interrupt_Setup  PCMSK2 = 0x08;PCICR|=(1<<PCIE2);
     #define PPM_Signal_Interrupt PCINT2_vect

// Аппаратный переключатель мощности
    #define SW1_IN A2  // Power switch 1 on 25 pin
    #define SW2_IN A3  // Power switch 2 on 26 pin  
    #define SW1_IS_ON (PINC & 0x04) == 0x00  // проверка sw1 
    #define SW2_IS_ON (PINC & 0x08) == 0x00  // проверка sw2 
     
#endif


#if (TX_BOARD_TYPE == 4)           // HawkEye TX module
    #define PPM_IN 8
    #define USE_ICP1              // Use ICP1 in input capture mode
    #define SWAP_RXTX             // управление RX/TX ки подключено наоборот 
    #define BUTTON A0
    #define RED_LED_pin 6
    #define GREEN_LED_pin 5
    #define RFM_POWER_PIN 7      // цепь управления питанием RFMки - 11я ножка
    #define RFM_POWER_MIN PORTD |= 0x80      // понизить мощу
    #define RFM_POWER_MAX PORTD &= 0x7f      // повысить мощу
    
    #define Red_LED_ON PORTD |= _BV(6)
    #define Red_LED_OFF PORTD &= ~_BV(6)
    #define Green_LED_ON PORTD |= _BV(5)
    #define Green_LED_OFF PORTD &= ~_BV(5)
    #define nIRQ_1 (PIND & 0x04)==0x04 //D2
    #define nIRQ_0 (PIND & 0x04)==0x00 //D2
    #define nSEL_on PORTD |= (1<<4) //D4
    #define nSEL_off PORTD &= 0xEF //D4
    #define SCK_on PORTB |= _BV(5) //B5
    #define SCK_off PORTB &= ~_BV(5) //B5
    #define SDI_on PORTB |= _BV(3) //B3
    #define SDI_off PORTB &= ~_BV(3) //B3
    #define SDO_1 (PINB & _BV(4)) == _BV(4) //B4
    #define SDO_0 (PINB & _BV(4)) == 0x00 //B4
    #define SDO_pin 12
    #define SDI_pin 11
    #define SCLK_pin 13
    #define SDN_pin 9
    #define IRQ_pin 2
    #define nSel_pin 4
    #define IRQ_interrupt 0

// Аппаратный переключатель мощности
    #define SW1_IN A1  // Power switch 1 on 24 pin
    #define SW2_IN A2  // Power switch 2 on 25 pin  
    #define SW1_IS_ON (PINC & 0x02) == 0x00  // проверка sw1 
    #define SW2_IS_ON (PINC & 0x04) == 0x00  // проверка sw2 
#endif

#if (TX_BOARD_TYPE == 5)              // Expert 2G board (under constraction)
    //## RFM22BP Pinouts for Tx Tiny Board
    #define SDO_pin 12
    #define SDI_pin 11        
    #define SCLK_pin 13 
    #define SDN_pin A5

    #define IRQ_pin 2
    #define nSel_pin 10
    #define IRQ_interrupt 
    #define PPM_IN 8
    #define USE_ICP1 // Use ICP1 in input capture mode
    #define BUTTON A4        
    #define nIRQ_1 (PIND & 0x04)==0x04 //D2
    #define nIRQ_0 (PIND & 0x04)==0x00 //D2
      
    #define nSEL_on PORTB |= 0x04  //B2
    #define nSEL_off PORTB &= 0xFB //B2
    
    #define SCK_on PORTB |= _BV(5) // B5
    #define SCK_off PORTB &=~_BV(5) // B5
    
    #define SDI_on PORTB |= _BV(3)  // B3
    #define SDI_off PORTB &=~_BV(3) // B3
    
    #define SDO_1 (PINB & 0x10) == 0x10 //B4
    #define SDO_0 (PINB & 0x10) == 0x00 //B4

    //#### Other interface pinouts ###
    #define RED_LED_pin A2   // C2
    #define GREEN_LED_pin A3 // C3
    
    #define Red_LED_ON   PORTC |= _BV(2);  // C2
    #define Red_LED_OFF  PORTC &= ~_BV(2); 
    
    #define Green_LED_ON  PORTC |= _BV(3);  // C3
    #define Green_LED_OFF PORTC &= ~_BV(3);    

// Аппаратный переключатель мощности
    #define SW1_IN A0  // Power switch 1 on 23 pin
    #define SW2_IN A1  // Power switch 2 on 24 pin  
    #define SW1_IS_ON (PINC & 0x01) == 0x00  // проверка sw1 
    #define SW2_IS_ON (PINC & 0x02) == 0x00  // проверка sw2 
    #define PA_VOUT_PIN 5  // PWM выход для управления мощностью       
#endif

#if (TX_BOARD_TYPE == 6)          // HawkEye DeluxeTX (Atmega32u4) 
    #define IRQ_pin 11 //PB7
    #define nSel_pin 12

    #define SWAP_RXTX             // управление RX/TX ки подключено наоборот 
    #define PPM_IN 4 // ICP1
    #define USE_ICP1 // use ICP1 for PPM input for less jitter
    #define BUTTON A0

    #define  nIRQ_1 (PINB & (1<<PINB7))==(1<<PINB7) //PB7
    #define  nIRQ_0 (PINB & (1<<PINB7))==0x00 //PB7

    #define  nSEL_on PORTD |= (1<<PORTD6) //PD6
    #define  nSEL_off PORTD &= ~(1<<PORTD6) //PD6

    #define  SCK_on  PORTB |= (1<<PORTB1)  //PB1
    #define  SCK_off PORTB &= ~(1<<PORTB1) //PB1

    #define  SDI_on  PORTB |= (1<<PORTB2)  //PB2 MOSI
    #define  SDI_off PORTB &= ~(1<<PORTB2) //PB2 MOSI

    #define  SDO_1 (PINB & (1<<PINB3)) == (1<<PINB3) //PB3 MISO
    #define  SDO_0 (PINB & (1<<PINB3)) == 0x00  //PB3 MISO

    //#### Other interface pinouts ###
    #define GREEN_LED_pin 5 //PC6
    #define RED_LED_pin 6 //PD7

    #define Red_LED_ON  PORTD |= (1<<PORTD7);
    #define Red_LED_OFF  PORTD &= ~(1<<PORTD7);

    #define Green_LED_ON   PORTC |= (1<<PORTC6);
    #define Green_LED_OFF  PORTC &= ~(1<<PORTC6);

    #define Terminal Serial1           /* Работа с терминалом через UART. Если отключить - поток пойдет через USB */
#endif

#if (TX_BOARD_TYPE == 7)              // Orange reciever тест через прерывания D3
      //### PINOUTS OF OpenLRS Rx V2 Board
      #define SDO_pin A0
      #define SDI_pin A1        
      #define SCLK_pin A2 
      #define IRQ_pin 2
      #define nSel_pin 4
      #define IRQ_interrupt 0
      
      #define PPM_IN 3
      #define BUTTON 6

      #define  nIRQ_1 (PIND & 0x04)==0x04 //D2
      #define  nIRQ_0 (PIND & 0x04)==0x00 //D2
      
      #define  nSEL_on PORTD |= 0x10 //D4
      #define  nSEL_off PORTD &= 0xEF //D4
      
      #define  SCK_on PORTC |= 0x04 //C2
      #define  SCK_off PORTC &= 0xFB //C2
      
      #define  SDI_on PORTC |= 0x02 //C1
      #define  SDI_off PORTC &= 0xFD //C1
      
      #define  SDO_1 (PINC & 0x01) == 0x01 //C0
      #define  SDO_0 (PINC & 0x01) == 0x00 //C0
      

      //#### Other interface pinouts ###
      #define GREEN_LED_pin 13
      #define RED_LED_pin A3
    
      #define Red_LED_ON  PORTC |= _BV(3);
      #define Red_LED_OFF  PORTC &= ~_BV(3);
      
      #define Green_LED_ON  PORTB |= _BV(5);
      #define Green_LED_OFF  PORTB &= ~_BV(5);

     #define PPM_Pin_Interrupt_Setup  PCMSK2 = 0x08;PCICR|=(1<<PCIE2);
     #define PPM_Signal_Interrupt PCINT2_vect
     
#endif


// Functions & variable declarations

void RF22B_init_parameter(void);
unsigned char _spi_read(unsigned char address); 
void to_sleep_mode(void);
void ppmLoop(unsigned char n=12);
extern unsigned int mppmDif,maxDif;
extern unsigned int PPM[];     // текущие длительности канальных импульсов
extern unsigned char ppmAge;   // age of PPM data
void printlnPGM(char *adr, char ln=1);   // печать строки из памяти программы ln - перевод строки
void _spi_write(unsigned char, unsigned char);  // Gfsk,  fd[8] =0, no invert for Tx/Rx data, fifo mode, txclk -->gpio 

