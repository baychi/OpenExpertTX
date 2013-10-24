// ***********************************************************
// Baychi Soft 2013         
// ***       OpenTiny TX Configuration file               **
// ***********************************************************
// Version Number     : 2.1
// Latest Code Update : 2013-10-22

// Версия и номер компиляции. Используется для проверки целостности программы
// При модификации программы необходимо изменить одно из этих чисел 
unsigned char version[] = { 2, 4 };

// Время для входа в меню
#define MENU_WAIT_TIME 9999

//######### TRANSMISSION VARIABLES ##########

#define CARRIER_FREQUENCY  433075  // 433Mhz startup frequency !!! не менять
#define HOPPING_STEP_SIZE  6 // 60kHz hopping steps
#define HOPE_NUM          8 /* number of hope frequensies */ 
#define AFC_POROG         3 /* предельное отклонение частоты, требующее коррекции */

//###### HOPPING CHANNELS #######
//Каналы прыжков (регистры 11-18) Select the hopping channels between 0-255
//Frequency = CARRIER_FREQUENCY + (StepSize(60khz)* Channel_Number) 
static unsigned char hop_list[HOPE_NUM] = {77,147,89,167,109,189,127,209};   // по умолчанию - мои частоты

// Четыре первых регистра настроек (S/N, номер Bind, поправка частоты, разрешение коррекции частоты и детектирования FS ретранслятора
static unsigned char Regs4[6] = {99 ,72, 204, 1, 0, 0 };    // последний бай - уровень отладки
// Регистры управления мощностью (19-23): канал, мошность1 - мощность3 (0-7).
static unsigned char  PowReg[4] =  { 8, 0, 2, 7 };  

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

//####### TX BOARD TYPE #######
// 1 = TX 2G/Tiny original Board
// 2 = RX Open/orange v2 Board in TX mode (PPM input on D3 chdnnel (5-th slot)
// 3 = TX Open/orange v2 Board

#define TX_BOARD_TYPE 2

#if (TX_BOARD_TYPE == 1)           // Expert Tiny module
    //## RFM22BP Pinouts for Tx Tiny Board
    #define SDO_pin 12
    #define SDI_pin 11        
    #define SCLK_pin 13 
    #define IRQ_pin 7
    #define nSel_pin 10
    #define IRQ_interrupt 0
    #define PPM_IN 8
    #define USE_ICP1 // Use ICP1 in input capture mode
    #define BUTTON 6
        
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
    #define GREEN_LED_pin 5
    #define RED_LED_pin 4
    
    #define Red_LED_ON   PORTD |= _BV(5);  
    #define Red_LED_OFF  PORTD &= ~_BV(5); 
    
    #define Green_LED_ON  PORTD = PORTC;  // фиктивно
    #define Green_LED_OFF PORTD = PORTC;    

    #define Serial_PPM_IN PORTB |= _BV(0) //Serial PPM IN
#endif


#if (TX_BOARD_TYPE == 2)              // Orange reciever in transmittr mode
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
    
#endif

#if (TX_BOARD_TYPE == 3)              // Orange transmitter  через прерывания
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
extern unsigned int PPM[RC_CHANNEL_COUNT+2];     // текущие длительности канальных импульсов
extern unsigned char ppmAge; // age of PPM data
void printlnPGM(char *adr, char ln=1);   // печать строки из памяти программы ln - перевод строки

