// **********************************************************
// Baychi soft 2013
// **      RFM22B/23BP/Si4432 Transmitter with Expert protocol **
// **      This Source code licensed under GPL            **
// **********************************************************
// Latest Code Update : 2013-10-22
// Supported Hardware : Expert Tiny, Orange/OpenLRS Tx/Rx boards (store.flytron.com)
// Project page       : https://github.com/baychi/OpenExpertTX
// **********************************************************

#include "config.h"
#include <EEPROM.h>
#include <avr/wdt.h>

byte FSstate = 0;          // 1 = waiting timer, 2 = send FS, 3 sent waiting BUTTON release
unsigned long FStime = 0;  // time when button went down...
unsigned long lastSent = 0;

void checkFS(bool led=true)        // проверка нажатия кнопочки для отсылки FS кадра
{
  switch (FSstate) {
  case 0:
    if (!digitalRead(BUTTON)) {
      FSstate = 1;
      FStime = millis();
    }

    break;

  case 1:
    if (!digitalRead(BUTTON)) {
      if ((millis() - FStime) > 500) {
        FSstate = 2;
      }
    } else {
      FSstate = 0;
    }
    break;

  case 2:
    if (digitalRead(BUTTON)) {
      FSstate = 0;
    } else {
      if(led) Green_LED_ON;
    }

    break;
  }
}

void setup(void)
{
#if(SDN_pin != 0)    
   pinMode(SDN_pin, OUTPUT); //SDn
   digitalWrite(SDN_pin, LOW);
#endif

#if(RFM_POWER_PIN != 0)    
   pinMode(RFM_POWER_PIN, OUTPUT); // управление питанием RFMки
   RFM_POWER_MIN;
#endif

   pinMode(SDO_pin, INPUT); //SDO
   pinMode(SDI_pin, OUTPUT); //SDI        
   pinMode(SCLK_pin, OUTPUT); //SCLK
   pinMode(IRQ_pin, INPUT); //IRQ
   digitalWrite(IRQ_pin, HIGH);
   pinMode(nSel_pin, OUTPUT); //nSEL
     
   pinMode(0, INPUT); // Serial Rx
   pinMode(1, OUTPUT);// Serial Tx
   digitalWrite(0, HIGH);
   digitalWrite(1, HIGH);

  //LED and other interfaces
   pinMode(RED_LED_pin, OUTPUT);   //RED LED
   pinMode(GREEN_LED_pin, OUTPUT);   //GREEN LED
   pinMode(BUTTON, INPUT);   //Buton
   digitalWrite(BUTTON, HIGH);

#if(SW1_IN!=0)
   pinMode(SW1_IN, INPUT);   // ключ 1
   digitalWrite(SW1_IN, HIGH);
   pinMode(SW2_IN, INPUT);   // ключ 2
   digitalWrite(SW2_IN, HIGH);
#endif

   pinMode(PPM_IN, INPUT);   //PPM from TX
   digitalWrite(PPM_IN, HIGH); // enable pullup for TX:s with open collector output

   Serial.begin(SERIAL_BAUD_RATE);

   setupPPMinput();
   EIMSK &=~1;          // запрещаем INT0 
   sei();
}

void loop(void)        // главный фоновый цикл 
{
  byte i,j,k;
  word pwm;

  printHeader();
  wdt_enable(WDTO_1S);     // запускаем сторожевой таймер 
  Red_LED_ON;
  delay(99);
  for(byte i=0; i<RC_CHANNEL_COUNT; i++) PPM[i]=0;
  Red_LED_OFF;
  makeAutoBind(0);     // проверяем на необходимость автобинда и делаем его, если надо
  eeprom_check();      // Считываем и проверяем FLASH и настройки    

  RF22B_init_parameter();
  rx_reset();
  ppmAge = 255;

  Serial.println();
  showState();     // отображаем режим и дебуг информацию 

  for(i=0; i<32; i++) {   // ждем старта RFM ки до 3-х секунд 
    getTemper();           // меряем темперартуру
    if (curTemperature > -40 && _spi_read(0x0C) != 0) break;  // если даные вменяемы, можно стартовать
    RF22B_init_parameter();
    delay(99);          
  }

  rx_reset();

  mppmDif=maxDif=0;       // сброс статистики
  unsigned long time = micros();
  lastSent=time; 

  while(1) {
    ppmLoop();
    wdt_reset();               // поддержка сторожевого таймера

    if(checkMenu()) {          // проверяем на вход в меню
       doMenu(); 
       lastSent=micros(); 
    }
    
    if (_spi_read(0x0C) == 0) {     // detect the locked module and reboot
      Serial.println("RFM lock");
      Green_LED_ON;
      Sleep(249);
re_init:
      RF22B_init_parameter();
      rx_reset();
      mppmDif=maxDif=0; // !!!!!!!
      continue;      
    }

    ppmLoop();
    time = micros();
    i=checkPPM();                   // Проверяем PPM на запрет передачи      
    if(i && ppmAge < 7) {
      checkFS();                               // отслеживаем нажатие кнопочки
      pwm=time - lastSent;                     //  проверяем не пора ли готовить отправку
      if(pwm >= 28999) {
        if(pwm > 32999) lastSent=time-31500;   // при слишком больших разбежках поправим время отправки
        Hopping();
        if(!to_tx_mode()) goto re_init;        // формируем и посылаем пакет
        getTemper();                           // меряем темперартуру
        ppmAge++;
        showState();                           // отображаем режим и дебуг информацию 
      }
    } else if(ppmAge == 255) {
       getTemper();                            // меряем темперартуру
       showState(); 
       Sleep(99);
    } else if(ppmAge > 5 || i == 0) {
extern byte prevFS;  
      if(!prevFS) to_sleep_mode();             // нет PPM - нет и передачи
       getTemper();                            // меряем темперартуру
       showState(); 
       Sleep(99);
    }
  }  
}
