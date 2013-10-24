// **********************************************************
// Baychi soft 2013
// **      RFM22B/23BP/Si4432 Transmitter with Expert protocol **
// **      This Source code licensed under GPL            **
// **********************************************************
// Latest Code Update : 2013-10-22
// Supported Hardware : Expert Tiny, Orange/OpenLRS Tx/Rx boards (store.flytron.com)
// Project page       : https://github.com/baychi/OpenExpertTX
// **********************************************************

//-------------------------------------------------------
// 
// Реализация протокола SBUS через ICP/INTx(возможно) вход.
// Драйвер ICP фиксирует моменты изменения лог уровня в буфере sbusbuf.
// Фоновый процесс sbusLoop анализирует первичный буфер и преобразует моменты перехода в байты протокола

word PPM[RC_CHANNEL_COUNT+2];     // текущие длительности канальных импульсов
byte ppmAge = 0; // age of PPM data
byte ppmCounter = RC_CHANNEL_COUNT; // ignore data until first sync pulse
byte ppmDetecting = 1; // countter for microPPM detection
byte ppmMicroPPM = 0;  // status flag for 'Futaba microPPM mode'
byte ppmSBUS = 0;      // status flag SBUS mode'

/****************************************************
 * Interrupt Vector
 ****************************************************/
static byte pCntr=0;
static word lastP=0;

static void processPulse(word pulse)
{
  pCntr++;                                      // так как наш драйвер ловит оба фронта
  if(!(pCntr&1)) {                                 // половину нужно игнорировать  
    lastP=pulse;
    return;
  }
  pulse+=lastP;                                  

  if (ppmDetecting) {                           // на стадии детектирования определяем наличие импульсов
    if (ppmDetecting>50) {
      ppmDetecting=0;

      if(ppmSBUS>10) {
        ppmMicroPPM=0xff;                        // признак работы в режиме SBUS
      } else if(ppmMicroPPM>10) {
        ppmMicroPPM=1;                           // работа в режиме Futaba PPM12
      } else {
        ppmMicroPPM=0;
      }
    } else {
      if (pulse<100)                              // импульсы короче 50 мкс трактуем, как SBUS
        ppmSBUS++;                           
      else if (pulse<1500)                        // импульсы короче 750 мкс трактуем, как Футабские
        ppmMicroPPM++;

      ppmDetecting++;
    }
  } else {
    if (!ppmMicroPPM) {
      pulse= (pulse+1)>>1;                        // переводим в микросекунды
    }

    if (pulse > 2500) {            // Если обнаружена пауза свыше 2.5 мс
      ppmCounter = 0;              // переходим к первому канала
      ppmAge = 0;                  // призна поступления нового пакета
    } else if ((pulse > 700) && (ppmCounter < RC_CHANNEL_COUNT)) { 
      PPM[ppmCounter++] = pulse;   // Запоминаем очередной импульс
    } else {
      ppmCounter = RC_CHANNEL_COUNT; // Короткие пички отключают цикл
    }
  }
}


#define PULSE_BUF_SIZE 180                 // размер буфера импульсов
volatile word pulseBuf[PULSE_BUF_SIZE];   // буфер фронтов импульсов принимаемых через PPM вход
volatile word *pbPtr=pulseBuf;            // указатель в буфере импульсов
byte eCntr1=0;                            // счетчик ошибок четности sbus
word eCntr2=0;                            // счетчик битых пакетов sbus

// -----------------------------------------------------------
//
// Обработчики прерываний

#ifdef USE_ICP1 // Правильный режим -  использованиие ICP1 in input capture mode

ISR(TIMER1_CAPT_vect)
{
  *pbPtr++=ICR1;  // просто сохраняем значение и продвигаемся в буфере
  if(pbPtr >= pulseBuf+PULSE_BUF_SIZE) pbPtr=pulseBuf;

  if(TCCR1B == ((1 << WGM12) | (1 << WGM13) |  (1 << CS11))) 
       TCCR1B = ((1 << WGM12) | (1 << WGM13) | (1 << CS11) | (1<<ICES1));  // меняем тип фронта
  else TCCR1B = ((1 << WGM12) | (1 << WGM13) | (1 << CS11));               // это нужно для SBUS
}  

void setupPPMinput()
{
  ppmDetecting = 1;
  ppmMicroPPM = 0;
  TCCR1A = ((1 << WGM10) | (1 << WGM11));
  TCCR1B = ((1 << WGM12) | (1 << WGM13) | (1 << CS11));   // Setup timer1 for input capture (PSC=8 -> 0.5ms precision, falling edge)

  OCR1A = 65535;
  TIMSK1 |= (1 << ICIE1);   // Enable timer1 input capture interrupt
}

#else // sample PPM using pinchange interrupt

ISR(PPM_Signal_Interrupt)
{
    *pbPtr++=TCNT1;  // просто сохраняем значение и продвигаемся в буфере
    if(pbPtr >= pulseBuf+PULSE_BUF_SIZE) pbPtr=pulseBuf;
}

void setupPPMinput(void)
{
  ppmDetecting = 1;
  ppmMicroPPM = 0;
  // Setup timer1 for input capture (PSC=8 -> 0.5ms precision, top at 20ms)
  TCCR1A = ((1 << WGM10) | (1 << WGM11));
  TCCR1B = ((1 << WGM12) | (1 << WGM13) | (1 << CS11));
  OCR1A = 65535;
  TIMSK1 = 0;
  PPM_Pin_Interrupt_Setup
}
#endif

ISR(INT0_vect){                 // обработчик прерывания от RFMки НЕ используется
/****************************
  if (RF_Mode == Transmit) {
    RF_Mode = Transmitted;
  }

  if (RF_Mode == Receive) {
    RF_Mode = Received;
  }
**************************/  
}  

// Рвбота с SBUS протоколом

byte sbusPkt[25];          // буфер для сборки пакета sbus
byte curByte=0, bitFlag=0, bitCntr=0, pktPtr=0;   // текущий байт, бит, счетчики бит и байт
byte bit1Cntr=0;           // счетчик единичных бит
unsigned long lastIn=0;    // момент последнего поступления данных
static word lastPulse=0;

void inline endPkt(void)        // завершаем текущий пакет, проверяем и переносим данные в массив PPM длительностей
{
   byte i,j,k,m;
   word pwm,wm;
   
   if(pktPtr >= sizeof(sbusPkt) && eCntr1 == 0 &&  // если набран кворум, нет ошибок по четности
      sbusPkt[0] == 0x0F && sbusPkt[24] == 0x00 ) {  // проверяем начало и конец
      k=m=1;                                 // счетчик байт в sbus и маска бита
      for(i=0; i<RC_CHANNEL_COUNT+1; i++) {  // формируем наши PPM длительности  (13-й канал, для управления мощностью)
        pwm=0; wm=1;
        for(j=0; j<11; j++) {          // счетчик бит в представлении
          if(sbusPkt[k] & m) pwm |= wm;
          wm += wm;
          if(m == 0x80) { m=1; k++; } // обнуляем
          else m = m + m;             // или двигаем маску
        }
        PPM[i]=((pwm+pwm+pwm+pwm+pwm+5)>>3) + 880;   // формируем значение в PPM буфере
      }    
      ppmAge = 0;                    // признак обновления PPM
   } else if(pktPtr) eCntr2++;       // считаем ошибки, если были пакеты в стадии формирования

   curByte=bitFlag=bitCntr=pktPtr=bit1Cntr=eCntr1=0;
}

#define TICK_IN_BIT 20      // количество тиков по 0.5 мкс на один бит

void inline sbusPulse(word val)    // обработка очередного принятого импульса
{
  byte i=(val+TICK_IN_BIT/2)/TICK_IN_BIT;    // сколько бит формируем 
  if(i > 10) { endPkt(); return; }           // паузы трактуем, как паузы между пакетами
  
  if(bitCntr == 0) { i--; bitCntr=1; }  // стартовый бит пропускаем
  for(;  i > 0; i--) {
    bit1Cntr += bitFlag;                 // считаем единичные биты  
    curByte |= bitFlag << (bitCntr-1);   // формируем текущий байт
    if(++bitCntr == 12 || (bitCntr>9 && pktPtr >= 24))  {                  // при поступлении стопового бита
      sbusPkt[pktPtr] = curByte;         // запоминаем сформированный байт
      eCntr1+=(bit1Cntr&1);              // считаем ошибки по четности
      curByte=bitCntr=bit1Cntr=0;
      if(++pktPtr >= sizeof(sbusPkt)) {
        endPkt();
        break;
      }
    }
  }
  bitFlag=1-bitFlag;                      // меняем значение бита на противополеженное 
}  

word *ppmPtr= (word* ) pulseBuf;
unsigned long loopTime=0;
word avrLoop=0,ppmDif=0,mppmDif=0;

void ppmLoop(byte m)                       // Фоновый цикл обработки импульсов. Самая ресурсоемкая часть программы в sbus режиме
{
  word i,j,n,*lastPb;
  unsigned long lt;

  cli();
  lastPb=(word *)pbPtr;       // берем текущиее положение указателя драйвера
  sei();   

//---------------------------------- отладка для контроля времени и буфера импульсов 
  if(Regs4[5]&2) {           // если отладка включена   
    lt=micros(); 
    i=lt-loopTime;  
    if(maxDif < i) maxDif=i;  // меряем максимальное время цикла ppmLoop
    loopTime=lt;

    avrLoop=avrLoop-(avrLoop>>5) + i;     // и его усреденение за 1 сек
    
    ppmDif=lastPb-ppmPtr;
    if(ppmDif >= PULSE_BUF_SIZE) ppmDif=PULSE_BUF_SIZE+ppmDif;
    if(mppmDif<ppmDif) mppmDif=ppmDif; // меряем максимальное количество необработанных времен в буфере
  }
//-------------------------------
 
  for(n=0; ppmPtr != lastPb && n < m; n++) {  // пока есть новые данные в буфере, обрабатываем порцию размером до n 
     i=*ppmPtr++;
     if(ppmPtr >= pulseBuf+PULSE_BUF_SIZE) ppmPtr=(word *)pulseBuf;
     j=i-lastPulse; 
     lastPulse=i;
     if(ppmMicroPPM != 255) processPulse(j); 
     else sbusPulse(j); 
  }
}


// Отслеживание и отображение изменения состояния

word prevDif=0, prevErr=0, prevLat=0;
byte nchan=0, prevMode=0;
char prevTemp=0;
int ptAvr=0;
byte ptAvrCnt=0;
byte showNum=0;

bool checkTemp(void)            // проверяем, нужно ли отображать изменение температуры (что-бы не плясать на границе градуса)
{
  signed char d=prevTemp - curTemperature;

  if(d == 0) return false;

  if(abs(d) >= 2) {            // Более 1 градуса отображаем сразу
    ptAvr=0; 
    ptAvrCnt=0;
    return true;
  }
 
  if(ptAvrCnt >= 99) {        // Проверяем усреднение за 3 сек
     if((ptAvr/ptAvrCnt) != curTemperature) {
       ptAvr=0; 
       ptAvrCnt=0;
       return true;
     }
  } else {
    ptAvr += curTemperature;
    ptAvrCnt++;
  }

  return false;
}

bool showState(void)   // Отображение состояния после отправки пакета 
{
  byte i; 
  if(maxDif > 3999) maxDif=0;      // обнуляем очевидное
  
  if(checkTemp() || prevErr != eCntr2 ||  prevMode != ppmAge || prevDif != maxDif || prevLat != mppmDif) {
     prevDif=maxDif;
     prevErr=eCntr2;
     prevTemp=curTemperature;
     prevMode = ppmAge;
     prevLat = mppmDif;

     Serial.print("\r");
     if(ppmAge == 255) Serial.print("Waiting start:");
     else if(ppmAge > 5) Serial.print("Input lost:");
     else {
       if(!nchan) {            // один раз подстчитаем каналы PPM
         for(i=0; i<RC_CHANNEL_COUNT; i++) {
            if(PPM[i]) nchan++;
         }
         ppmLoop();
       } 
       if(ppmMicroPPM == 255) Serial.print("SBUS mode:");
       else {
         if(ppmMicroPPM) Serial.print("Fut750u ");
         Serial.print("PPM");   ppmLoop();
         Serial.print(nchan); Serial.print(" mode:");
       }
     }
     ppmLoop();
     Serial.print(" T=");  Serial.print((int)prevTemp);  // температура
     Serial.print(" Tc=");  Serial.print((int)freqCorr);  // поправка частоты
     ppmLoop();
     
     if(Regs4[5]&2) {           // если требуется доп. информация
       Serial.print(" A=");  Serial.print(avrLoop>>5); // средняя длительность цикла
       ppmLoop();
       Serial.print(" M=");  Serial.print(prevDif);    // макс длительность цикла
       ppmLoop();
       if(ppmMicroPPM == 255) {      // в режиме SBus 
         Serial.print(" B=");  Serial.print(prevLat);  // макс. запаздывание
         ppmLoop();
         Serial.print(" E=");  Serial.print(prevErr);  // ошибки пакетов
         ppmLoop();
       }
     }

     if(Regs4[5]&1) {        // если включен вывод PPM импульсов
       for(i=0; i<8; i++) { Serial.print("    "); ppmLoop(6); }             // подчистим грязь
     }
     Serial.println();  
     
     showNum=0;
     return true;
  } 
  
  if((Regs4[5]&1) && nchan >2) {
    Serial.print(PPM[showNum]);
    ppmLoop();
    Serial.write(' ');
    if(++showNum >= nchan) {
       Serial.write('\r');
       showNum=0;
    }
  }
  return false;
}

bool checkPPM(void)         // проверка PPM/SBUS на failSafe ретранслятора
{
  if(Regs4[4]) {                   // если проверка разрешена
    if(ppmMicroPPM == 255) {       // режим SBUS, FS = бит3 в управл. байте
      if(sbusPkt[23]&0x8) return false;
    } else {
      for(byte i=0; i<nchan; i++) {
         if(PPM[i] < 998 || PPM[i] > 2011) return false;  // проверяем выход канала за диапазон
      }
    }

  }
  return true;            // PPM в порядке
}  
