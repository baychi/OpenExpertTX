//-------------------------------------------------------
// 
// Реализация протокола SBUS через ICP/INTx(возможно) вход.
// Драйвер ICP фиксирует моменты изменения лог уровня в буфере sbusbuf.
// Фоновый процесс sbusLoop анализирует первичный буфер и преобразует моменты перехода в байты протокола

volatile word PPM[RC_CHANNEL_COUNT];     // текущие длительности канальных импульсов
volatile byte ppmAge = 0; // age of PPM data
volatile byte ppmCounter = RC_CHANNEL_COUNT; // ignore data until first sync pulse
volatile byte ppmDetecting = 1; // countter for microPPM detection
volatile byte ppmMicroPPM = 0;  // status flag for 'Futaba microPPM mode'
volatile byte ppmSBUS = 0;      // status flag SBUS mode'

/****************************************************
 * Interrupt Vector
 ****************************************************/
static byte pCntr=0;
static word lastP=0;

static void processPulse(word pulse)
{

  pCntr++;
  if(pCntr&1) {
    lastP=pulse;
    return;
  }
  pulse+=lastP;                                  // так как наш драйвер ловит оба фронта

  if (ppmDetecting) {                              // на стадии детектирования определяем наличие импульсов
    if (ppmDetecting>50) {
      ppmDetecting=0;
      if(ppmSBUS>20) {
        ppmMicroPPM=0xff;                        // признак работы в режиме SBUS
      } else if(ppmMicroPPM>10) {
        ppmMicroPPM=1;                           // работа в режиме Futaba PPM12
      } else {
        ppmMicroPPM=0;
      }
      // Serial.println(ppmMicroPPM?"Futaba micro mode":"Normal PPM mode");
    } else {
      if (pulse<100)                              // импульсы короче 50 мкс трактуем, как SBUS
        ppmSBUS++;                           
      else if (pulse<1500)                        // импульсы короче 750 мкс трактуем, как Футабские
        ppmMicroPPM++;

      ppmDetecting++;
    }
  } else {
    if (!ppmMicroPPM) {
      pulse= (pulse+1)>>1;                        // divide by 2 to get servo value on normal PPM
    }

    if (pulse > 2500) {      // Verify if this is the sync pulse (2.5ms)
      ppmCounter = 0;             // -> restart the channel counter
      ppmAge = 0;                 // brand new PPM data received
    } else if ((pulse > 700) && (ppmCounter < RC_CHANNEL_COUNT)) { // extra channels will get ignored here
      PPM[ppmCounter++] = pulse;   // Store measured pulse length (converted)
    } else {
      ppmCounter = RC_CHANNEL_COUNT; // glitch ignore rest of data
    }
  }
}

#define PULSE_BUF_SIZE 64
volatile word pulseBuf[PULSE_BUF_SIZE];       // буфер фронтов импульсов принимаемых через PPM вход
volatile word *pbPtr=pulseBuf;    // указатель в буфере импульсов
#ifdef USE_ICP1 // Use ICP1 in input capture mode

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
    if(pbPtr >= pulseBuf+PULSE_BUF_SIZE)) pbPtr=pulseBuf;
    TCNT1 = 0; // reset the timer1 value for next
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



byte sbusPkt[25];        // буфер для борки пакета sbus
byte curByte=0, bitFlag=0, bitCntr=0, pktPtr=0;
unsigned long lastIn=0;    // момент последнего поступления данных
static word lastPulse=0;

void newPkt( void)
{
  if(pktPtr) endPkt();   // если пакет был в стадии формирования завершим его и передадим обработчику 
  curByte=0, bitFlag=0, bitCntr=0;
  pktPtr=0;
}

void endPkt(void)
{
}


void sbusPulse(word val)
{
  val=(val+32)/56;
  if(bitCntr == 0) { val--; bitCntr=1; }  // стартовый бит пропускаем
  while(val > 0) {
    curByte |= bitFlag << (bitCntr-1);
    bitCntr++;
    if(bitCntr > 8)  {
      sbusPkt[pktPtr] = curByte;
      curByte=0; bitCntr=0;
      if(++pktPtr > sizeof(sbusPkt)) {
        endPkt();
        break;
      }
    } 
    val--;
  }
  bitFlag=1-bitFlag;
}  

word *ppmPtr= (word* ) pulseBuf;
unsigned long loopTime=0;

void ppmLoop(void)
{
  word i,j,*lastPb;
//--------------------------------------
  if(loopTime > 0) {
    i=micros()-loopTime;
    if(maxDif < i) maxDif=i;
  }
  loopTime=micros();
//-------------------------------

  cli();
  lastPb=(word *)pbPtr;   // берем текущиее положение указателя драйвера
  sei();   
 
  while(ppmPtr != lastPb) {  // пока есть новые данные в буфере
     cli();
     i=*ppmPtr++;
     sei();   
     if(ppmPtr >= pulseBuf+PULSE_BUF_SIZE) ppmPtr=(word *)pulseBuf;
     j=i-lastPulse; 
     lastPulse=i;
     if(ppmMicroPPM != 255) processPulse(j); 
     else {
       lastIn=micros();
       sbusPulse(j); 
     }
  }
  if(pktPtr && (micros() - lastIn) > 999) endPkt();  // завершение пакета по таймауту
 
}

void showSbus()
{
//        if(1) {
//          for(i=0; i<j; i++) { Serial.print(int(vbuf[i]),HEX); Serial.print(" "); }
//          Serial.println(); 
//        } else {
//          Serial.println("Waiting Sbus\r");
//          delay(299);
//        }
}
