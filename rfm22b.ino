// **********************************************************
// Baychi soft 2013
// **      RFM22B/23BP/Si4432 Transmitter with Expert protocol **
// **      This Source code licensed under GPL            **
// **********************************************************
// Latest Code Update : 2013-10-22
// Supported Hardware : Expert Tiny, Orange/OpenLRS Tx/Rx boards (store.flytron.com)
// Project page       : https://github.com/baychi/OpenExpertTX
// **********************************************************
 
#define NOP() __asm__ __volatile__("nop") 

// Режимы RFM для 7-го регистра
#define RF22B_PWRSTATE_READY    01 
#define RF22B_PWRSTATE_TX       0x09 
#define RF22B_PWRSTATE_RX       05 
#define RF22B_PWRSTATE_POWERDOWN  00 

// Режимы прерывания для 5-го регистра
#define RF22B_Rx_packet_received_interrupt   0x02 
#define RF22B_PACKET_SENT_INTERRUPT  04 

unsigned char ItStatus1, ItStatus2; 

unsigned char read_8bit_data(void); 
// void to_tx_mode(void); 
void to_ready_mode(void); 
void send_8bit_data(unsigned char i); 
void send_read_address(unsigned char i); 
void _spi_write(unsigned char address, unsigned char data); 
void RF22B_init_parameter(void); 

void port_init(void);   
unsigned char _spi_read(unsigned char address); 
void Write0( void ); 
void Write1( void ); 
void timer2_init(void); 
void Write8bitcommand(unsigned char command); 
void to_sleep_mode(void); 
 
 
//***************************************************************************** 

//-------------------------------------------------------------- 
void Write0( void ) 
{ 
    SCK_off;  
    NOP(); 
     
    SDI_off; 
    NOP(); 
     
    SCK_on;  
    NOP(); 
} 
//-------------------------------------------------------------- 
void Write1( void ) 
{ 
    SCK_off;
    NOP(); 
     
    SDI_on;
    NOP(); 
     
    SCK_on; 
    NOP(); 
} 
//-------------------------------------------------------------- 
void Write8bitcommand(unsigned char command)    // keep sel to low 
{ 
 unsigned char n=8; 
    nSEL_on;
    SCK_off;
    NOP(); 
    nSEL_off; 

    while(n--) 
    { 
         if(command&0x80) Write1(); 
         else Write0();    
         command = command << 1; 
    } 
    SCK_off;
}  

//-------------------------------------------------------------- 
void send_read_address(unsigned char i) 
{ 
  i &= 0x7f; 
  
  Write8bitcommand(i); 
}  
//-------------------------------------------------------------- 
void send_8bit_data(unsigned char i) 
{ 
  unsigned char n = 8; 
  SCK_off;
  NOP(); 

  while(n--)    { 
     if(i&0x80)  Write1(); 
     else  Write0();    
     i = i << 1; 
   } 
   SCK_off;
}  
//-------------------------------------------------------------- 

unsigned char read_8bit_data(void) 
{ 
  unsigned char Result, i; 
  
 SCK_off;
 NOP(); 

 Result=0; 
 for(i=0;i<8;i++)    {                    //read fifo data byte 
       Result=Result<<1; 
       SCK_on;
       NOP(); 
 
       if(SDO_1)  { 
         Result|=1; 
       } 
       SCK_off;
       NOP(); 
  } 
  return Result; 
}  

//-------------------------------------------------------------- 
unsigned char _spi_read(unsigned char address) 
{ 
  unsigned char result; 
  send_read_address(address); 
  result = read_8bit_data();  
  nSEL_on; 
  
  return(result); 
}  

//-------------------------------------------------------------- 
void _spi_write(unsigned char address, unsigned char data) 
{ 
  address |= 0x80; 
  Write8bitcommand(address); 
  send_8bit_data(data);  
  nSEL_on;
}  


//-------Defaults 7400 baud---------------------------------------------- 
void RF22B_init_parameter(void) 
{ 
   _spi_write(0x07, 0x80);      // сброс всех регистров RFM
  delay(1);
  ItStatus1 = _spi_read(0x03); // read status, clear interrupt   
  ItStatus2 = _spi_read(0x04); 
  
  _spi_write(0x06, 0x00);    // no interrupt: no wakeup up, lbd, 
  _spi_write(0x07, RF22B_PWRSTATE_READY);      // disable lbd, wakeup timer, use internal 32768,xton = 1; in ready mode 
  if(Regs4[2]!=0)  _spi_write(0x09, Regs4[2]);     // точная подстройка частоты 
  else _spi_write(0x09, 199);     // если сброшена, используем умолчание   
  _spi_write(0x0a, 0x05);    // выход CLK 2 МГц ?

#if(TX_BOARD_TYPE==4)         // в Навке почему-то извратились
   _spi_write(0x0b, 0x15);    // gpio0 TX State
   _spi_write(0x0c, 0x12);    // gpio1 RX State 
#else 
   _spi_write(0x0b, 0x12);    // gpio0 TX State
   _spi_write(0x0c, 0x15);    // gpio1 RX State 
#endif

  _spi_write(0x0e, 0x00);    // gpio 0, 1,2 NO OTHER FUNCTION. 
  
// From Expert
  _spi_write(0x1F, 0x03);    //  Clock recovery
  _spi_write(0x1E, 0x0A);    //  AFC timing
  _spi_write(0x12, 0x60);    //  Режим измерения температуры -40..+85 C
  _spi_write(0x13, 0xF8);    //  Смещение температуры ?
  _spi_write(0x0F, 0x80);    //  АЦП для измерения температуры
  _spi_write(0x1D, 0x40);    //  AFC enable

//--------------------------
   
  _spi_write(0x1c, 0x26);    // IF filter bandwidth
  _spi_write(0x20, 0x44);    // clock recovery oversampling rate  
  _spi_write(0x21, 0x00);    // 0x21 , rxosr[10--8] = 0; stalltr = (default), ccoff[19:16] = 0; 
  _spi_write(0x22, 0xF2);    // 0x22   ncoff =5033 = 0x13a9 
  _spi_write(0x23, 0x7C);    // 0x23 
  _spi_write(0x24, 0x06);    // 0x24 
  _spi_write(0x25, 0x7D);    // 0x25 clock recovery ...
  _spi_write(0x2a, 0x0f);    // AFC limiter

  _spi_write(0x30, 0xAC);    // Mode: enable packet handler, msb first, enable crc CCITT, 
  _spi_write(0x32, 0x8C);    // Header: 0x32address enable for headere byte 0, 1,2,3, receive header check for byte 0, 1,2,3 
  _spi_write(0x33, 0x0A);    // header 3, 2, 1,0 used for head length, fixed packet length, synchronize word length 3, 2, 
  _spi_write(0x34, 0x04);    // 2 bytes preamble 
  _spi_write(0x35, 0x22);    // expect full preamble 
  _spi_write(0x36, 0x2d);    // synchronize word 3
  _spi_write(0x37, 0xd4);    // 2
  _spi_write(0x38, 0x00);    // 1
  _spi_write(0x39, 0x00);    // 0

  _spi_write(0x3e, RF_PACK_SIZE);    // total tx 16 byte 

//  _spi_write(0x6d, 0x58);    // 7 set power min TX power 
  _spi_write(0x6d, 0x0F);    // 7 set power min TX power 

// 7400 bps data rate
  _spi_write(0x6e, 0x3C); //  RATE_7400 
  _spi_write(0x6f, 0x9F); //   

  _spi_write(0x69, 0x60);    //  AGC enable
  _spi_write(0x70, 0x2E);    //  manchester enable!!!
  _spi_write(0x79, 0x00);    // no hopping (1-st channel)
  _spi_write(0x7a, HOPPING_STEP_SIZE);    // 60khz step size (10khz x value) // no hopping 

  _spi_write(0x71, 0x00);  // Gfsk,  fd[8] =0, no invert for Tx/Rx data, fifo mode, txclk -->gpio 
  _spi_write(0x71, 0x23);  // Gfsk,  fd[8] =0, no invert for Tx/Rx data, fifo mode, txclk -->gpio 
  _spi_write(0x72, 0x0E);  // frequency deviation setting to 8750

  //band 434.075
 _spi_write(0x75, 0x53);   // 433075 кГц  
 _spi_write(0x76, 0x4C);    
 _spi_write(0x77, 0xE0); 
}


//----------------------------------------------------------------------- 
void rx_reset(void) 
{ 
  _spi_write(0x07, RF22B_PWRSTATE_READY); 
  _spi_write(0x7e, 36);    // threshold for rx almost full, interrupt when 36 byte received 
  ppmLoop(); 

  _spi_write(0x08, 0x03);    // clear fifo disable multi packet 
  _spi_write(0x08, 0x00);    // clear fifo, disable multi packet 
  ppmLoop(); 

  _spi_write(0x07, RF22B_PWRSTATE_RX );  // to rx mode 
  _spi_write(0x05, RF22B_Rx_packet_received_interrupt); 
  ppmLoop(); 

  ItStatus1 = _spi_read(0x03);  //read the Interrupt Status1 register 
  ItStatus2 = _spi_read(0x04);  
}  
//-----------------------------------------------------------------------    

void to_rx_mode(void) 
{  
  to_ready_mode(); 
  Sleep(50); 
  rx_reset(); 
}  


#define ONE_BYTE_MKS 1055              // временнной интервал между байтами при скорости 74000
#define TX_MAX_WAIT_TIME 31999         // предельное время ожидания при отправке пакета   

void sendOnFlyStd()
{
  byte i,j,k,m,b12;
  word pwm;
  unsigned long tx_start;

// Цикл формирования и отсылки данных на лету
  tx_start=micros();
  i=1;  // первый байт мы уже отослали         

  while(nIRQ_1 && ((micros()-tx_start) < TX_MAX_WAIT_TIME)) {  // ждем окончания, но не бесконечно
    ppmLoop();

    if((i<RF_PACK_SIZE) && (micros()-tx_start) > (i+2)*ONE_BYTE_MKS) {   // ждем пока не подойдет реальное время отправки (запас 1 байт)
        if(i == 1) {                                 // формируем байт старших бит и делаем предварительную подготовку пакета
          for(m=j=k=b12=0; m<8; m++) {                   
            pwm=PPM[m];
            if(pwm >= 1024) j |= 1<<m;              // байт старших бит
            RF_Tx_Buffer[m+2]=((pwm>>2)&255);       // 8 средних бит первых 8 каналов
            if(m<7) {
              if(pwm&2) k |= (2<<m);                 // байт младших бит (10 бит кодирование в упр. байте)
              if(pwm&1) b12 |= (1<<m);               // и байт дополнительных 11-х бит
            }
          }
          _spi_write(0x7f,(RF_Tx_Buffer[1]=j));      // отсылаем байт старших бит
          RF_Tx_Buffer[RC_CHANNEL_COUNT+3]=k;        // запоминаем младшие биты
          if(Regs4[5]) RF_Tx_Buffer[RC_CHANNEL_COUNT+1]=b12; // и сверхмладшие
        } else if(i < RF_PACK_SIZE-2) {              // отправляем основные байты данных
           pwm=PPM[i-2];
           if(i < 10) {                              // для первых 8 байт нужна особая обработка, учитывая что байт старших бит
             k=1<<(i-2); j=0;                        // уже отправлен и его не изменить.
             if(pwm >= 1024) j=k;
             if((RF_Tx_Buffer[1]&k) == j) {          // если бит совпадает, с прежним, можно кодировать средние биты
                RF_Tx_Buffer[i]=((pwm>>2)&255);      // 8 младших бит первых 8 каналов
                if(Regs4[5]) {                       // если надо, то еще и 11-е биты вместо последнего канала пакуем 
                  if(pwm&1) RF_Tx_Buffer[RC_CHANNEL_COUNT+1] |= k;       // 11-е первых 8-ми за счет канала 12
                  else RF_Tx_Buffer[RC_CHANNEL_COUNT+1] &= ~k;            
                }
                if(i < 9) {
                  k=k+k;                             // маска младшего бита
                  if(pwm&2) RF_Tx_Buffer[RC_CHANNEL_COUNT+3] |= k;       // и не забываем уточнять младшие биты 
                  else RF_Tx_Buffer[RC_CHANNEL_COUNT+3] &= ~k;           // первых 7 ми каналов в управляющем байте 
                }
             }     
          } else {
            if(Regs4[5] == 0 || (i != RC_CHANNEL_COUNT && i != RC_CHANNEL_COUNT+1)) RF_Tx_Buffer[i]=(pwm>>3)&255;        // остальные каналы просто формируем на лету
            else if(i == RC_CHANNEL_COUNT) {         // в режиме 11 бит формируем последние младшие биты каналов, за счет канала 11
              pwm=PPM[i-2];
              RF_Tx_Buffer[i]=pwm&7;                 // 3 бита для байта 9
              pwm=PPM[i-1];
              RF_Tx_Buffer[i] |= (pwm&7)<<3;         // 3 бита для байта 10 
              pwm=PPM[7];  
              RF_Tx_Buffer[i] |= (pwm&2)<<5;         // и один бит для байта 8 
            }
          }  
          _spi_write(0x7f,RF_Tx_Buffer[i]);           // отсылаем очередной байт
          
        } else if(i == RF_PACK_SIZE-2) {
          RF_Tx_Buffer[RC_CHANNEL_COUNT+2] = CRC8(RF_Tx_Buffer+2, RC_CHANNEL_COUNT); // формируем СRC8
          _spi_write(0x7f,RF_Tx_Buffer[RC_CHANNEL_COUNT+2]);  // и отсылаем ее
        } else {
          if(FSstate == 2) RF_Tx_Buffer[RC_CHANNEL_COUNT+3]=0x01; // Нажатие кнопки == команде установки FS 
          _spi_write(0x7f,RF_Tx_Buffer[RC_CHANNEL_COUNT+3]);  // отсылаем последний, управляющий байт (или байт 10-х бит)
        } 
        i++;                                       // продвигаемся в буфере
    }
  }
}

//-------------------------------------------------------------- 
//
// Функция для подготовки отправляемых каналов на лету
// Добавляет биты канала в очередные байты отправляемые в эфир

static byte curB, bitMask, byteCntr, chCntr;

void prepFB(byte idx)
{
  if(idx < byteCntr) return;     // уже сформирванно
  word pwm;
  byte i;
  
  
  if(Regs4[5] == 3) pwm=PPM[10-chCntr++];  // в режиме 3 каналы передаются в обратном порядке
  else pwm=PPM[chCntr++];             // берем очередной канал 
  
  for(i=0; i<11; i++) {               // переносим его 11 бит в буфер отправки
    if(pwm & 1) curB |= bitMask;
    if(bitMask == 0x80) {             // дошли до конца байта
       RF_Tx_Buffer[++byteCntr]=curB; // кладем его в буфер
       curB=0; bitMask=1;             // и начинаем новый
     } else bitMask+=bitMask;     // двигаем маску 
     pwm >>= 1;
  }
}  

void sendOnFlyFutaba(void)                         // отправка на лету футабовского кадра
{
  byte i,j;
  unsigned long tx_start;

// Цикл формирования и отсылки данных на лету
  tx_start=micros();
  i=1;  // первый байт мы уже отослали         
  curB=0, bitMask=1, byteCntr=0, chCntr=0;         // готовим контекст  

  while(nIRQ_1 && ((micros()-tx_start) < TX_MAX_WAIT_TIME)) {  // ждем окончания, но не бесконечно
    ppmLoop();

    if((i<RF_PACK_SIZE) && (micros()-tx_start) > (i+2)*ONE_BYTE_MKS) {   // ждем пока не подойдет реальное время отправки (запас 1 байт)
       if(i == RF_PACK_SIZE-1) {
          RF_Tx_Buffer[RF_PACK_SIZE-1] = CRC8(RF_Tx_Buffer+1, RF_PACK_SIZE-2); // формируем СRC8
       }  else {
         prepFB(i);
         if(i == RF_PACK_SIZE-2) {
           RF_Tx_Buffer[RF_PACK_SIZE-2] &= 0x3f;   // уберем лишнее из последнего байта
           if(FSstate == 2)                        // и добавим флаг кадра с FS
             RF_Tx_Buffer[RF_PACK_SIZE-2] |= 0x80;
         }  
       }
        _spi_write(0x7f,RF_Tx_Buffer[i]);   // отсылаем байт
        i++;                                       // продвигаемся в буфере
    }
  }
}

bool to_tx_mode(void)                  // Подготовка и отсылка пакета на лету
{ 
  byte i;
  word pwm;

  to_ready_mode();
 
// Управление мощностью
//
  if(PowReg[0] > 0 && PowReg[0] <= 13) { // если задан канал 1-12
    cli();
    pwm=PPM[PowReg[0]-1];                // берем длительность импульса
    sei();
    if(pwm < 682) i=PowReg[1];           // и определяем, какую мощность требуют 
    else if(pwm >= 1364) i=PowReg[3];
    else i=PowReg[2];
  } else if(PowReg[0] == 0) {           // аппаратный переключатель на 3-х позиционном тумблере
    if(SW1_IS_ON) i=PowReg[1];          // внизу - режим минмальной мощности
    else if(SW2_IS_ON) i=PowReg[3];     // вверху- режим максимальной мощности
    else i=PowReg[2];                   // в середине - средняя мощность
  } else i=PowReg[3];                   // если не задано, используем макс. мощность  
  
  i=i&7;
  if(++lastPower > (8-i)*3) {
    lastPower=0;                        // мигаем с частотой пропорциональной мощности
     Green_LED_ON;
  }
  _spi_write(0x6d, i+8);                // Вводим мощность в RFMку 

  _spi_write(0x08, 0x03);    // disABLE AUTO TX MODE, enable multi packet clear fifo 
  _spi_write(0x08, 0x00);   
  
  _spi_write(0x7f,(RF_Tx_Buffer[0]=Regs4[1]));       // отсылаем номер линка в FIFO
  _spi_write(0x05, RF22B_PACKET_SENT_INTERRUPT);     // переводим 

  ItStatus1 = _spi_read(0x03);         //  read the Interrupt Status1 register 
  ItStatus2 = _spi_read(0x04); 

  while((micros() - lastSent) < 31350) ppmLoop(1);   // точная предварительная подгонка старта
//  while((micros() - lastSent) < 31300);             // точная окончательная подгонка старта
  _spi_write(0x07, RF22B_PWRSTATE_TX);              // старт передачи
  lastSent += 31500;                                // формируем момент следующей отправки пакета

  if(Regs4[5] >= 2) sendOnFlyFutaba();
  else sendOnFlyStd(); 
  
  if(nIRQ_1) {                                     // Если не дождались отсылки
    Serial.println("Timeout");
    return false;
  } 

  Green_LED_OFF;

  to_ready_mode();

  return true;
}  

//-------------------------------------------------------------- 
void to_ready_mode(void) 
{ 
  ItStatus1 = _spi_read(0x03);   
  ItStatus2 = _spi_read(0x04); 
  ppmLoop(); 
  _spi_write(0x07, RF22B_PWRSTATE_READY); 
}  

//-------------------------------------------------------------- 
void to_sleep_mode(void) 
{ 
  to_rx_mode();
} 

//############# FREQUENCY HOPPING FUNCTIONS #################
void Hopping(void)
{
    unsigned char hn;
    
    hn=Regs4[2]; 
    if(hn!=0) {                 // если разрешена точная подстройка
      if(Regs4[3]) hn-=-freqCorr; // дополнительно учитываем поправку температуры
      _spi_write(0x09, hn);     // точная подстройка частоты 
      ppmLoop();
    }
    hopping_channel++;
    if (hopping_channel>=HOPE_NUM) hopping_channel = 0;
    hn=hop_list[hopping_channel];
    _spi_write(0x79, hn);
}

// Получение температуры и температурная коррекция кварца
void getTemper (void)
{
   _spi_write(0x0f, 0x80);               // запускаем измерение температуры 
   SleepMks(333);                             
   curTemperature=_spi_read(0x11)-0x40;  // читаем температуру из АЦП
#if(TX_BOARD_TYPE == 1 || TX_BOARD_TYPE == 4)  // RFM23BP
   if(curTemperature > -40 && curTemperature < 85) freqCorr=-(curTemperature-25)/10;     // работаем только вреальном диапазоне
   else freqCorr=0;
#else
   if(curTemperature < 20) freqCorr=-(curTemperature-30)/10;            // область холода
   else if(curTemperature > 30) freqCorr=-(curTemperature-30)/7;        // область жары
   else freqCorr=0;
#endif
   if(Regs4[3] == 255) freqCorr=-freqCorr; // отрицательная поправка
   ppmLoop();
}  

// Процедуры автонастройеи передатчика
//
#define NOISE_POROG 222                   // порог шума, выше которого канал не рассматриваем 
#define BUT_HOLD_TIME 4999                // время ожидания старта бинда по кнопке
#define LAST_FREQ_CHNL 232                // верхняя граница рассматриваемого диапазона

static byte zw=29;                         // ширина одной частотной зоны

byte findChnl(byte zn)                   // поиск лучшего чатотного канала в зоне zn или соседних
{
  byte n=scanZone(zn);                   // сканируем свою зону
  if(n > LAST_FREQ_CHNL) {
    if(zn) n=scanZone(zn-1);              // попробуем поискать в соседней
    if(n < LAST_FREQ_CHNL) return n;
    if(zn < 7) n=scanZone(zn-1);          // попробуем поискать в соседней
  }

  return n;
}  

byte scanZone(byte zn)                     // поиск лучшего чатотного канала в зоне zn
{
    byte *buf=(byte *)pulseBuf;            // используем буфер SBUS для работы
    byte i,j,n,m;                             
    zn=zn*zw; m=zn+zw;                     // это границы нашей зоны
    j=n=255;
    for(i=zn; i<m; i++) {
      if(buf[i] < NOISE_POROG) {
        if(buf[i] < j) { j=buf[i]; n=i; }   // ищем минимуум в зоне
      }
    }
    if(j<NOISE_POROG) {                     // если нашли успешно 
      Serial.write(' '); Serial.print(n);
      Serial.print("/"); Serial.print(j/4);
      if(n) buf[n-1]=255;                   // запрещаем соседние частоты
      if(n>1) buf[n-2]=255;
      buf[n]=buf[n+1]=buf[n+2]=255;        // и свою, что-б никто не полез
    }  
    return n;
}


char btxt1[] PROGMEM = "\r\nTry to new bind...";
char btxt2[] PROGMEM = "\n\rFreq/noise: ";
char btxt3[] PROGMEM = "Error: too noise!";
char btxt4[] PROGMEM = "Bind N=";

// Автонастройка передатчика
//
void makeAutoBind(byte p)                 // p=0 - c проверкой кнопки, p=255 - принудительный сброс настроек          
{
  byte *buf=(byte *)pulseBuf;               // используем буфер SBUS для накопления
  byte i,j,k,n;
  unsigned long t=0;
  
  if(p == 0) {                   // при обычном вызове
    t=millis()+BUT_HOLD_TIME;
    Green_LED_OFF;
    do {
      wdt_reset();               //  поддержка сторожевого таймера
      checkFS(0);                 // отслеживаем нажатие кнопочки
      
      if(millis() > t) Green_LED_ON;  // индициируем, что пора заканчивать
    } while(FSstate);
    if(millis() < t) return;     // не нажимали или держали мало 
    if(!read_eeprom()) goto reset;    
  } else {
    if(p == 255) {               // принудительный вызов со сбросом регистров в дефолт 
reset:
      Regs4[3]=1; Regs4[4]=Regs4[5]=Regs4[6]=0;
      PowReg[0]=0; PowReg[1]=0; PowReg[2]=2; PowReg[3]=7;
    }
    Green_LED_ON;
  }

repeat:  
  if(Regs4[2] < 170 || Regs4[2] > 230) Regs4[2]=199;  // на всякий случай проверим поправку
  RF22B_init_parameter();      // готовим RFMку 
  to_rx_mode(); 
  rx_reset();

  printlnPGM(btxt1,0);
  for(i=0; i<255; i++) buf[i]=0;
//     
// Постоянно сканируем эфир, на предмет выявления возможных частот 

  for(k=0; k<31; k++) {  
    wdt_reset();               //  поддержка сторожевого таймера
    for(i=0; i<LAST_FREQ_CHNL; i++) {
      _spi_write(0x79, i);      // ставим канал
      delayMicroseconds(999);   // ждем пока перекинется
      j=_spi_read(0x26);        // читаем уровень сигнала
      j >>= 3;                  // делим на 8 
      if(j > 7) j=8;            // что-бы не перебрать
      buf[i] += j;              // накапливаем среднее
    }
  }
  Green_LED_OFF;                // сигнализируем о завершении
  
  zw=(LAST_FREQ_CHNL+7)/8;      // ширина зоны  
  
  printlnPGM(btxt2,0);          // раскидываем каналы прыжков
  hop_list[0]=findChnl(0);
  hop_list[1]=findChnl(4);
  hop_list[2]=findChnl(1);
  hop_list[3]=findChnl(5);
  hop_list[4]=findChnl(2);
  hop_list[5]=findChnl(6);
  hop_list[6]=findChnl(3);
  hop_list[7]=findChnl(7);
  Serial.println();
  
  for(i=0; i<8; i++) {         // проверяем каналы
   if(hop_list[i] > LAST_FREQ_CHNL) {
      printlnPGM(btxt3);
      Red_LED_Blink(5);
      goto repeat;
    }
  }    
  t=millis() - t;         // номер бинда формируется благодоря случайному всеремни отпускания кнопки
  i=t&255; if(!i) i++;    // избегаем 0-ля
  printlnPGM(btxt4,0);  Serial.println(i); // отображаем
  Regs4[1]=i;

  write_eeprom();         // запоминаем настройки 
}
