// **********************************************************
// Baychi soft 2013
// **      RFM22B/23BP/Si4432 Transmitter with Expert protocol **
// **      This Source code licensed under GPL            **
// **********************************************************
// Latest Code Update : 2013-10-22
// Supported Hardware : Expert Tiny, Orange/OpenLRS Tx/Rx boards (store.flytron.com)
// Project page       : https://github.com/baychi/OpenExpertTX
// **********************************************************


// Функции меню терминала
//
static char regs[] PROGMEM = {1, 2, 3, 4, 5, 6, 11,12,13,14,15,16,17,18,19,20,21,22 }; // номера отображаемых регистров
static char help[][32] PROGMEM = {
  "Bind N",
  "Freq correction const",
  "Term corr.(0=no, 1=+, 255=-)",
  "FS check enable",
  "11bit/10ch(1=yes,2/3=Futaba)",
  "Debug out (1-PPM, 2-perf.)",
  "Hop F1",
  "Hop F2",
  "Hop F3",
  "Hop F4",
  "Hop F5",
  "Hop F6",
  "Hop F7",
  "Hop F8",
  "Power switch chan (1-13,0=SW)",
  "Power min (0-7, +128=highU)",  
  "Power middle (0-7,+128=highU)",
  "Power max (0-7, +128=highU)"
};  
  
char htxt1[] PROGMEM = "\r\nBaychi soft 2013";
char htxt2[] PROGMEM = "TX Open Expert V2 F";
char htxt3[] PROGMEM = "Press 'm' to start MENU";

void printHeader(void)
{
  printlnPGM(htxt1);
  printlnPGM(htxt2,0); Serial.println(version[0]);
  showRegs();
  printlnPGM(htxt3);
}  

void printlnPGM(char *adr, char ln)   // печать строки из памяти программы
{
  byte b;
  while(1) {
    b=pgm_read_byte(adr++);
    if(!b) break;
    Serial.write(b);
  }

  if(ln) Serial.println();  
}


bool checkMenu(void)   // проверка на вход в меню
{
   int in; 
   
   if (Serial.available() > 0) {
      in= Serial.read();             // все, что пришло, отображаем
      if(in == 'c' || in == 'C') mppmDif=maxDif=0; // сброс статистики загрузки
      if(in == 'm' || in == 'M') return true; // есть вход в меню
   } 
   return false;                        // не дождались 
}


void getStr(char str[])             // получение строки, завершающейся Enter от пользователя
{
  int in,sn=0;
  str[0]=0;
  while(1) {
    wdt_reset();               //  поддержка сторожевого таймера
    if (Serial.available() > 0) {
       in= Serial.read();             // все, что пришло, отображаем
       if(in > 0) {
          Serial.write(in);
          if(in == 0xd || in == 0xa) {
            Serial.println();
            return;                     // нажали Enter
          }
          if(in == 8) {                 // backspace, удаляем последний символ
            if(sn) sn--;
            continue;
          } 
          str[sn]=in; str[sn+1]=0;
          if(sn < 6) sn++;              // не более 6 символов
        }
     } else delay(1);
  }
}

#define R_AVR 199               // усреднение RSSI

byte margin(byte v)
{
   if(v < 10) return 0; 
   else if(v>70) return 60;

   return  v-10;
}

void print3(unsigned char val)  // печать 3-цифр с выравниваем пробелами
{
  if(val < 100) Serial.write(' ');
  if(val < 10) Serial.write(' ');
  Serial.print(val);
  Serial.write(' ');
}  

byte _spi_read(byte address); 
void _spi_write(byte address,byte val); 
char ntxt1[] PROGMEM = "FHn: Min Avr Max";

void showNoise(char str[])             // отображаем уровень шумов по каналам
{
  byte fBeg=0, fMax=254;
  byte rMin, rMax;
  word rAvr;
  byte i,j,k;

  rAvr=atoi(str+1);           // считаем параметры, если есть в виде Nbeg-end
  if(rAvr > 0 && rAvr < 255) {
     fBeg=rAvr;
     for(i=2; i<10; i++) {
      if(str[i] == 0) break;
      if(str[i] == '-') {
        rAvr=atoi(str+i+1);
        if(rAvr > fBeg && rAvr < 255) fMax=rAvr;
        break;
      }
    }
  }
  
  RF22B_init_parameter();      // подготовим RFMку 
  to_rx_mode(); 
 
  printlnPGM(ntxt1);
 
  for(i=fBeg; i<=fMax; i++) {    // цикл по каналам
     _spi_write(0x79, i);       // ставим канал
     delayMicroseconds(749);
     rMin=255; rMax=0; rAvr=0;
     for(j=0; j<R_AVR; j++) {   // по каждому каналу 
       delayMicroseconds(99);
       k=_spi_read(0x26);         // Read the RSSI value
       rAvr+=k;
       if(k<rMin) rMin=k;         // min/max calc
       if(k>rMax) rMax=k;
     }
     print3(i);
     k=':';
     for(j=0; j<HOPE_NUM; j++) {   // отметим свои частоты
        if(hop_list[j] == i) {
          k='#';
        }
     }
     Serial.write(k); Serial.write(' ');
     print3(rMin);   
     k=rAvr/R_AVR;  print3(k);
     print3(rMax);

     if(str[0] == 'N') {         // если надо, печатаем псевдографику 
       rMin=margin(rMin); 
       rMax=margin(rMax); 
       k=margin(k); 

       for(j=0; j<=rMax; j++) {                         // нарисуем псевдографик
         if(j == k) Serial.write('*');
         else if(j == rMin) Serial.write('<');
         else if(j == rMax) Serial.write('>');
         else if(j>rMin && j <rMax) Serial.write('.');
         else Serial.write(' ');
       }
     }
     
     Serial.println();
     wdt_reset();               //  поддержка сторожевого таймера
  }
}

    
// Перенесем текст меню в память программ
char mtxt1[] PROGMEM = "\n\rTo Enter MENU Press ENTER";
char mtxt2[] PROGMEM = "Type Reg and press ENTER, type Value and press ENTER (q=Quit; Nx-y=Show noise)";
char mtxt3[] PROGMEM = "\r\nRg=Val \tComments -----------------------";
char mtxt4[] PROGMEM = "Make new bind? Are you sure(y/n)?";

void showRegs(void)         // показать значения регистров
{
  unsigned char i,j=0,k;
  
  printlnPGM(mtxt3,0); printlnPGM(htxt2+14,0); Serial.println(version[0]);
  
  for(int i=1; i<=REGS_NUM; i++) {
    if(pgm_read_byte(regs+j) == i) {
      Serial.print(i);
      Serial.write('=');
      Serial.print(read_eeprom_uchar(i));
      Serial.write('\t');
      printlnPGM(help[j]);   // читаем строки из программной памяти
      j++;
    }
  }
}


void doMenu()                       // работаем с меню
{
  char str[8];
  int reg,val;

  to_sleep_mode();
  printlnPGM(mtxt1);
  getStr(str);
  if(str[0] == 'q' || str[0] == 'Q') return;     // Q - то quit
  
  while(1) {
    showRegs();
    printlnPGM(mtxt2);

rep:  
    getStr(str);

    if(str[0] == 'n' || str[0] == 'N') {  // отсканировать и отобразить уровень шума 
       showNoise(str);
       goto rep;
    }
    if(str[0] == 'f' || str[0] == 'F') {  // непрерывная передача на канале F для ностройки частоты по приборам
       freqTest(str);
       goto rep;
    }
    if(str[0] == 'r' && str[1] == 'e' && str[2] == 'b' && str[3] == 'i' && str[4] == 'n' && str[5] == 'd') { // rebinding
      printlnPGM(mtxt4,0);
      getStr(str);
      if(str[0] == 'y' || str[0] == 'Y') {
        makeAutoBind(1);    
        continue;
      }    
    }      

    if(str[0] == 'q' || str[0] == 'Q') return;     // Q - то quit
    reg=atoi(str);
    if(reg<0 || reg>REGS_NUM) continue; 

    getStr(str);
    if(str[0] == 'q' || str[0] == 'Q') return;     // Q - то quit
    val=atoi(str);
    if(val<0 || val>255) continue; 
    if(reg == 0 && val ==0) continue;              // избегаем потери s/n

    Serial.print(reg); Serial.write('=');   Serial.println(val);  // Отобразим полученное
    
    write_eeprom_uchar(reg,val);  // пишем регистр
    read_eeprom();                // читаем из EEPROM    
    write_eeprom();               // и тут-же пишем, что-бы сформировать КС 
  }    
}  
