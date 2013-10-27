// **********************************************************
// Baychi soft 2013
// **      RFM22B/23BP/Si4432 Transmitter with Expert protocol **
// **      This Source code licensed under GPL            **
// **********************************************************
// Latest Code Update : 2013-10-22
// Supported Hardware : Expert Tiny, Orange/OpenLRS Tx/Rx boards (store.flytron.com)
// Project page       : https://github.com/baychi/OpenExpertTX
// **********************************************************

#define REGS_EERPON_ADR  4     /* first byte of eeprom */

#define FLASH_SIZE 16384         /* размер контроллируемой памяти программ */
#define FLASH_SIGN_ADR 64         /* адрес сигнатуры прошивки в EEPROM */
#define FLASH_KS_ADR 66           /* адрес контрольной суммы прошивки в EEPROM */
#define EEPROM_KS_ADR 68          /* адрес контрольной суммы настроек в EEPROM */


unsigned int read_eeprom_uint(int address)
{
 return (EEPROM.read(address) * 256) + EEPROM.read(address+1); 
}

unsigned char read_eeprom_uchar(int address)
{
 return  EEPROM.read(address+REGS_EERPON_ADR); 
}


void write_eeprom_uint(int address,unsigned int value)
{
 EEPROM.write(address,value / 256);  
 EEPROM.write(address+1,value % 256);  
}

void write_eeprom_uchar(int address,unsigned char value)
{
  return  EEPROM.write(REGS_EERPON_ADR+address,value); 
}


// Проверка целостности прошивки
//

int flash_check(void)
{
  unsigned int i,sign,ks=0;
  
  for(i=0; i<FLASH_SIZE; i++)        // считаем сумму 
      ks+=pgm_read_byte(i);
  
   sign=version[0] + (version[1]<<8);     // сигнатура из номера версии
   i=read_eeprom_uint(FLASH_SIGN_ADR);    // прежняя сигнатура
   if(sign != i) {                        // при несовпадении, пропишем новые значения
     write_eeprom_uint(FLASH_SIGN_ADR,sign); 
     write_eeprom_uint(FLASH_KS_ADR,ks);
   } else {                               // в противном случае проверяем КС
     i=read_eeprom_uint(FLASH_KS_ADR);
     if(i != ks) return 1;                // признак разрушенной прошивки
   }
   
   return 0;                               // все в порядке
}    


// Чтение всех настроек
bool read_eeprom(void)
{
   byte i;
   unsigned int ks=0;
   
   for(i=0; i<sizeof(Regs4); i++)    ks+=Regs4[i] = read_eeprom_uchar(i);  // первые 5 регистров

   // hopping channels
   for(i=0; i<HOPE_NUM; i++)  ks+=hop_list[i] = read_eeprom_uchar(i+11);
  
   // Регистры управления мощностью (19-23): канал, мошность1 - мощность3.
   for(i=0; i<sizeof(PowReg); i++)    ks+=PowReg[i] = read_eeprom_uchar(i+19);

   if(read_eeprom_uint(EEPROM_KS_ADR) != ks) return false;            // Checksum error

   return true;                                            // OK
} 

// Запись всех настроек
void write_eeprom(void)
{
   byte i;
   unsigned int ks=0;
   
   for(i=0; i<sizeof(Regs4); i++) {
     write_eeprom_uchar(i,Regs4[i]);  
     ks+=Regs4[i];  
   }

   // hopping channels
   for(i=0; i<HOPE_NUM; i++) {
     write_eeprom_uchar(i+11,hop_list[i]);   
     ks+=hop_list[i];
   }
  
   // Регистры управления мощностью (19-23): канал, мошность1 - мощность3.
   for(i=0; i<sizeof(PowReg); i++) {
     write_eeprom_uchar(i+19,PowReg[i]);  
     ks+=PowReg[i];  
   }
   write_eeprom_uint(EEPROM_KS_ADR,ks);        // Write checksum
} 

char etxt1[] PROGMEM = "FLASH ERROR!!! Can't work!";
char etxt2[] PROGMEM = "Error read settings!";

void eeprom_check(void)              // читаем и проверяем настройки из EEPROM, а также целостность программы
{

  if(flash_check()) {
      printlnPGM(etxt1);
      Red_LED_Blink(59999);  // долго мигаем красным, если КС не сошлась
  }    
  
   if(!read_eeprom()) {
        printlnPGM(etxt2);
        Red_LED_Blink(59999);  // мигаем красным, если КС не сошлась
   }
}  


