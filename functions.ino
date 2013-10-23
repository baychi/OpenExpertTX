// **********************************************************
// Baychi soft 2013
// **      RFM22B/23BP/Si4432 Transmitter with Expert protocol **
// **      This Source code licensed under GPL            **
// **********************************************************
// Latest Code Update : 2013-10-22
// Supported Hardware : Expert Tiny, Orange/OpenLRS Tx/Rx boards (store.flytron.com)
// Project page       : https://github.com/baychi/OpenExpertTX
// **********************************************************

void Red_LED_Blink(unsigned short blink_count)  // на самом деле индикатор у нас только один :)
{
  word i;
  for (i=0;i<blink_count;i++)     {
     Sleep(125);
     Green_LED_ON;
     Sleep(125);
     Green_LED_OFF;
     if(blink_count > 50) {
       if(checkMenu()) {
         doMenu();
         break;
       }
     }  
  }
}


// Вычисление CRC8 по массиву данных
//
unsigned char CRC8(unsigned char buf[], unsigned char len)
{
   unsigned char i,j,crc=0;
    
   for(i=0; i<len; i++) {
     crc = crc ^ buf[i];
     for(j=0; j<8; j++) {
       if (crc & 0x01) crc = (crc >> 1) ^ 0x8C;
       else crc >>= 1;
     }
   }

   return crc;
}  

// Пауза с отдачей квантов
void Sleep(word ms) 
{
  unsigned long t=millis()+ms;
  
  while(millis() < t) ppmLoop();
}
