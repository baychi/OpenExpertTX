// **********************************************************
// **                   OpenLRS Functions                  **
// **        Developed by Melih Karakelle on 2010-2011     **
// **          This Source code licensed under GPL         **
// **********************************************************
// Latest Code Update : 2013-08-17
// Supported Hardware : Open Tiny LRS Rx boards (store.flytron.com)
// Project Forum      : http://forum.flytron.com/viewforum.php?f=7
// Google Code Page   : http://code.google.com/p/openlrs/



void Red_LED_Blink(unsigned short blink_count)
{
  unsigned char i;
  for (i=0;i<blink_count;i++)     {
     Sleep(125);
     Red_LED_ON;
     Sleep(125);
     Red_LED_OFF;
  }
}


// Преобразование данных входного буфера buf в длительности PWM
/**********************
void Buf_To_Servo(unsigned char buf[])
{
     unsigned int i;
     int temp_int;

     for(i = 0; i<RC_CHANNEL_COUNT; i++) { // Write into the Servo Buffer
        temp_int = 4 * buf[i+2];
        if(i<8) {                   // 2-й байт пакета содержит старшие биты 8-ми первых каналов 
           if(buf[1]&(1<<i)) temp_int +=1024; 
         } else temp_int +=temp_int;
         Servo_Buffer[i] = temp_int+1990; // кодируем PWM в мкс*2
      }
}  
*******************************/

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
