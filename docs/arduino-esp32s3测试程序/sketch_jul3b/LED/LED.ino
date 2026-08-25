#define RGB_LED_Pin 38                                         // RGB_LED 引脚

void RGD_LED(int round)
{
  // 红、绿、蓝、白灯交替闪烁，周期1秒
 for(int i = 0; i < round; i++)
 {
 neopixelWrite(RGB_LED_Pin, 128,0,0);                        // Red 
 delay(200);
 neopixelWrite(RGB_LED_Pin, 0,128,0);                        // Green
 delay(200);
 neopixelWrite(RGB_LED_Pin, 0,0,128);                        // Blue
 delay(200);
 neopixelWrite(RGB_LED_Pin, 2,2,2);                          // white
 delay(200);
 neopixelWrite(RGB_LED_Pin, 0,0,0);                          // Off / black
 delay(200);
 }
}

void setup() {
 pinMode(RGB_LED_Pin, OUTPUT);                                // 初始化 RGD_LED 
}

void loop() {
 RGD_LED(1);
 delay(10);
}
