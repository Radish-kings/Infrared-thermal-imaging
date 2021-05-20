#include <Arduino.h>
#include <TFT_eSPI.h>
#include <Button2.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_AMG88xx.h>
#include "interpolation.h"
#include "WiFi.h"
#include <PubSubClient.h>
#include "Ticker.h"

const char *ssid = "Tenda_0F1588";  //WiFi 名
const char *password = "19034490232"; //WiFi密码
const char *mqtt_server = "183.230.40.39"; //ONENET 的IP地址

#define mqtt_devid "698269093" //设备ID
#define mqtt_pubid "410438" //产品ID
#define mqtt_password "123456"

WiFiClient espClient;
PubSubClient client(espClient); // 创建一个PubSub客户端, 传入创建的WIFI客户端

char msg_buf[200];                                //发送信息缓冲区
char dataTemplate[] = "{\"temp\":%.2f}"; //信息模板
char msgJson[75];                                 //要发送的json格式的数据
unsigned short json_len = 0;                      //json长度
Ticker tim1;                                      //定时器,用来循环上传数据

TFT_eSPI tft = TFT_eSPI();
Button2 btn1(35);
Button2 btn2(0);

/*256位的温度颜色对照表*/
const uint16_t camColors[] = {0x480F,
0x400F,0x400F,0x400F,0x4010,0x3810,0x3810,0x3810,0x3810,0x3010,0x3010,
0x3010,0x2810,0x2810,0x2810,0x2810,0x2010,0x2010,0x2010,0x1810,0x1810,
0x1811,0x1811,0x1011,0x1011,0x1011,0x0811,0x0811,0x0811,0x0011,0x0011,
0x0011,0x0011,0x0011,0x0031,0x0031,0x0051,0x0072,0x0072,0x0092,0x00B2,
0x00B2,0x00D2,0x00F2,0x00F2,0x0112,0x0132,0x0152,0x0152,0x0172,0x0192,
0x0192,0x01B2,0x01D2,0x01F3,0x01F3,0x0213,0x0233,0x0253,0x0253,0x0273,
0x0293,0x02B3,0x02D3,0x02D3,0x02F3,0x0313,0x0333,0x0333,0x0353,0x0373,
0x0394,0x03B4,0x03D4,0x03D4,0x03F4,0x0414,0x0434,0x0454,0x0474,0x0474,
0x0494,0x04B4,0x04D4,0x04F4,0x0514,0x0534,0x0534,0x0554,0x0554,0x0574,
0x0574,0x0573,0x0573,0x0573,0x0572,0x0572,0x0572,0x0571,0x0591,0x0591,
0x0590,0x0590,0x058F,0x058F,0x058F,0x058E,0x05AE,0x05AE,0x05AD,0x05AD,
0x05AD,0x05AC,0x05AC,0x05AB,0x05CB,0x05CB,0x05CA,0x05CA,0x05CA,0x05C9,
0x05C9,0x05C8,0x05E8,0x05E8,0x05E7,0x05E7,0x05E6,0x05E6,0x05E6,0x05E5,
0x05E5,0x0604,0x0604,0x0604,0x0603,0x0603,0x0602,0x0602,0x0601,0x0621,
0x0621,0x0620,0x0620,0x0620,0x0620,0x0E20,0x0E20,0x0E40,0x1640,0x1640,
0x1E40,0x1E40,0x2640,0x2640,0x2E40,0x2E60,0x3660,0x3660,0x3E60,0x3E60,
0x3E60,0x4660,0x4660,0x4E60,0x4E80,0x5680,0x5680,0x5E80,0x5E80,0x6680,
0x6680,0x6E80,0x6EA0,0x76A0,0x76A0,0x7EA0,0x7EA0,0x86A0,0x86A0,0x8EA0,
0x8EC0,0x96C0,0x96C0,0x9EC0,0x9EC0,0xA6C0,0xAEC0,0xAEC0,0xB6E0,0xB6E0,
0xBEE0,0xBEE0,0xC6E0,0xC6E0,0xCEE0,0xCEE0,0xD6E0,0xD700,0xDF00,0xDEE0,
0xDEC0,0xDEA0,0xDE80,0xDE80,0xE660,0xE640,0xE620,0xE600,0xE5E0,0xE5C0,
0xE5A0,0xE580,0xE560,0xE540,0xE520,0xE500,0xE4E0,0xE4C0,0xE4A0,0xE480,
0xE460,0xEC40,0xEC20,0xEC00,0xEBE0,0xEBC0,0xEBA0,0xEB80,0xEB60,0xEB40,
0xEB20,0xEB00,0xEAE0,0xEAC0,0xEAA0,0xEA80,0xEA60,0xEA40,0xF220,0xF200,
0xF1E0,0xF1C0,0xF1A0,0xF180,0xF160,0xF140,0xF100,0xF0E0,0xF0C0,0xF0A0,
0xF080,0xF060,0xF040,0xF020,0xF800,};

//low range of the sensor (this will be blue on the screen)
#define MINTEMP 14
//high range of the sensor (this will be red on the screen)
#define MAXTEMP 30

/*32*32为优化后的分辨率，不宜太高，提升分辨率修改此处即可*/
#define INTERPOLATED_COLS 32
#define INTERPOLATED_ROWS 32
#define AMG_COLS 8
#define AMG_ROWS 8

Adafruit_AMG88xx amg;
unsigned long delayTime;
float pixels[AMG_COLS * AMG_ROWS];
uint16_t displayPixelWidth, displayPixelHeight;
uint16_t displayPixelX, displayPixelY;

/* 画温度颜色对比条 */
float drawTemperateBar () {
  const int colorSize = 256;
  const int barHeight = 128;
  const int barWidth = 10;
  const int posY = (tft.height()-barHeight)/2-1;
  for(int i=0; i<colorSize; i++) {
    int x = i/barHeight;
    tft.drawLine(14+x*barWidth, barHeight-i%barHeight+posY, 14+x*barWidth+barWidth, barHeight-i%barHeight+posY, camColors[i]);
  }

  tft.setTextSize(1);

  tft.setTextDatum(BL_DATUM);
  tft.setTextColor(camColors[0]);
  tft.drawString(String(MINTEMP), 0, tft.height());

  tft.setTextColor(camColors[barHeight]);
  tft.drawString(String(MINTEMP+int(MAXTEMP-MINTEMP)/2), 38, tft.height());

  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(camColors[barHeight-1]);
  tft.drawString(String(MINTEMP+int(MAXTEMP-MINTEMP)/2), 0, 0);

  tft.setTextColor(camColors[colorSize-1]);
  tft.drawString(String(MAXTEMP), 38, 0);
}

/* 画热成像素点 */
void drawpixels(float *p, uint8_t rows, uint8_t cols, uint8_t boxWidth, uint8_t boxHeight, boolean showVal) {
  int colorTemp;
  float curMax = get_point(p, rows, cols, 0, 0), 
    curMin = get_point(p, rows, cols, 0, 0), 
    curMid = get_point(p, rows, cols, cols/2, rows/2);
  uint16_t colorMax = camColors[0], 
    colorMin = camColors[0],
    colorMid = camColors[0]; 

  //画热成像
  for (int y=0; y<rows; y++) {
    for (int x=0; x<cols; x++) {
      float val = get_point(p, rows, cols, x, y);

      if(val >= MAXTEMP) 
        colorTemp = MAXTEMP;
      else if(val <= MINTEMP) 
        colorTemp = MINTEMP;
      else 
        colorTemp = val;
      
      uint8_t colorIndex = map(colorTemp, MINTEMP, MAXTEMP, 0, 255);
      colorIndex = constrain(colorIndex, 0, 255);
      //draw the pixels!
      tft.fillRect(displayPixelX + boxWidth * x, displayPixelY + boxHeight * y, boxWidth, boxHeight, camColors[colorIndex]);
      
      //取最大值
      if(val >= curMax){    
        curMax = val;
        colorMax = camColors[colorIndex];
      }          
      //取最小值
      if(val <= curMin){
        curMin = val;
        colorMin = camColors[colorIndex];
      }
      //取中心点
      if(y == rows/2 && x == cols/2){
        colorMid = camColors[colorIndex];
      }

      if (showVal) {
        tft.setCursor(boxWidth * y + boxWidth/2 - 12, 40 + boxHeight * x + boxHeight/2 - 4);
        tft.setTextSize(1);
        tft.setTextColor(TFT_WHITE);
        tft.print(val,1);
      }
    } 
  }
  //画中心点十字
  uint8_t midX = displayPixelX+cols*boxWidth/2;
  uint8_t midY = displayPixelY+rows*boxHeight/2;
  tft.drawLine(midX-10, midY, midX+10, midY, TFT_WHITE);
  tft.drawLine(midX, midY-10, midX, midY+10, TFT_WHITE);
  
  char temp[7];
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE);
  //显示最高温度
  tft.setTextDatum(TL_DATUM);
  tft.drawString(F("MAX"), tft.width()-40, 0);    
  tft.fillRect(tft.width()-45, 10, 40, 30, colorMax);  
  tft.setTextDatum(CL_DATUM);
  memset(temp, 0, sizeof(temp));
  //curMax为实时数值
  sprintf(temp, "%0.1f", curMax+5.5);
  tft.drawString(String(temp), tft.width()-40, 25);
  //显示最低温度
  tft.setTextDatum(TL_DATUM);
  tft.drawString(F("MIN"), tft.width()-40, 47);    
  tft.fillRect(tft.width()-45, 57, 40, 30, colorMin); 
  tft.setTextDatum(CL_DATUM);     
  memset(temp, 0, sizeof(temp));
  sprintf(temp, "%0.1f", curMin);
  tft.drawString(String(temp), tft.width()-40, 72);
  //显示中心点温度
  tft.setTextDatum(TL_DATUM);
  tft.drawString(F("POINT"), tft.width()-40, 94);    
  tft.fillRect(tft.width()-45, 104, 40, 30, colorMid); 
  tft.setTextDatum(CL_DATUM);
  memset(temp, 0, sizeof(temp));
  sprintf(temp, "%0.1f", curMid);
  tft.drawString(String(temp), tft.width()-40, 119);
}


//连接WIFI相关函数
void setupWifi()
{
  delay(10);
  Serial.println("连接WIFI");
  WiFi.begin(ssid, password);
  while (!WiFi.isConnected())
  {
    Serial.print(".");
    delay(500);
  }
  Serial.println("OK");
  Serial.println("Wifi连接成功");
}


//收到主题下发的回调, 注意这个回调要实现三个形参 1:topic 主题, 2: payload: 传递过来的信息 3: length: 长度
void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.println("message rev:");
  Serial.println(topic);
  for (size_t i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}


float Read_data()
{
  float temp;
  temp = amg.readThermistor();
  return temp;
}


//向主题发送模拟的温湿度数据
void sendTempAndHumi()
{
  float j;
  j = Read_data();
  if (client.connected())
  {
    //float curMax = get_point(p, rows, cols, 0, 0);
    snprintf(msgJson, 40, dataTemplate, j+5.5); //将模拟温湿度数据套入dataTemplate模板中, 生成的字符串传给msgJson
    json_len = strlen(msgJson);                   //msgJson的长度
    msg_buf[0] = char(0x03);                       //要发送的数据必须按照ONENET的要求发送, 根据要求,数据第一位是3
    msg_buf[1] = char(json_len >> 8);              //数据第二位是要发送的数据长度的高八位
    msg_buf[2] = char(json_len & 0xff);            //数据第三位是要发送数据的长度的低八位
    memcpy(msg_buf + 3, msgJson, strlen(msgJson)); //从msg_buf的第四位开始,放入要传的数据msgJson
    msg_buf[3 + strlen(msgJson)] = 0;              //添加一个0作为最后一位, 这样要发送的msg_buf准备好了
    Serial.print("public message:");
    Serial.println(msgJson);
    client.publish("$dp", (uint8_t *)msg_buf, 3 + strlen(msgJson)); //发送数据到主题$dp
  }
}
 
//重连函数, 如果客户端断线,可以通过此函数重连
void clientReconnect()
{
  while (!client.connected()) //再重连客户端
  {
    Serial.println("reconnect MQTT...");
    if (client.connect(mqtt_devid, mqtt_pubid, mqtt_password))
    {
      Serial.println("connected");
    }
    else
    {
      Serial.println("failed");
      Serial.println(client.state());
      Serial.println("try again in 5 sec");
      delay(5000);
    }
  }
}

void setup() {

  Serial.begin(115200);                                  //初始化串口
  delay(3000);                                           //这个延时是为了让我打开串口助手
  setupWifi();                                           //调用函数连接WIFI
  client.setServer(mqtt_server, 6002);                   //设置客户端连接的服务器,连接Onenet服务器, 使用6002端口
  client.connect(mqtt_devid, mqtt_pubid, mqtt_password); //客户端连接到指定的产品的指定设备.同时输入鉴权信息
  client.setCallback(callback);                          //设置好客户端收到信息是的回调
  tim1.attach(20, sendTempAndHumi);                      //定时每20秒调用一次发送数据函数sendTempAndHumi


  Serial.begin(9600); 

  //初始化屏幕
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(F("Thermal Camera"), tft.width()/2, tft.height()/2);
  delay(2000);

  tft.fillScreen(TFT_BLACK);  
  displayPixelWidth = tft.height() / 8;
  displayPixelHeight = tft.height() / 8;
  displayPixelX = (tft.width() - displayPixelWidth*8)/2;
  displayPixelY = (tft.height() - displayPixelHeight*8)/2;

  drawTemperateBar();

  bool status;
    
  // default settings
  status = amg.begin();
  if (!status) {
      Serial.println("Could not find a valid AMG8833 sensor, check wiring!");
      while (1);
  }
  
  Serial.println("-- Thermal Camera Test --");
  delay(100); // let sensor boot up



}

void loop() {

  if (!WiFi.isConnected()) //先看WIFI是否还在连接
  {
    setupWifi();
  }
  if (!client.connected()) //如果客户端没连接ONENET, 重新连接
  {
    clientReconnect();
  }
  client.loop(); //客户端循环检测

  //read all the pixels
  amg.readPixels(pixels);

  float dest_2d[INTERPOLATED_ROWS * INTERPOLATED_COLS];

  interpolate_image(pixels, AMG_ROWS, AMG_COLS, dest_2d, INTERPOLATED_ROWS, INTERPOLATED_COLS);

  uint16_t boxsize = min(tft.width() / INTERPOLATED_COLS, tft.height() / INTERPOLATED_COLS);
  drawpixels(dest_2d, INTERPOLATED_ROWS, INTERPOLATED_COLS, boxsize, boxsize, false);
  //delay(100);



}
