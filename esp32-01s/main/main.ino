#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>

const char *ssid = "Xiaomi_E8E3"; // WiFi网络名称
const char *password = "z110110110"; // WiFi网络密码
const int tcpPort = 800; // TCP监听端口
const int udpPort = 1234; // UDP目标端口
unsigned char rgb_flag;
WiFiServer server(tcpPort); // TCP服务器
WiFiUDP udp; // UDP客户端
unsigned long lastUDPSendTime = 0;
const unsigned long udpInterval = 1000;
int shaft_velocity;
String receivedText;

void setup() {
  Serial.begin(115200); // 初始化串口通信，设置波特率为115200
  delay(10);
  // 连接到WiFi网络
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // 开始TCP服务
  server.begin();

  // 开始MDNS服务
  MDNS.begin("esp8266");

  // 开始UDP客户端
  udp.begin(udpPort);
    //RGB
  strip.begin();                    // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();                     // Turn OFF all pixels ASAP
  strip.setBrightness(brightness);  // Set BRIGHTNESS to about 1/5 (max = 255)
  colorWipe_delay(strip.Color(255, 106, 106), 50);
  colorWipe_delay(strip.Color(0, 255, 255), 50);
  colorWipe_delay(strip.Color(148, 0, 211), 50);
}

void loop() {
  receivedText = ""; // 清空接收文本

  // 处理TCP连接
  WiFiClient client = server.available();
  if (client) {
    Serial.println("New TCP client connected");
    while (client.connected()) {
      if (client.available()) {
        String line = client.readStringUntil('\r');
        Serial.println("Received from TCP client: " + line);
        
        // 这里你可以处理TCP接收到的数据

        client.flush(); // 清空接收缓冲区
      }
    }
    Serial.println("TCP client disconnected");
  }

  // 检查串口是否有数据可读取
  while (Serial.available()) {
    char receivedChar = Serial.read();
    receivedText += receivedChar;
    delay(5);
  }

  // 输出接收到的完整文本
  if (receivedText != "") {
    Serial.print("Received text: ");
    Serial.println(receivedText);
      // 检查是否需要发送UDP数据包
    if (millis() - lastUDPSendTime >= udpInterval) {
      sendUDP(receivedText); //发送数据
      Serial.print("UDP text: ");
      Serial.println(receivedText);
      lastUDPSendTime = millis();
    }
  }
  //更新RGB效果
  unsigned long currentMillis = millis();
  if (currentMillis - pixelPrevious >= pixelInterval) {  //  Check for expired time
    pixelPrevious = currentMillis;                       //  Run current frame
    switch (rgb_flag) {
      case 0:
        rgb_off();
        break;
      case 1:
        if (shaft_velocity > 0) {
          pixelInterval = 150 - shaft_velocity;
          strip2();
        } else {
          pixelInterval = 150 + shaft_velocity;
          strip3();
        }
        break;
      case 2:
        pixelInterval = 100;
        strip2();
        break;
      case 3:
        pixelInterval = 100;
        strip3();
        break;
      case 4:
        strip1();
        break;
      case 5:
        rainbow1();
        break;
      case 6:
        rainbow2();
        break;
      case 7:
        pulse_rainbow1();
        break;
    }
  }

void sendUDP(const String &message) {
  udp.beginPacket(IPAddress(192, 168, 31, 167), udpPort); // 设置目标IP地址和端口
  udp.print(message); // 发送消息
  udp.endPacket(); // 结束数据包
}
