#include "UDPHandler.h"

float bufToFloat(uint8_t *buf, int len)
{

  std::string s(reinterpret_cast<char *>(buf), len);
  float result;
  std::stringstream ss(s);
  ss >> result;
  if (ss.fail())
  {
  }
  return result;
}

void UDPHandler::init(int *_time)
{
  time = _time;
  lastMsgTime = 0;

  if (udp.listen(1234))
  {
    udp.onPacket([&](AsyncUDPPacket packet)
                 {
                    if(packet.remotePort() != senderPort){
                      senderPort = packet.remotePort();
                    }
                    uint8_t buf[30];

                    memcpy(&buf[0], packet.data(), packet.length());

                    uint8_t firstByte = buf[0];

                    switch(firstByte){
                      case 0x6d: //disable motors
                      
                        break;
                      case 0x72: //restart
                        ESP.restart();
                        break;
                      case 0x77: //disable wireless
                        packet.printf("Shutting Down Wireless");
                        //ArduinoOTA.end();
                        //WiFi.disconnect(true);
                        //WiFi.mode(WIFI_OFF);
                        esp_wifi_disconnect();           // break connection to AP
                        esp_wifi_stop();                 // shut down the wifi radio
                        esp_wifi_deinit();               // release wifi resources
                        vTaskDelete(NULL);
                        break;
                      case 0x54: //T set target
                        packet.printf("Sending to %u",10);
                        break;
                      
                      case 0x50: //P
                      break;
                      case 0x49: //I
                      break;
                      case 0x44: //D
                      break;
                      case 0x41: //A
                      break;
                      case 0x56: //V
                        
                        break;
                    }
                    packet.printf("Got %u bytes of data: %x", packet.length(), *buf); });
  }
}

void UDPHandler::sendUDPevery(const char *data, int msec)
{
  if ((*time - lastMsgTime) < msec)
  {
    return;
  }
  lastMsgTime = *time;
  udp.broadcastTo(data, senderPort);
  //udp.broadcast(data);
}

void UDPHandler::sendUDP(const char *data)
{
  udp.broadcastTo(data, senderPort);
}

void UDPHandler::sendUDP(int *data)
{
  char buf[10];
  int out = *data;
  sprintf(buf, "%d", (int)*data);
  udp.broadcastTo(buf, senderPort);
}

UDPHandler::UDPHandler(int i)
{
  lastMsgTime = 0;
  isInitialized = true;
  senderPort = PORT;
}

void UDPHandler::print(const char str[])
{
  sendUDP(str);
}
void UDPHandler::println(const char str[])
{
  sendUDP(str);
}

void UDPHandler::print(std::string str)
{
  sendUDP(str.c_str());
}

void UDPHandler::println(String str)
{
  sendUDP(str.c_str());
}

UDPHandler UDP(1);
