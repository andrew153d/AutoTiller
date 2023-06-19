#ifndef UDPHANDLER_H
#define UDPHANDLER_H

#include <Arduino.h>
#include <esp_wifi.h>
#include <AsyncUdp.h>
#include <string.h>
#include "string.h"
#include <sstream>
#include <iostream>
#include <cstdint>

#define SEND_PORT 8888

class UDPHandler
{
private:
    AsyncUDP udp;
    int *time;
    int lastMsgTime;
    int senderPort;

public:
    bool isInitialized;

    UDPHandler(int i);
    void begin(int useless);


    void sendUDPevery(const char *data, int msec);

    void sendUDP(const char *data);
    void sendUDP(int num);
    void sendUDP(float num);
    void sendUDP(double num);
    void sendUDP(bool b);
    
    void print(const char str[]);
    void print(std::string str);
    void print(float num);

    void println(const char str[]);
    void println(String str = "");
    void println(float num);
};

extern UDPHandler UDP;

#endif