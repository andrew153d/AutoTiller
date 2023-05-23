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

#define PORT 61912

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
    void init(int *_time);
    void sendUDPevery(const char *data, int msec);
    void sendUDP(const char *data);
    void sendUDP(int *data);

    void print(const char str[]);
    void print(std::string str);
    void println(const char str[]);
    void println(String str);
};

extern UDPHandler UDP;

#endif