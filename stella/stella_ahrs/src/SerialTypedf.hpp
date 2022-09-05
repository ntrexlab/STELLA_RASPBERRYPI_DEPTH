#pragma once

#define BIN_PACKET_LEN		13

struct SerialMessage
{
    int length;
    long ID;
    char data[8];
};


struct Usb2Packet
{
    double time_recv;
    union 
    {
        SerialMessage msg;
    };
    
};


