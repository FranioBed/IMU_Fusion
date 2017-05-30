#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <memory>
#include <windows.h>
#include <chrono>
#include <thread>
#include <vector>
#include "prio_api_export.h"
#include "prio_api.hpp"
#include "serial.h"
#include <fstream>
#include <iostream>

int threadBroke = 0;
shared_ptr<Serial> port;
std::mutex _portThreadMutex;

void ComPortThread()
{
    printf("Making new thread!\n");
    Timeout t;
    t.inter_byte_timeout = 100;
    t.read_timeout_multiplier = 100;
    t.read_timeout_constant = 100;
    t.write_timeout_multiplier = 3;
    t.write_timeout_constant = 2;

    _portThreadMutex.lock();
    threadBroke = 0;
    try
    {
        port = shared_ptr<Serial>(new Serial("COM57", 115200, t));
    }
    catch (IOException&)
    {
        port.reset();
        printf("Failed COMPORT CREATE IO ERROR\n");
        threadBroke = 1;
        _portThreadMutex.unlock();
        return;
    }

    if (!port->isOpen())
    {
        port.reset();
        printf("Failed COMPORT CREATE PORT NOT OPEN\n");
        threadBroke = 1;
        _portThreadMutex.unlock();
        return;
    }

    _portThreadMutex.unlock();

    while (1)
    {
        _portThreadMutex.lock();
        if (threadBroke == 1)
        {
            printf("Thread closing!\n");
            port->close();
            port.reset();
            _portThreadMutex.unlock();
            break;
        }
        _portThreadMutex.unlock();
        yost::sleep_ms(0);
    }

}

void com_test()
{
    std::thread comThread;
    comThread = std::thread(ComPortThread);

    U8 buff[256];
    U8 index = 0;
    U8 checksum = 0;

    buff[0] = 0xf7;

    index = 1;
    buff[1] = 223;
    index++;
    checksum += 223;

    buff[index] = checksum;
    index++;

    char* _versionString;
    while (1)
    {
        if (threadBroke == 1)
        {
            if (comThread.joinable())
                comThread.join();
            try
            {
                printf("Creating a new thread\n");
                comThread = std::thread(ComPortThread);
            }
            catch (...)
            {
                printf("Error in thread creation!\n");
                getchar();
                continue;
            }
            if (threadBroke == 1)
            {
                printf("Error in thread creation!\n");
                getchar();
                continue;
            }
            printf("Made new thread!\n");
        }
        printf("I have a thread!\n");
        _portThreadMutex.lock();
        try
        {
            port->write(buff, index);
        }
        catch (...)
        {
            printf("Error caught in write!\n");
            threadBroke = 1;
        }

        U8 buff[33] = { 'n', 'n', 'n', 'n', 'n', 'n', 'n', 'n', 'n', 'n', 'n', 'n', 'n', 'n', 'n', 'n', 'n', 'n', 'n', 'n', 'n', 'n', 'n', 'n', 'n', 'n', 'n', 'n', 'n', 'n' };
        try
        {
            port->read(buff, 32);
        }
        catch (...)
        {
            printf("Error caught in read!\n");
            threadBroke = 1;
        }
        buff[32] = '\0';

        _versionString = (char*)buff;

        printf("Version string: %s\n", _versionString);
        port->flush();
        _versionString = "";
        _portThreadMutex.unlock();
        getchar();
    }
}

int main()
{
    com_test();
    system("pause");
    return 0;
}