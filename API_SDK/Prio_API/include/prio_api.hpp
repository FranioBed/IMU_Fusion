#pragma once

#include "prio_device.hpp"
#include "prio_hub.hpp"
#include "prio_basestation.hpp"

#include <mutex>
#include <thread>

#include "serial_enumerator.hpp"

class PrioAPI
{
public:
    PrioAPI();
    ~PrioAPI();
    void init();
    void deinit();
    void registerStreamingDevice(PrioDevice* device);
    void unregisterStreamingDevice(PrioDevice* device);

    void unpauseStreamingDevice(PrioDevice* device);
    void pauseStreamingDevice(PrioDevice* device);

    std::mutex _portMutex;
    std::thread _readerThread;
    vector<shared_ptr<Serial>> _readPorts;
    vector<PrioDevice*> _readDevices;
    bool _breakReadThread;
    SerialEnumerator _serialEnumerator;
    std::mutex _commandThreadMutex;
};

extern PrioAPI gPrioAPI;

vector<prio_ComPort> prioAPIFindPorts(U32 find_flags = 0xffffffff);