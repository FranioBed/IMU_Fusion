#include "prio_api.hpp"
#include <vector>
#include <algorithm>
#include <iostream>
#include <fstream>

#if !defined(_WIN32) && !defined(_WIN64)
    #include <stdio.h>
    #include <sys/time.h>

    #include <sys/types.h>
    #include <unistd.h>
#endif

using namespace std;

PrioAPI gPrioAPI;

#define PRIO_READER_THREAD_BUFF_SIZE 4096
#define PRIO_READER_DONGLE_PROCESS_LIMIT 4096
#define PRIO_READER_NUMBER_OF_RETERIES 100

U8 prioHandleStream(PrioHub* hub)
{
    U8 buff[PRIO_READER_THREAD_BUFF_SIZE];
    if (hub->_port->available() < (size_t)PRIO_READER_THREAD_BUFF_SIZE)
    {
        int read_data = hub->_port->read(&buff[0], hub->_port->available());
        hub->_parser_mutex.lock();
        for (int i = 0; i < read_data; i++)
        {
            hub->_unparsed_stream_data.push_back(buff[i]);
        }
        hub->_parser_mutex.unlock();
    }
    else
    {
        int read_data =  hub->_port->read(&buff[0], PRIO_READER_THREAD_BUFF_SIZE);
        hub->_parser_mutex.lock();
        for (int i = 0; i < read_data; i++)
        {
            hub->_unparsed_stream_data.push_back(buff[i]);
        }
        hub->_parser_mutex.unlock();
    }
    return 0;
}


U8 prioHandleStreamingData(PrioDevice* device)
{
    if (device->_type == PRIO_DEVICE_TYPE_HUB)
    {
        if (device->_port->available() != 0 )
            return prioHandleStream((PrioHub*)device);
    }
    else
    {
        shared_ptr<PrioHub> hub = ((PrioBaseStation*)device)->_children[0];
        if (hub.get()->_port->available() != 0)
        {
            return prioHandleStream(hub.get());
        }
    }
    return 0;
}

void prioReaderThread()
{
    while (1)
    {
        if (gPrioAPI._breakReadThread)
            break;

        gPrioAPI._portMutex.lock();

        if (gPrioAPI._readDevices.size() > 0)
        {
            //wait for any object to be ready
            WaitForMultiplePorts(gPrioAPI._readPorts);

            //walk through list of devices, pulling anything off their ports and putting it in their buffers
            for (vector<PrioDevice*>::iterator it = gPrioAPI._readDevices.begin(); it != gPrioAPI._readDevices.end(); ++it)
            {
                U8 tmp = prioHandleStreamingData(*it);
            }

            gPrioAPI._portMutex.unlock();
        }
        else
        {
            gPrioAPI._portMutex.unlock();
        }
    }
}

PrioAPI::PrioAPI()
{
    init();
}

PrioAPI::~PrioAPI()
{
    deinit();
}

void PrioAPI::init()
{

}

void PrioAPI::deinit()
{
    if (_readerThread.joinable())
    {
        _breakReadThread = true;
        _readerThread.join();
    }
}

void PrioAPI::registerStreamingDevice(PrioDevice* device)
{
    _portMutex.lock();
    _readPorts.push_back(device->_port);
    _readDevices.push_back(device);

    if (_readDevices.size() == 1 )
    {
        _breakReadThread = false;
        _readerThread = std::thread(prioReaderThread);
    }
    _portMutex.unlock();
}

void PrioAPI::unregisterStreamingDevice(PrioDevice* device)
{
    _portMutex.lock();
    _readPorts.erase(std::remove(_readPorts.begin(), _readPorts.end(), device->_port), _readPorts.end());
    _readDevices.erase(std::remove(_readDevices.begin(), _readDevices.end(), device), _readDevices.end());

    _portMutex.unlock();

    if (_readDevices.size() == 0)
    {
        _breakReadThread = true;

		if (_readerThread.joinable())
		{
			_readerThread.join();
		}
    }
}

void PrioAPI::unpauseStreamingDevice(PrioDevice* device)
{
    _portMutex.lock();
    _readPorts.push_back(device->_port);
    _readDevices.push_back(device);
    _portMutex.unlock();
}

void PrioAPI::pauseStreamingDevice(PrioDevice* device)
{
    _portMutex.lock();
    _readPorts.erase(std::remove(_readPorts.begin(), _readPorts.end(), device->_port), _readPorts.end());
    _readDevices.erase(std::remove(_readDevices.begin(), _readDevices.end(), device), _readDevices.end());
    _portMutex.unlock();
}

#define PRIO_VID 0x2476

vector<prio_ComPort> prioAPIFindPorts(U32 find_flags)
{
    vector<BasicPortInfo> in_ports = gPrioAPI._serialEnumerator.getPorts();

    vector<prio_ComPort> out_ports;

    for (auto& port : in_ports)
    {
        PRIO_TYPE dev_type = PRIO_UNKNOWN;

        if (port.vendor_id == PRIO_VID)
        {
            if (port.product_id == HUB_PID)
            {
                dev_type = PRIO_HUB;
            }
            else if (port.product_id == BS_PID)
            {
                dev_type = PRIO_BS;
            }
        }

        if( dev_type & find_flags )
        {
            prio_ComPort new_port;
            new_port.port_name = new char[64];

            memcpy(new_port.port_name, port.port_name.c_str(), 64);
            new_port.device_type = dev_type;
            out_ports.push_back(new_port);
        }
    }

    return out_ports;
}