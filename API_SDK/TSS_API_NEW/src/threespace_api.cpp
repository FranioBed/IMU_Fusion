#include "threespace_api.hpp"
#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;

TssAPI gAPI;

#define TSS_READER_THREAD_BUFF_SIZE 4096
#define TSS_READER_DONGLE_PROCESS_LIMIT 4096

void tssHandleStreamingDataSensor(TssSensor* sensor)
{
    U32 total_packet_size = sensor->_streamDataSize + sensor->_responseHeaderSize;

    U32 bytes_to_read = sensor->_port->available() / total_packet_size;

    if (bytes_to_read > 0)
    {
        bytes_to_read *= total_packet_size;

        while (bytes_to_read > TSS_READER_THREAD_BUFF_SIZE)
            bytes_to_read -= total_packet_size;

        U8 buff[TSS_READER_THREAD_BUFF_SIZE];
        unsigned long read_bytes;
        read_bytes = sensor->_port->read(buff, bytes_to_read);

        U32 read_index = 0;
        while (read_index < bytes_to_read)
        {
            TssHeader header;
            _tssParseResponseHeader(buff + read_index, sensor->_responseHeaderFlags, &header);
            read_index += sensor->_responseHeaderSize;

            sensor->_streamBuffer.addPacket(&header, buff + read_index, sensor->_streamDataSize);

            read_index += sensor->_streamDataSize;
        }
    }
}

void tssHandleStreamingDataDongle(TssDongle* dongle)
{
    int max_bytes_to_read = dongle->_port->available(); //hope to god this is a complete packet for the time being, read as though it is
    if (max_bytes_to_read > TSS_READER_DONGLE_PROCESS_LIMIT)
        max_bytes_to_read = TSS_READER_DONGLE_PROCESS_LIMIT;

    U8 buff[TSS_READER_THREAD_BUFF_SIZE];

    while (max_bytes_to_read > 0)
    {
        unsigned long read_bytes;
        read_bytes = dongle->_port->read(buff, dongle->_responseHeaderWirelessSize);

        TssHeader header;
        _tssParseResponseHeader(buff, dongle->_responseHeaderWirelessFlags, &header);
        max_bytes_to_read -= dongle->_responseHeaderWirelessSize;

        shared_ptr<TssSensor> child;

        //if the logical id is present, we can use that to find the appropriate sensor...
        if (header.LogicalId < TSS_DONGLE_NUM_CHILDREN)
        {
            child = dongle->_children[header.LogicalId];
        }
        else
        {
            //no logical id?  bail
            break;
        }

        if (child)
        {
            read_bytes = dongle->_port->read(buff, child->_streamDataSize);

            child->_streamBuffer.addPacket(&header, buff, child->_streamDataSize);
            max_bytes_to_read -= child->_streamDataSize;
        }
        else
        {
            //uh...didn't find a child?  bail
            break;
        }

    }
}

void tssHandleStreamingData(TssDevice* device)
{
    if (device->_type == TSS_DEVICE_TYPE_SENSOR)
    {
        tssHandleStreamingDataSensor((TssSensor*)device);
    }
    else
    {
        tssHandleStreamingDataDongle((TssDongle*)device);
    }
}

void tssReaderThread()
{
    while (1)
    {
        if (gAPI._breakReadThread)
            break;

        gAPI._readerThreadMutex.lock();

        if (gAPI._readDevices.size() > 0)
        {
            //wait for any object to be ready
            //there is no linux/mac equivalent of this implemented here yet
#if defined(_WIN64) || defined(_WIN32)
            WaitForMultiplePorts(gAPI._readPorts);
#endif
            //walk through list of devices, pulling anything off their ports and putting it in their buffers
            for (vector<TssDevice*>::iterator it = gAPI._readDevices.begin(); it != gAPI._readDevices.end(); ++it)
            {
                tssHandleStreamingData(*it);
            }
        }

        gAPI._readerThreadMutex.unlock();
    }
}

TssAPI::TssAPI()
{
    init();
}

TssAPI::~TssAPI()
{
    deinit();
}

void TssAPI::init()
{
}

void TssAPI::deinit()
{
    if (_readerThread.joinable())
    {
        _breakReadThread = true;
        _readerThread.join();
    }
}

void TssAPI::registerStreamingDevice(TssDevice* device)
{
    _readerThreadMutex.lock();
    _readPorts.push_back(device->_port);
    _readDevices.push_back(device);

    if (_readDevices.size() == 1)
    {
        _breakReadThread = false;
        _readerThread = std::thread(tssReaderThread);
    }

    _readerThreadMutex.unlock();
}

void TssAPI::unregisterStreamingDevice(TssDevice* device)
{
    _readerThreadMutex.lock();
    _readPorts.erase(std::remove(_readPorts.begin(), _readPorts.end(), device->_port), _readPorts.end());
    _readDevices.erase(std::remove(_readDevices.begin(), _readDevices.end(), device), _readDevices.end());

    U32 size = _readDevices.size();

    _readerThreadMutex.unlock();

    if (size == 0)
    {
        _breakReadThread = true;
        _readerThread.join();
    }
}

#define TSS_VID 0x2476

vector<TssComPort> tssFindSensorPorts(U32 find_flags)
{
    vector<BasicPortInfo> in_ports = gAPI._serialEnumerator.getPorts();

    vector<TssComPort> out_ports;

    for (auto& port : in_ports)
    {
        TSS_TYPE dev_type = TSS_UNKNOWN;

        if (port.vendor_id == TSS_VID)
        {
            if (port.product_id == DNG_PID)
            {
                dev_type = TSS_DNG;
            }
            else if (port.product_id == WL_PID)
            {
                dev_type = TSS_WL;
            }
            else if (port.product_id == USB_PID)
            {
                dev_type = TSS_USB;
            }
            else if (port.product_id == EM_PID)
            {
                dev_type = TSS_EM;
            }
            else if (port.product_id == BT_PID)
            {
                dev_type = TSS_BT;
            }
            else if (port.product_id == DL_PID)
            {
                dev_type = TSS_DL;
            }
            else if (port.product_id == BTL_PID)
            {
                dev_type = TSS_BTL;
            }
        }

        if (dev_type & find_flags)
        {
            TssComPort new_port;
            new_port.port_name = port.port_name;
            new_port.device_type = dev_type;
            out_ports.push_back(new_port);
        }
    }

    return out_ports;
}