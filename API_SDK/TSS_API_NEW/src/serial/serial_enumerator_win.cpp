/********************************************//**
 * Copyright 1998-2014, YEI Corporation.
 * This Source Code Form is subject to the terms of the YEI 3-Space Open
 * Source License available online at:
 * http://www.yeitechnology.com/yei-3-space-open-source-license
 ***********************************************/

#ifdef _WIN32
#define USE_BTHAPI  !defined(__GNUC__)

#include "serial_enumerator.hpp"

#include <objbase.h>
#include <initguid.h>
#include <setupapi.h>
#include <stddef.h>

#include <algorithm>
#include <stdlib.h>

#if USE_BTHAPI
#include <BluetoothAPIs.h>
#endif // USE_WIN_BTH


/********************************************//**
 * A list of GUID types to check.
 ***********************************************/
const GUID g_device_class_guids[] =
{
    // Ports (COM & LPT ports), Class = Ports
    {0x4D36E978, 0xE325, 0x11CE, {0xBF, 0xC1, 0x08, 0x00, 0x2B, 0xE1, 0x03, 0x18}},
    // Bluetooth Devices, Class = Bluetooth
    {0xE0CBF06C, 0xCD8B, 0x4647, {0xBB, 0x8A, 0x26, 0x3B, 0x43, 0xF0, 0xF9, 0x74}},
    // COM port device interface class
    {0x86E0D1E0, 0x8089, 0x11D0, {0x9C, 0xE4, 0x08, 0x00, 0x3E, 0x30, 0x1F, 0x73}},
    // For use with com0com virtual ports
    {0xDF799E12, 0x3C56, 0x421B, {0xB2, 0x98, 0xB6, 0xD3, 0x64, 0x2B, 0xC8, 0x78}},
    // Eltima Virtual Serial Port Driver v4 GUID
    {0xCC0EF009, 0xB820, 0x42F4, {0x95, 0xA9, 0x9B, 0xFA, 0x6A, 0x5A, 0xB7, 0xAB}},
    // Advanced Virtual COM Port GUID
    {0x9341CD95, 0x4371, 0x4A37, {0xA5, 0xAF, 0xFD, 0xB0, 0xA9, 0xD1, 0x96, 0x31}}
};


/********************************************//**
 * Gets the property value string of the specified property from the registry Handle key.
 ***********************************************/
static std::string getRegKeyValue(HKEY key, LPCSTR property)
{
    DWORD buff_size = 0;
    DWORD type;
    ::RegQueryValueExA(key, property, NULL, NULL, NULL, &buff_size);
    BYTE *buff = new BYTE[buff_size];
    std::string result;
    if (::RegQueryValueExA(key, property, NULL, &type, buff, &buff_size) == ERROR_SUCCESS)
    {
        result = std::string((const char*)buff);
    }
    ::RegCloseKey(key);
    delete [] buff;
    return result;
}


/********************************************//**
 * Gets the registry property string of the specific property from the device registry.
 ***********************************************/
static std::string getDeviceProperty(HDEVINFO dev_info, PSP_DEVINFO_DATA dev_data, DWORD property)
{
    DWORD buff_size = 0;
    ::SetupDiGetDeviceRegistryPropertyA(dev_info, dev_data, property, NULL, NULL, 0, &buff_size);
    BYTE *buff = new BYTE[buff_size];
    ::SetupDiGetDeviceRegistryPropertyA(dev_info, dev_data, property, NULL, buff, buff_size, NULL);
    std::string result((const char*)buff);
    delete [] buff;
    return result;
}


/********************************************//**
 * Gets the device's path string of the specific interface detail from the device registry.
 ***********************************************/
static std::string getDeviceInterfaceDetail(HDEVINFO dev_info, PSP_DEVINFO_DATA dev_data, const GUID *guid, PSP_DEVICE_INTERFACE_DATA dev_intf)
{
    ::SetupDiCreateDeviceInterfaceA(dev_info, dev_data, guid, NULL, 0, dev_intf);
    DWORD buff_size = 0;
    ::SetupDiGetDeviceInterfaceDetailA(dev_info, dev_intf, NULL, 0, &buff_size, NULL);
    // Allocate for the size of the information of the device
    PSP_DEVICE_INTERFACE_DETAIL_DATA_A dev_detail_ptr = reinterpret_cast<PSP_DEVICE_INTERFACE_DETAIL_DATA_A>(::GlobalAlloc(GPTR, buff_size));
    dev_detail_ptr->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_A);
    ::SetupDiGetDeviceInterfaceDetailA(dev_info, dev_intf, dev_detail_ptr, buff_size, NULL, NULL);
    std::string result((const char*)dev_detail_ptr->DevicePath);
    ::GlobalFree(reinterpret_cast<HGLOBAL>(dev_detail_ptr));
    return result;
}


/********************************************//**
 * Gets detail information about the device.
 ***********************************************/
static void getDeviceDetails(BasicPortInfo *port_info, HDEVINFO dev_info, PSP_DEVINFO_DATA dev_data)
{
    std::string hardware_ids = getDeviceProperty(dev_info, dev_data, SPDRP_HARDWAREID);
    std::transform(hardware_ids.begin(), hardware_ids.end(), hardware_ids.begin(), ::toupper);
    std::size_t vid_idx = hardware_ids.find("VID_") + 4;
    std::size_t pid_idx = hardware_ids.find("PID_") + 4;
    if (vid_idx != std::string::npos && pid_idx != std::string::npos)
    {
        port_info->vendor_id = (uint16_t)strtoul(hardware_ids.substr(vid_idx, 4).c_str(), NULL, 16);
        port_info->product_id = (uint16_t)strtoul(hardware_ids.substr(pid_idx, 4).c_str(), NULL, 16);
    }

    HKEY dev_key = ::SetupDiOpenDevRegKey(dev_info, dev_data, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_QUERY_VALUE);
    LPCSTR property_name = "PortName";
    
    port_info->port_name = getRegKeyValue(dev_key, property_name);
}


/********************************************//**
 * Gets a list of addresses of Bluetooth devices that are connected.
 ***********************************************/
#if USE_BTHAPI
static void connectedBthAddrs(std::vector<uint64_t> *bt_addr_list)
{
    // Set our Bluetooth search parameters
    BLUETOOTH_DEVICE_SEARCH_PARAMS bt_search_params;
    bt_search_params.dwSize = sizeof(bt_search_params);
    bt_search_params.fReturnAuthenticated = TRUE;
    bt_search_params.fReturnRemembered = FALSE;
    bt_search_params.fReturnUnknown = TRUE;
    bt_search_params.fReturnConnected = TRUE;
    bt_search_params.fIssueInquiry = TRUE;
    bt_search_params.cTimeoutMultiplier = 1;
    bt_search_params.hRadio = 0;

    // Create Bluetooth structure
    BLUETOOTH_DEVICE_INFO bt_dev_info;
    bt_dev_info.dwSize = sizeof(bt_dev_info);

    // Find Bluetooth devices
    HBLUETOOTH_DEVICE_FIND bt_handle;
    bt_handle = BluetoothFindFirstDevice(&bt_search_params, &bt_dev_info);

    if (bt_handle != NULL)
    {
        // Found a device
        bt_addr_list->push_back(bt_dev_info.Address.ullLong);
        // Now find more devices
        while (BluetoothFindNextDevice(bt_handle, &bt_dev_info))
        {
            bt_addr_list->push_back(bt_dev_info.Address.ullLong);
        }
        BluetoothFindDeviceClose(bt_handle);
    }
}
#endif // USE_WIN_BTH


/********************************************//**
 * Gets a list of ports to check against.
 ***********************************************/
static void enumerateDevices(const GUID &guid, std::vector<BasicPortInfo> *info_list)
{
    HDEVINFO dev_info;
#if USE_BTHAPI
    std::vector<uint64_t> bt_addr_list;
#endif // USE_BTHAPI
    if ((dev_info = ::SetupDiGetClassDevsA(&guid, NULL, NULL, DIGCF_PRESENT)) != INVALID_HANDLE_VALUE)
    {
        SP_DEVINFO_DATA dev_info_data;
        dev_info_data.cbSize = sizeof(SP_DEVINFO_DATA);
        for(int8_t i=0; ::SetupDiEnumDeviceInfo(dev_info, i, &dev_info_data); ++i)
        {
            BasicPortInfo info;
            info.product_id = info.vendor_id = 0;
            getDeviceDetails(&info, dev_info, &dev_info_data);
            if (startsWith(info.port_name, "COM"))
            {
                if (!startsWith(getDeviceProperty(dev_info, &dev_info_data, SPDRP_ENUMERATOR_NAME), "BTHENUM", true))
                {
                    info_list->push_back(info);
                }
                else
                {
                    continue; /// Need to look into the getDeviceInterfaceDetail function. Causing some error later in the code.
                    SP_DEVICE_INTERFACE_DATA dev_intf_data;
                    dev_intf_data.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);
                    std::string dev_path = getDeviceInterfaceDetail(dev_info, &dev_info_data, &guid, &dev_intf_data);
                    std::size_t mac_idx = dev_path.rfind("&") + 1;
                    std::string bt_addr_str = dev_path.substr(mac_idx, 12);
                    uint64_t bt_addr = strtoull(bt_addr_str.c_str(), NULL, 16);
                    if (bt_addr > 0)
                    {
#if USE_BTHAPI
                        if (bt_addr_list.size() == 0)
                        {
                            connectedBthAddrs(&bt_addr_list);
                        }

                        for (uint8_t bt = 0; bt<bt_addr_list.size(); ++bt)
                        {
                            if (bt_addr_list[bt] == bt_addr)
                            {
                                info.vendor_id = 0x2476;
                                info.product_id = 0x1060;
                                info_list->push_back(info);
                                break;
                            }
                        }
#else
                        info_list->push_back(info);
#endif // USE_WIN_BTH
                    }
                }
            }
        }
        ::SetupDiDestroyDeviceInfoList(dev_info);
    }
}


std::vector <BasicPortInfo> SerialEnumeratorPrivate::getPorts_sys()
{
    std::vector<BasicPortInfo> ports;
    const uint8_t count = sizeof(g_device_class_guids) / sizeof(g_device_class_guids[0]);
    for (uint8_t i=0; i<count; ++i)
    {
        enumerateDevices(g_device_class_guids[i], &ports);
    }
    std::sort(ports.begin(), ports.end(), lessThan);
    return ports;
}


#endif // _WIN32
