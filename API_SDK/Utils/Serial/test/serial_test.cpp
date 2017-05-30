#include <string>
#include <sstream>
#include <iostream>
#include <cstdio>
#include <iomanip>
#include <thread>
#include <atomic>

// OS Specific sleep
#ifdef _WIN32
    #include <windows.h>
#else
    #include <unistd.h>
#endif // _WIN32

#include "serial.h"
#include "serial_enumerator.hpp"

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;

void my_sleep(unsigned long milliseconds)
{
    #ifdef _WIN32
        Sleep(milliseconds); // 100 ms
    #else
        usleep(milliseconds*1000); // 100 ms
    #endif // _WIN32
}

void _endianSwap32(uint32_t* bytes)
{
    *bytes = (*bytes >> 24) |
        ((*bytes << 8) & 0x00FF0000) |
        ((*bytes >> 8) & 0x0000FF00) |
        (*bytes << 24);
}


int runTest1()
{
    // Set ASCII command and read size
    string test_string = ":237\n"; // Serial Number
    uint8_t test_bin[3] = { 0xf7, 0xed, 0xed };
    int test_read_bytes = 9;
    uint32_t test_bin_read_bytes;

    // Set the baudrate
    unsigned long baud = 115200;

    // Find a serial port
    SerialEnumerator serial_enumer;
    std::vector<BasicPortInfo> ports = serial_enumer.getPorts();
    for (unsigned int i=0; i<ports.size(); ++i)
    {
        cout << "PortName: " << ports[i].port_name << endl;
        cout << "VendorID: " << std::setfill('0') << std::setw(4) << ports[i].vendor_id << std::hex << std::uppercase << endl;
        cout << "ProductID: " << std::setfill('0') << std::setw(4) << ports[i].product_id << std::hex << std::uppercase << endl;
        cout << "======================" << endl;

        // Check if port's vendor id matches our vendor id
        if (ports[i].vendor_id != 0x2476) {continue;}

        // Create serial port
        string port(ports[i].port_name);

        // port, baudrate, timeout in milliseconds
        serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(250));

        // Run tests
        cout << "Is the serial port open?";
        if (my_serial.isOpen())
        {
            cout << " Yes." << endl;
        }
        else
        {
            cout << " No." << endl;
            return 1;
        }

        size_t bytes_wrote = my_serial.write(test_string);

        string result = my_serial.read(test_read_bytes);

        cout << "Bytes written: " << bytes_wrote << ", Bytes read: " << result.length() << ", String read: " << result << endl;

        size_t bytes_wrote_bin = my_serial.write(test_bin, 3);

        size_t result_bin = my_serial.read((uint8_t *)&test_bin_read_bytes, 4);

        cout << "Bytes written: " << bytes_wrote_bin << ", Bytes read: " << result_bin << ", String read: " << test_bin_read_bytes << endl;

        _endianSwap32(&test_bin_read_bytes);

        std::stringstream hex_string;
        hex_string << std::setfill('0') << std::setw(8) << std::hex << std::uppercase << test_bin_read_bytes;

        cout << "String read: " << hex_string.str() << endl;

        my_serial.close();
    }

    return 0;
}

std::atomic<bool> g_continue_reading (false);

void readLoop(serial::Serial* port)
{
    while (g_continue_reading)
    {
        cout << port->read(35) << endl;
    }
}

int runTest2()
{
    std::vector<serial::Serial*> serial_list;

    // Set the baudrate
    unsigned long baud = 115200;

    // Find a serial port
    SerialEnumerator serial_enumer;
    std::vector<BasicPortInfo> ports = serial_enumer.getPorts();
    for (unsigned int i=0; i<ports.size(); ++i)
    {
        // Check if port's vendor id matches our vendor id
        if (ports[i].vendor_id != 0x2476) {continue;}

        // Create serial port
        string port(ports[i].port_name);

        // port, baudrate, timeout in milliseconds
        serial_list.push_back(new serial::Serial(port, baud, serial::Timeout::simpleTimeout(250)));


        size_t bytes_wrote = serial_list[0]->write(":237\n");

        string result = serial_list[0]->read(9);

        cout << "Bytes written: " << bytes_wrote << ", Bytes read: " << result.length() << ", String read: " << result << endl;

        // Setup streaming slots
        serial_list[0]->write(":80,6,255,255,255,255,255,255,255\n");

        // Setup streaming timing
        serial_list[0]->write(":82,0,-1,0\n");
    }

    g_continue_reading = true;
    std::thread test(readLoop, serial_list[0]);
    serial_list[0]->write(":85\n");


    my_sleep(500);

    g_continue_reading = false;
    serial_list[0]->write(":86\n");
    //may need a flush if more data is needed after here

    test.join();

    serial_list[0]->close();
    return 0;
}

int main (int argc, char **argv)
{
    try
    {
        return runTest1();
        //return runTest2();
    }
    catch (exception &e)
    {
        cerr << "Unhandled Exception: " << e.what() << endl;
    }
}
