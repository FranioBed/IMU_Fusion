#define _CRT_SECURE_NO_DEPRECATE
#include <stdio.h>
#include <fstream>
#include <time.h>
#include "prio_api_export.h"

#define PRIO_STREAM_DURATION_INFINITE 0xffffffff
void main() {
	// An unique identifier used by the PrioVR API to identify the PrioVR BaseStation
	prio_device_id bs_device;
	// An unique identifier used by the PrioVR API to identify the PrioVR Hub
	prio_device_id hub_device;
	// The communication port of a PrioVR Hub
	prio_ComPort port;
	port.port_name = new char[64];
	// An index into the array of communication ports found by the Prio API
	uint8_t com_offset = 0;
	// Find all of the communication ports that corresponded to PrioVR Basestations (was Hubs)
	//prio_findPorts(PRIO_BS);
	prio_findPorts(PRIO_HUB);
	// Get the communication port at the index of com_offset
	prio_getPort(port.port_name, com_offset, &port.device_type);
	// Create a PrioVR Basestation (was Hub) object from the communication port
	prio_createHub(port.port_name, &hub_device);
	//prio_createBaseStation(port.port_name, &bs_device);
	// Get the PrioVR Hub paired to the PrioVR Basestation
	//prio_bs_getWirelessHub(bs_device, &hub_device);
	// Setup to record the first 1000 packets
	prio_hub_setupRecordingOptions(hub_device, 1000, false);
	// This starts recording untared orientations, at an interval of 10000 mircoseconds forever
	prio_hub_startRecordingWired(hub_device, PRIO_STREAM_ALL_RAW_COMPONENT_SENSOR_DATA, 10000,
		PRIO_STREAM_DURATION_INFINITE);
	/*prio_bs_startRecording(bs_device, PRIO_STREAM_UNTARED_ORIENTATION_AS_QUATERNION, 0,
		PRIO_STREAM_DURATION_INFINITE);*/
	// Wait to get 1000 packets
	printf("Press enter when you are finished recording 1000 packets");
	getchar();
	// Stops the recording of the PrioVR Hub
	prio_hub_stopRecordingWired(hub_device);
	//prio_bs_stopRecording(bs_device);
	// Gather the actual number of recorded packets
	U32 packet_count = 0;
	prio_hub_getLengthOfRecordedSamples(hub_device, &packet_count);
	// Create buffers to hold the header data, and packet data for 1000 packets sized for 19 sensors
	const int sensorDataSize = 36;
	const int packetSize = 19 * sensorDataSize;
	PrioHeader headerData[1000];
	U8 packetData[packetSize * 1000];
	// Grab the data from the PrioAPI
	prio_hub_getRecordedSamples(hub_device, headerData, packetData, packet_count);
	// Create a file pointer and file
	FILE * fp;
	fp = fopen("PrioRecordedData.txt", "w + ");
	//std::ifstream fp("PrioRecordedData.txt");
	// Iterate over every packet and write the packet to the file
	for (int i = 0; i < (int)packet_count; i++) {
		// Get the packet header data for the current index
		PrioHeader tmpHeader = headerData[i];
		fprintf(fp, " == = = Header Data at Index % d == = = \n", i);
		fprintf(fp, "TimeStamp: % d\n", (int)tmpHeader.SensorTimestamp);
		fprintf(fp, "Battery Level : % d\n", (int)tmpHeader.BatteryLevel);
		fprintf(fp, "Joystick 1 X axis : % d\n", (int)tmpHeader.Joystick1.x_Axis);
		fprintf(fp, "Joystick 1 Y axis : % d\n", (int)tmpHeader.Joystick1.y_Axis);
		fprintf(fp, "Joystick 1 Trigger: % d\n", (int)tmpHeader.Joystick1.Trigger);
		fprintf(fp, "Joystick 1 Button State : % d\n", (int)tmpHeader.Joystick1.ButtonState);
		fprintf(fp, "Joystick 2 X axis : % d\n", (int)tmpHeader.Joystick2.x_Axis);
		fprintf(fp, "Joystick 2 Y axis : % d\n", (int)tmpHeader.Joystick2.y_Axis);
		fprintf(fp, "Joystick 2 Trigger: % d\n", (int)tmpHeader.Joystick2.Trigger);
		fprintf(fp, "Joystick 2 Button State : % d\n", (int)tmpHeader.Joystick2.ButtonState);
		fprintf(fp, " == = == = == = == = == = == = == = == = == = == = == = == = == = = \n");
		fprintf(fp, " == = = Sensor Data == = = \n");
		fprintf(fp, " (GyroData x, y, z), (AccData x, y, z), (MagData x, y, z), \n");

		for (int j = 0; j < 19; j++) {
			// Get the current sensor quaterion
			float tmp[9];
			memcpy(tmp, packetData + (i * packetSize) + j * sensorDataSize, sensorDataSize);
			fprintf(fp, "(%f,\t %f,\t %f),\t\t (%f,\t %f,\t %f),\t\t (%f,\t %f,\t %f)\n", tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5], tmp[6], tmp[7], tmp[8]);
		}
		fprintf(fp, " == = = End of Sensor Data == = \n");
	}
	// Close the file
	fclose(fp);
	// Deinitialize the Prio API
	prio_deinitAPI();
}