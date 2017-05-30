#include "yost_connection_processor.hpp"
#include "threespace_api.hpp"
#include "threespace_api_export.h"

namespace yost
{
    // Class for Prio device connections
    class TssConnection : public YostConnection
    {
    public:
        TssConnection();
        ~TssConnection();

        void runProcess();

        // Device functions
        void reconnectDevice();
        void disconnectDevice();

        void startStreaming();
        void stopStreaming();

        void clearRecordedSamples();

        // Skeleton calls
        bool calibrate(float wait_time = 0.0f);
        void setupSkeletonMap();
        U32 removeUnusedSensors();

    private:
        uint8_t _active_sensors_len;

        vector<shared_ptr<TssSensor>> _active_sensors;
        vector<shared_ptr<TssDongle>> _active_dongles;
    };
};

