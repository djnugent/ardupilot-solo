#ifndef __AP_ACCELCAL_H__
#define __AP_ACCELCAL_H__

#include <GCS_MAVLink.h>
#include "AccelCalibrator.h"

#define AP_ACCELCAL_MAX_NUM_CLIENTS 4

class GCS_MAVLINK;
class AP_AccelCal_Client;

class AP_AccelCal {
public:
    AP_AccelCal():
    _num_clients(0) {}

    void start(GCS_MAVLINK *gcs);
    void clear();
    void update();
    void register_client(AP_AccelCal_Client* client);
    void collect_sample();

    accel_cal_status_t get_status();

private:
    GCS_MAVLINK *_gcs;
    uint8_t _step;
    accel_cal_status_t _prev_status;
    uint8_t _num_clients;
    AP_AccelCal_Client* _clients[AP_ACCELCAL_MAX_NUM_CLIENTS];

    AccelCalibrator* get_calibrator(uint8_t i);
    void _printf(const char*, ...);
};

class AP_AccelCal_Client {
friend class AP_AccelCal;
private:
    virtual void _acal_save_corrections() = 0;
    virtual AccelCalibrator* _acal_get_calibrator(uint8_t instance) = 0;
};

#endif
