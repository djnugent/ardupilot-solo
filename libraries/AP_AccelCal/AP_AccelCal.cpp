#include "AP_AccelCal.h"
#include <stdarg.h>
#include <GCS.h>
#include <AP_HAL.h>

const extern AP_HAL::HAL& hal;

void AP_AccelCal::update()
{
    if (!get_calibrator(0)) {
        // no calibrators
        return;
    }

    if (_gcs == NULL) {
        clear();
        return;
    }

    AccelCalibrator *cal;
    for(uint8_t i=0; (cal = get_calibrator(i)); i++) {
        cal->check_for_timeout();
    }

    accel_cal_status_t status = get_status();

    if (status == ACCEL_CAL_FAILED) {
        printf("Calibration FAILED");
        clear();
        return;
    }

    if (status == ACCEL_CAL_SUCCESS) {
        // save
        for(uint8_t i=0; i<_num_clients; i++) {
            _clients[i]->_acal_save_corrections();
        }
        printf("Calibration successful");
        clear();
        return;
    }

    uint8_t step;
    // if we're waiting for orientation, ensure that all calibrators are on the same step
    if (status == ACCEL_CAL_WAITING_FOR_ORIENTATION) {
        cal = get_calibrator(0);
        step = cal->get_num_samples_collected();

        for(uint8_t i=1 ; (cal = get_calibrator(i))  ; i++) {
            if (step != cal->get_num_samples_collected()) {
                clear();
                return;
            }
        }
    }

    // if we've transitioned from collecting sample to waiting for orientation,
    // print a message describing the orientation we want
    if (_prev_status == ACCEL_CAL_COLLECTING_SAMPLE && status == ACCEL_CAL_WAITING_FOR_ORIENTATION) {
        const prog_char_t *msg;
        switch (step) {
            case 0:
                msg = "level";
                break;
            case 1:
                msg = "on its LEFT side";
                break;
            case 2:
                msg = "on its RIGHT side";
                break;
            case 3:
                msg = "nose DOWN";
                break;
            case 4:
                msg = "nose UP";
                break;
            case 5:
                msg = "on its BACK";
                break;
            default:
                clear();
                return;
        }
        printf("Place vehicle %s and press any key.", msg);
    }

    _prev_status = status;
}

void AP_AccelCal::start(GCS_MAVLINK *gcs)
{
    if (gcs == NULL) {
        return;
    }

    AccelCalibrator *cal;
    for(uint8_t i=0; (cal = get_calibrator(i)); i++) {
        cal->start(ACCEL_CAL_AXIS_ALIGNED_ELLIPSOID, 6, 0.5f);
    }

    _gcs = gcs;

    update();
}

void AP_AccelCal::collect_sample()
{
    AccelCalibrator *cal;
    for(uint8_t i=0 ; (cal = get_calibrator(i))  ; i++) {
        cal->collect_sample();
    }
}

void AP_AccelCal::clear()
{
    AccelCalibrator *cal;
    for(uint8_t i=0 ; (cal = get_calibrator(i))  ; i++) {
        cal->clear();
    }

    _gcs = NULL;
}

accel_cal_status_t AP_AccelCal::get_status() {
    AccelCalibrator *cal;

    for(uint8_t i=0 ; (cal = get_calibrator(i))  ; i++) {
        if (cal->get_status() == ACCEL_CAL_FAILED) {
            return ACCEL_CAL_FAILED;
        }
    }

    for(uint8_t i=0 ; (cal = get_calibrator(i))  ; i++) {
        if (cal->get_status() == ACCEL_CAL_COLLECTING_SAMPLE) {
            return ACCEL_CAL_COLLECTING_SAMPLE;
        }
    }

    for(uint8_t i=0 ; (cal = get_calibrator(i))  ; i++) {
        if (cal->get_status() == ACCEL_CAL_WAITING_FOR_ORIENTATION) {
            return ACCEL_CAL_WAITING_FOR_ORIENTATION;
        }
    }

    for(uint8_t i=0 ; (cal = get_calibrator(i))  ; i++) {
        if (cal->get_status() == ACCEL_CAL_NOT_STARTED) {
            return ACCEL_CAL_NOT_STARTED;
        }
    }

    return ACCEL_CAL_SUCCESS;
}

AccelCalibrator* AP_AccelCal::get_calibrator(uint8_t index) {
    AccelCalibrator* ret;
    for(uint8_t i=0; i<_num_clients; i++) {
        for(uint8_t j=0 ; (ret = _clients[i]->_acal_get_calibrator(j)) ; j++) {
            if (index == 0) {
                return ret;
            }
            index--;
        }
    }
    return NULL;
}

void AP_AccelCal::register_client(AP_AccelCal_Client* client) {
    if (client == NULL || _num_clients == AP_ACCELCAL_MAX_NUM_CLIENTS) {
        return;
    }
    _clients[_num_clients] = client;
    _num_clients++;
}

void AP_AccelCal::_printf(const char* fmt, ...)
{
    char msg[50];
    va_list ap;
    va_start(ap, fmt);
    hal.util->vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    if (msg[strlen(msg)-1] == '\n') {
        // STATUSTEXT messages should not add linefeed
        msg[strlen(msg)-1] = 0;
    }
    AP_HAL::UARTDriver *uart = _gcs->get_uart();
    /*
     *     to ensure these messages get to the user we need to wait for the
     *     port send buffer to have enough room
     */
    while (uart->txspace() < MAVLINK_NUM_NON_PAYLOAD_BYTES+MAVLINK_MSG_ID_STATUSTEXT_LEN) {
        hal.scheduler->delay(1);
    }
    _gcs->send_text(SEVERITY_HIGH, msg);
}
