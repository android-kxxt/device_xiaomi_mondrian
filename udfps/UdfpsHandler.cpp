/*
 * Copyright (C) 2022 The LineageOS Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_TAG "UdfpsHandler.xiaomi_sm8450"

#include <android-base/logging.h>
#include <android-base/unique_fd.h>
#include <android-base/properties.h>

#include <poll.h>
#include <sys/ioctl.h>
#include <fstream>
#include <thread>

#include "UdfpsHandler.h"
#include "xiaomi_touch.h"

#define COMMAND_NIT 10
#define PARAM_NIT_FOD 1
#define PARAM_NIT_NONE 0

#define COMMAND_FOD_PRESS_STATUS 1
#define PARAM_FOD_PRESSED 1
#define PARAM_FOD_RELEASED 0

#define COMMAND_FOD_PRESS_X 2
#define COMMAND_FOD_PRESS_Y 3

#define FOD_STATUS_OFF 0
#define FOD_STATUS_ON 1

#define TOUCH_DEV_PATH "/dev/xiaomi-touch"
#define TOUCH_ID 0
#define TOUCH_MAGIC 'T'
#define TOUCH_IOC_SET_CUR_VALUE _IO(TOUCH_MAGIC, SET_CUR_VALUE)
#define TOUCH_IOC_GET_CUR_VALUE _IO(TOUCH_MAGIC, GET_CUR_VALUE)

#define FOD_PRESS_STATUS_PATH "/sys/class/touch/touch_dev/fod_press_status"

#define DISP_FEATURE_PATH "/dev/mi_display/disp_feature"
#define MI_DISP_IOCTL_REGISTER_EVENT _IOC(_IOC_WRITE, 0x44, 0x7, 0xc)
#define MI_DISP_IOCTL_SET_LOCAL_HBM  _IOC(_IOC_WRITE, 0x44, 0xE, 0xc)

// flag, disp_id, type
// _IOC(_IOC_WRITE, 0x44, 0x7, 0xc)
// nr = 7, size = 13,
// disp_event_req from techpack/display/include/uapi/display/drm/mi_disp.h
//
// /dev/mi_display/disp_feature

struct disp_base
{
    __u32 flag;
    __u32 disp_id;
};

struct disp_event_req
{
    struct disp_base base;
    __u32 type;
};

namespace {

using ::android::base::GetProperty;

template <typename T>
static void set(const std::string& path, const T& value) {
    std::ofstream file(path);
    file << value;
}

static bool readBool(int fd) {
    char c;
    int rc;

    rc = lseek(fd, 0, SEEK_SET);
    if (rc) {
        LOG(ERROR) << "failed to seek fd, err: " << rc;
        return false;
    }

    rc = read(fd, &c, sizeof(char));
    if (rc != 1) {
        LOG(ERROR) << "failed to read bool from fd, err: " << rc;
        return false;
    }

    return c != '0';
}

}  // anonymous namespace

class XiaomiSm8450UdfpsHander : public UdfpsHandler {
  public:
    void init(fingerprint_device_t* device) {
        mDevice = device;
        halName = GetProperty("persist.vendor.sys.fp.vendor", "none");
        touch_fd_ = android::base::unique_fd(open(TOUCH_DEV_PATH, O_RDWR));
        disp_feature_fd_ = android::base::unique_fd(open(DISP_FEATURE_PATH, O_RDWR));
        for (unsigned disp = 0; disp < 2; disp++) {
            struct disp_event_req req = {
                .base = {
                    .flag = 0,
                    .disp_id = disp
                },
                .type = 2
            };
            ioctl(disp_feature_fd_.get(), MI_DISP_IOCTL_REGISTER_EVENT, &req);
            req.type = 5;
            ioctl(disp_feature_fd_.get(), MI_DISP_IOCTL_REGISTER_EVENT, &req);
        }

        std::thread([this]() {
            int fd = open(FOD_PRESS_STATUS_PATH, O_RDONLY);
            if (fd < 0) {
                LOG(ERROR) << "failed to open fd, err: " << fd;
                return;
            }

            struct pollfd fodPressStatusPoll = {
                    .fd = fd,
                    .events = POLLERR | POLLPRI,
                    .revents = 0,
            };

            while (true) {
                int rc = poll(&fodPressStatusPoll, 1, -1);
                if (rc < 0) {
                    LOG(ERROR) << "failed to poll fd, err: " << rc;
                    continue;
                }

                bool pressed = readBool(fd);
                LOG(INFO) << __func__ << " extCmd: COMMAND_FOD_PRESS_STATUS " << pressed;

                extCmd(COMMAND_FOD_PRESS_STATUS,
                                pressed ? PARAM_FOD_PRESSED : PARAM_FOD_RELEASED);
            }
        }).detach();
    }

    void extCmd(int32_t cmd, int32_t param) {
        mDevice->extCmd(mDevice, COMMAND_FOD_PRESS_X, fod_x);
        mDevice->extCmd(mDevice, COMMAND_FOD_PRESS_Y, fod_y);
        mDevice->extCmd(mDevice, cmd, param);
    }

    void onFingerDown(uint32_t x, uint32_t y, float /*minor*/, float /*major*/) {
        // fod_x = x;
        // fod_y = y;
        LOG(INFO) << __func__;
        setFingerDown();
    }

    void onFingerUp() {
        LOG(INFO) << __func__;
        setFingerUp();
    }

    void onAcquired(int32_t result, int32_t vendorCode) {
        LOG(INFO) << __func__ << " result: " << result << " vendorCode: " << vendorCode;
        if (result == FINGERPRINT_ACQUIRED_GOOD) {
            setFingerUp();
            setFodStatus(FOD_STATUS_OFF);
        } else {
            if (halName == "fpc_fod") {
                if (vendorCode == 20 || vendorCode == 22) {
                    /*
                    * vendorCode = 20 waiting for fingerprint authentication
                    * vendorCode = 22 waiting for fingerprint enroll
                    */
                    setFodStatus(FOD_STATUS_ON);
                }
            }
            else if (halName == "goodix_fod") {
                if (vendorCode == 21 || vendorCode == 23) {
                    /*
                    * vendorCode = 21 waiting for fingerprint authentication
                    * vendorCode = 23 waiting for fingerprint enroll
                    */
                    setFodStatus(FOD_STATUS_ON);
                }        
            }
        }
    }

    void cancel() {
        LOG(INFO) << __func__;
        setFingerUp();
        setFodStatus(FOD_STATUS_OFF);
    }

  private:
    fingerprint_device_t* mDevice;
    android::base::unique_fd touch_fd_;
    android::base::unique_fd disp_feature_fd_;
    uint32_t fod_x = 0;
    uint32_t fod_y = 0;
    std::string halName;

    void setFodStatus(int value) {
        // fpc:
        // Don't trigger /dev/xiaomi-touch here.
        if (halName != "fpc_fod") {
            int buf[MAX_BUF_SIZE] = {TOUCH_ID, TOUCH_FOD_ENABLE, value};
            ioctl(touch_fd_.get(), TOUCH_IOC_SET_CUR_VALUE, &buf);
        }
    }

    void setFingerUp() {
        extCmd(COMMAND_NIT, PARAM_NIT_NONE);

        int buf[MAX_BUF_SIZE] = {TOUCH_ID, THP_FOD_DOWNUP_CTL, 0};
        ioctl(touch_fd_.get(), TOUCH_IOC_SET_CUR_VALUE, &buf);
        struct disp_event_req req = {
            .base = {
                .flag = 1,
                .disp_id = 0
            },
            .type = 1
        };
        ioctl(disp_feature_fd_.get(), MI_DISP_IOCTL_SET_LOCAL_HBM, &req);
    }

    void setFingerDown() {   
        extCmd(COMMAND_NIT, PARAM_NIT_FOD);

        int buf[MAX_BUF_SIZE] = {TOUCH_ID, THP_FOD_DOWNUP_CTL, 1};
        ioctl(touch_fd_.get(), TOUCH_IOC_SET_CUR_VALUE, &buf);
        struct disp_event_req req = {
            .base = {
                .flag = 1,
                .disp_id = 0
            },
            .type = 2
        };
        ioctl(disp_feature_fd_.get(), MI_DISP_IOCTL_SET_LOCAL_HBM, &req);
    }
};

static UdfpsHandler* create() {
    return new XiaomiSm8450UdfpsHander();
}

static void destroy(UdfpsHandler* handler) {
    delete handler;
}

extern "C" UdfpsHandlerFactory UDFPS_HANDLER_FACTORY = {
        .create = create,
        .destroy = destroy,
};
