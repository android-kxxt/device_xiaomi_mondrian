/*
 * Copyright (C) 2014 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#define LOG_TAG "GatewayHal"

#include <android-base/logging.h>
#include <android-base/unique_fd.h>
#include <android-base/properties.h>

#include <stdlib.h>
#include <dlfcn.h>
#include <errno.h>
#include <string.h>
#include <cutils/log.h>
#include <hardware/hardware.h>
#include "fingerprint.h"

static const uint16_t kVersion = HARDWARE_MODULE_API_VERSION(2, 1);
static fingerprint_device_t* targetDevice = nullptr;

static int load(const char *id,
        const char *path,
        const struct hw_module_t **pHmi)
{
    int status = -EINVAL;
    struct hw_module_t *hmi = NULL;
    void *handle = dlopen(path, RTLD_NOW);

    if (handle == NULL) {
        char const *err_str = dlerror();
        ALOGE("load: module=%s\n%s", path, err_str?err_str:"unknown");
        return -EINVAL;
    }

    /* Get the address of the struct hal_module_info. */
    const char *sym = HAL_MODULE_INFO_SYM_AS_STR;
    hmi = (struct hw_module_t *)dlsym(handle, sym);
    if (hmi == NULL) {
        ALOGE("load: couldn't find symbol %s", sym);
        dlclose(handle);
        return -EINVAL;
    }

    /* Check that the id matches */
    if (strcmp(id, hmi->id) != 0) {
        ALOGE("load: id=%s != hmi->id=%s", id, hmi->id);
        dlclose(handle);
        return -EINVAL;
    }

    hmi->dso = handle;

    /* success */
    status = 0;
    *pHmi = hmi;

    return status;
}

fingerprint_device_t* openHal(const char* class_name) {
    int err;
    const hw_module_t* hw_mdl = nullptr;
    ALOGD("Opening fingerprint hal library...");
    if (0 != (err = load(FINGERPRINT_HARDWARE_MODULE_ID, class_name, &hw_mdl))) {
        ALOGE("Can't open fingerprint HW Module, error: %d", err);
        return nullptr;
    }

    if (hw_mdl == nullptr) {
        ALOGE("No valid fingerprint module");
        return nullptr;
    }

    fingerprint_module_t const* module = reinterpret_cast<const fingerprint_module_t*>(hw_mdl);
    if (module->common.methods->open == nullptr) {
        ALOGE("No valid open method");
        return nullptr;
    }

    hw_device_t* device = nullptr;

    if (0 != (err = module->common.methods->open(hw_mdl, nullptr, &device))) {
        ALOGE("Can't open fingerprint methods, error: %d", err);
        return nullptr;
    }

    if (kVersion != device->version) {
        // enforce version on new devices because of HIDL@2.1 translation layer
        ALOGE("Wrong fp version. Expected %d, got %d", kVersion, device->version);
        return nullptr;
    }

    return reinterpret_cast<fingerprint_device_t*>(device);
}

static int fingerprint_close(hw_device_t *dev)
{
    if (dev) {
        free(dev);
        return 0;
    } else {
        return -1;
    }
}

static int fingerprint_set_notify(struct fingerprint_device *dev,
                                fingerprint_notify_t notify) {
    /* Decorate with locks */
    dev->notify = notify;

    if (targetDevice) {
        LOG(INFO) << __func__;
        return targetDevice->set_notify(dev, notify);
    }

    return FINGERPRINT_ERROR;
}

static uint64_t fingerprint_pre_enroll(struct fingerprint_device *dev) {
    if (targetDevice) {
        LOG(INFO) << __func__;
        return targetDevice->pre_enroll(dev);
    }
    return FINGERPRINT_ERROR;
}

static int fingerprint_enroll(struct fingerprint_device *dev, const hw_auth_token_t *hat,
                uint32_t gid, uint32_t timeout_sec) {
    if (targetDevice) {
        LOG(INFO) << __func__ << " gid " << gid << " timeout_sec " << timeout_sec;
        return targetDevice->enroll(dev, hat, gid, timeout_sec);
    }
    return FINGERPRINT_ERROR;
}

static int fingerprint_post_enroll(struct fingerprint_device *dev) {
    if (targetDevice) {
        LOG(INFO) << __func__;
        return targetDevice->post_enroll(dev);
    }
    return FINGERPRINT_ERROR;
}

static uint64_t fingerprint_get_authenticator_id(struct fingerprint_device *dev) {
    if (targetDevice) {
        LOG(INFO) << __func__;
        return targetDevice->get_authenticator_id(dev);
    }
    return FINGERPRINT_ERROR;
}

static int fingerprint_cancel(struct fingerprint_device *dev) {
    if (targetDevice) {
        LOG(INFO) << __func__;
        return targetDevice->cancel(dev);
    }
    return FINGERPRINT_ERROR;
}

int fingerprint_enumerate(struct fingerprint_device *dev) {
    if (targetDevice) {
        LOG(INFO) << __func__;
        return targetDevice->enumerate(dev);
    }
    return FINGERPRINT_ERROR;
}

static int fingerprint_remove(struct fingerprint_device *dev, uint32_t gid, uint32_t fid) {
    if (targetDevice) {
        LOG(INFO) << __func__ << " gid " << gid << " fid " << fid;
        return targetDevice->remove(dev, gid, fid);
    }
    return FINGERPRINT_ERROR;
}

static int fingerprint_set_active_group(struct fingerprint_device *dev, uint32_t gid,
                        const char *store_path) {
    if (targetDevice) {
        LOG(INFO) << __func__ << " gid " << gid << " store_path " << store_path;
        return targetDevice->set_active_group(dev, gid, store_path);
    }
    return FINGERPRINT_ERROR;                        
}

static int fingerprint_authenticate(struct fingerprint_device *dev, uint64_t operation_id, uint32_t gid) {
    if (targetDevice) {
        LOG(INFO) << __func__ << " operation_id " << operation_id << " gid " << gid;
        return targetDevice->authenticate(dev, operation_id, gid);
    }
    return FINGERPRINT_ERROR;
}

static int fingerprint_extCmd(struct fingerprint_device* dev, int32_t cmd, int32_t param) {
    if (targetDevice) {
        LOG(INFO) << __func__ << " cmd " << cmd << " param " << param;
        return targetDevice->extCmd(dev, cmd, param);
    }
    return FINGERPRINT_ERROR;
}

static int fingerprint_open(const hw_module_t* module, const char __unused *id,
                            hw_device_t** device)
{
    targetDevice = openHal("fingerprint.fpc_fod.default.so");

    if (!targetDevice) {
        ALOGE("NULL target device on open");
        return -EINVAL;   
    }

    if (device == NULL) {
        ALOGE("NULL device on open");
        return -EINVAL;
    }

    fingerprint_device_t *dev = (fingerprint_device_t*)malloc(sizeof(fingerprint_device_t));
    memset(dev, 0, sizeof(fingerprint_device_t));

    dev->common.tag = HARDWARE_DEVICE_TAG;
    dev->common.version = HARDWARE_MODULE_API_VERSION(2, 1);
    dev->common.module = (struct hw_module_t*) module;
    dev->common.close = fingerprint_close;

    dev->set_notify = fingerprint_set_notify;
    dev->pre_enroll = fingerprint_pre_enroll;
    dev->enroll = fingerprint_enroll;
    dev->post_enroll = fingerprint_post_enroll;
    dev->get_authenticator_id = fingerprint_get_authenticator_id;
    dev->cancel = fingerprint_cancel;
    dev->enumerate = fingerprint_enumerate;
    dev->remove = fingerprint_remove;
    dev->set_active_group = fingerprint_set_active_group;
    dev->authenticate = fingerprint_authenticate;
    dev->extCmd = fingerprint_extCmd;
    dev->notify = NULL;

    *device = (hw_device_t*) dev;
    return 0;
}

static struct hw_module_methods_t fingerprint_module_methods = {
    .open = fingerprint_open,
};

fingerprint_module_t HAL_MODULE_INFO_SYM = {
    .common = {
        .tag                = HARDWARE_MODULE_TAG,
        .module_api_version = FINGERPRINT_MODULE_API_VERSION_1_0,
        .hal_api_version    = HARDWARE_HAL_API_VERSION,
        .id                 = FINGERPRINT_HARDWARE_MODULE_ID,
        .name               = "Fingerprint Gateway HAL",
        .author             = "The Android Open Source Project",
        .methods            = &fingerprint_module_methods,
    },
};