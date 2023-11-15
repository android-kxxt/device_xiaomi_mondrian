#include <jni.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include "xiaomi_touch.h"

#define TOUCH_DEV_PATH "/dev/xiaomi-touch"
#define TOUCH_ID 0
#define TOUCH_MAGIC 'T'
#define TOUCH_IOC_SET_CUR_VALUE _IO(TOUCH_MAGIC, SET_CUR_VALUE)
#define TOUCH_IOC_GET_CUR_VALUE _IO(TOUCH_MAGIC, GET_CUR_VALUE)

extern "C" JNIEXPORT void JNICALL
Java_org_pixelexperience_settings_doze_TouchControl_setDoubleTapMode(JNIEnv* env, jclass, jint value) {
    (void)env;

    int touchFd = open(TOUCH_DEV_PATH, O_RDWR);

    if (touchFd < 0) {
        return;
    }

    int buf[MAX_BUF_SIZE] = {TOUCH_ID, TOUCH_DOUBLETAP_MODE, value};
    ioctl(touchFd, TOUCH_IOC_SET_CUR_VALUE, &buf);

    close(touchFd);
}