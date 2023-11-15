/*
 * Copyright (C) 2015 The CyanogenMod Project
 *               2017-2019 The LineageOS Project
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

package org.pixelexperience.settings.doze;

import android.content.Context;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.hardware.display.AmbientDisplayConfiguration;
import android.os.UserHandle;
import android.provider.Settings;
import android.util.Log;
import androidx.preference.PreferenceManager;

import static android.provider.Settings.Secure.DOZE_ALWAYS_ON;
import static android.provider.Settings.Secure.DOZE_DOUBLE_TAP_GESTURE;
import static android.provider.Settings.Secure.DOZE_ENABLED;
import static android.provider.Settings.Secure.DOZE_PRESS_UDFPS_MODE;

class TouchControl {
    static {
        System.loadLibrary("touch_jni");
    }
    
    public static native void setDoubleTapMode(int value);
}

public final class DozeUtils {
    private static final String TAG = "DozeUtils";
    private static final boolean DEBUG = false;

    private static final String DOZE_INTENT = "com.android.systemui.doze.pulse";

    protected static final String ALWAYS_ON_DISPLAY = "always_on_display";
    protected static final String GESTURE_DOUBLE_TAP_KEY = "gesture_double_tap";
    protected static final String GESTURE_PRESS_UDFPS_KEY = "gesture_press_udfps_type";

    protected static final String DOZE_ENABLE = "doze_enable";

    public static void initialize(Context context) {
        updateDoubleTap(context);
    }

    protected static boolean enableDoze(Context context, boolean enable) {
        return Settings.Secure.putInt(context.getContentResolver(),
                DOZE_ENABLED, enable ? 1 : 0);
    }

    public static boolean isDozeEnabled(Context context) {
        return Settings.Secure.getInt(context.getContentResolver(),
                DOZE_ENABLED, 1) != 0;
    }

    protected static boolean enableAlwaysOn(Context context, boolean enable) {
        return Settings.Secure.putIntForUser(context.getContentResolver(),
                DOZE_ALWAYS_ON, enable ? 1 : 0, UserHandle.USER_CURRENT);
    }

    protected static boolean enableDoubleTap(Context context, boolean enable) {
        TouchControl.setDoubleTapMode(enable ? 1 : 0);
        return Settings.Secure.putIntForUser(context.getContentResolver(),
                DOZE_DOUBLE_TAP_GESTURE, enable ? 1 : 0, UserHandle.USER_CURRENT);
    }

    protected static void updateDoubleTap(Context context) {
        int value = Settings.Secure.getInt(context.getContentResolver(),
                DOZE_DOUBLE_TAP_GESTURE, 0);
        Log.w(TAG, "updateDoubleTap " + value);
        TouchControl.setDoubleTapMode(value);
    }

    protected static boolean setPressUdfpsMode(Context context, int mode) {
        return Settings.Secure.putIntForUser(context.getContentResolver(),
                DOZE_PRESS_UDFPS_MODE, mode, UserHandle.USER_CURRENT);
    }

    protected static boolean isAlwaysOnEnabled(Context context) {
        final boolean enabledByDefault = context.getResources()
                .getBoolean(com.android.internal.R.bool.config_dozeAlwaysOnEnabled);

        return Settings.Secure.getIntForUser(context.getContentResolver(),
                DOZE_ALWAYS_ON, alwaysOnDisplayAvailable(context) && enabledByDefault ? 1 : 0,
                UserHandle.USER_CURRENT) != 0;
    }

    protected static boolean alwaysOnDisplayAvailable(Context context) {
        return new AmbientDisplayConfiguration(context).alwaysOnAvailable();
    }
}
