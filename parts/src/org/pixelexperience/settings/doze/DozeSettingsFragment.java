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

import android.app.Activity;
import android.app.AlertDialog;
import android.app.Dialog;
import android.app.DialogFragment;
import android.content.Context;
import android.content.DialogInterface;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.os.Handler;
import android.widget.Switch;
import android.util.Log;

import androidx.preference.ListPreference;
import androidx.preference.Preference;
import androidx.preference.Preference.OnPreferenceChangeListener;
import androidx.preference.PreferenceCategory;
import androidx.preference.PreferenceFragment;
import androidx.preference.SwitchPreference;

import com.android.settingslib.widget.MainSwitchPreference;
import com.android.settingslib.widget.OnMainSwitchChangeListener;

import org.pixelexperience.settings.R;

public class DozeSettingsFragment extends PreferenceFragment implements OnPreferenceChangeListener,
        OnMainSwitchChangeListener {

    private MainSwitchPreference mSwitchBar;

    private SwitchPreference mAlwaysOnDisplayPreference;
    private SwitchPreference mDoubleTapPreference;
    private ListPreference mPressUdfpsPreference;

    @Override
    public void onCreatePreferences(Bundle savedInstanceState, String rootKey) {
        addPreferencesFromResource(R.xml.doze_settings);

        SharedPreferences prefs = getActivity().getSharedPreferences("doze_settings",
                Activity.MODE_PRIVATE);

        boolean dozeEnabled = DozeUtils.isDozeEnabled(getActivity());
        if (savedInstanceState == null && !prefs.getBoolean("first_saved", false)) {
            dozeEnabled = false;
            DozeUtils.enableDoze(getActivity(), dozeEnabled);
            getActivity().getSharedPreferences("doze_settings", Activity.MODE_PRIVATE)
                    .edit()
                    .putBoolean("first_saved", true)
                    .commit();
        }

        mSwitchBar = (MainSwitchPreference) findPreference(DozeUtils.DOZE_ENABLE);
        mSwitchBar.addOnSwitchChangeListener(this);
        mSwitchBar.setChecked(dozeEnabled);

        mAlwaysOnDisplayPreference = (SwitchPreference) findPreference(DozeUtils.ALWAYS_ON_DISPLAY);
        mAlwaysOnDisplayPreference.setEnabled(dozeEnabled);
        mAlwaysOnDisplayPreference.setChecked(DozeUtils.isAlwaysOnEnabled(getActivity()));
        mAlwaysOnDisplayPreference.setOnPreferenceChangeListener(this);

        mDoubleTapPreference = (SwitchPreference) findPreference(DozeUtils.GESTURE_DOUBLE_TAP_KEY);
        mDoubleTapPreference.setEnabled(dozeEnabled);
        mDoubleTapPreference.setOnPreferenceChangeListener(this);

        mPressUdfpsPreference = (ListPreference) findPreference(DozeUtils.GESTURE_PRESS_UDFPS_KEY);
        mPressUdfpsPreference.setEnabled(dozeEnabled);
        mPressUdfpsPreference.setOnPreferenceChangeListener(this);
    }

    @Override
    public boolean onPreferenceChange(Preference preference, Object newValue) {
        if (DozeUtils.ALWAYS_ON_DISPLAY.equals(preference.getKey())) {
            boolean value = Boolean.parseBoolean(newValue.toString());
            DozeUtils.enableAlwaysOn(getActivity(), value);
            mDoubleTapPreference.setEnabled(!value);
            mPressUdfpsPreference.setEnabled(!value);
        }
        else if (DozeUtils.GESTURE_DOUBLE_TAP_KEY.equals(preference.getKey())) {
            boolean value = Boolean.parseBoolean(newValue.toString());      
            DozeUtils.enableDoubleTap(getActivity(), value);
        }
        else if (DozeUtils.GESTURE_PRESS_UDFPS_KEY.equals(preference.getKey())) {
            int value = Integer.parseInt(newValue.toString());      
            DozeUtils.setPressUdfpsMode(getActivity(), value);
        }

        return true;
    }

    @Override
    public void onSwitchChanged(Switch switchView, boolean isChecked) {
        DozeUtils.enableDoze(getActivity(), isChecked);
        mSwitchBar.setChecked(isChecked);

        if (!isChecked) {
            DozeUtils.enableAlwaysOn(getActivity(), false);
            mAlwaysOnDisplayPreference.setChecked(false);
        }

        mAlwaysOnDisplayPreference.setEnabled(isChecked);
        mDoubleTapPreference.setEnabled(isChecked);
        mPressUdfpsPreference.setEnabled(isChecked);
    }
}
