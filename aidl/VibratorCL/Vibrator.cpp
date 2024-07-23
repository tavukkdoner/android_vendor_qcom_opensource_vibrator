/*
 * Copyright (c) 2022, 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the
 * disclaimer below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.

 *   * Neither the name of Qualcomm Innovation Center, Inc. nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
 * GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
 * HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#define LOG_TAG "vendor.qti.vibratorCL"

#include <dirent.h>
#include <inttypes.h>
#include <linux/input.h>
#include <log/log.h>
#include <string.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <thread>
#include <mutex>

#include "PalApi.h"
#include "PalDefs.h"
#include "rx_haptics_api.h"
#include "Vibrator.h"

namespace aidl {
namespace android {
namespace hardware {
namespace vibrator {

#define VIB_INVALID_VALUE           -1
#define WAKEUP_MIN_IDLE_CHECK   (1000 * 10)
#define MIN_EFFECT_TIME             50

static struct pal_stream_attributes stream_attributes;
static struct pal_device *pal_devices;
static pal_stream_handle_t *pal_stream_handle_;
uint8_t HapticsState = 2;

int32_t HapticsSetParameters(uint32_t param_mode, pal_param_haptics_cnfg_t payload);

std::mutex VibratorCL::EventMutex;
std::mutex VibratorCL::HapticsMutex;
std::condition_variable VibratorCL::cv;
std::condition_variable VibratorCL::Eventcv;
std::thread VibratorCL::OffThread;

bool VibratorCL::OffThrdCreated;
bool VibratorCL::ActiveUsecase = false;

VibratorCL::VibratorCL()
{
    mSupportGain = true;
    mSupportEffects = true;
    mSupportExternalControl = true;
}

VibratorCL::~VibratorCL()
{
}

/** Play vibration
 *
 *  @param effectId:  ID of the predefined effect will be played. If effectId is valid
 *                    (non-negative value), the timeoutMs value will be ignored, and the
 *                    real playing length will be set in param@playLengtMs and returned
 *                    to VibratorCLService. If effectId is invalid, value in param@timeoutMs
 *                    will be used as the play length for playing a constant effect.
 *  @param strenght:  Strength of the haptics predefined effect.
 *  @param timeoutMs: playing length, non-zero means playing, zero means stop playing.
 *  @param playLengthMs: the playing length in ms unit which will be returned to
 *                    VibratorCLService if the request is playing a predefined effect.
 *                    The custom_data in periodic is reused for returning the playLengthMs
 *                    from kernel space to userspace if the pattern is defined in kernel
 *                    driver. It's been defined with following format:
 *                       <effect-ID, play-time-in-seconds, play-time-in-milliseconds>.
 *                    The effect-ID is used for passing down the predefined effect to
 *                    kernel driver, and the rest two parameters are used for returning
 *                    back the real playing length from kernel driver.
 */
int VibratorCL::play(int effectId, int strength, long *playLengthMs, uint32_t timeoutMs) {

    int status = 0;
    pal_param_haptics_cnfg_t payload;

    int32_t no_of_devices = 1;
    stream_attributes.type = PAL_STREAM_HAPTICS;
    stream_attributes.direction = PAL_AUDIO_OUTPUT;
    stream_attributes.info.opt_stream_info.haptics_type = PAL_STREAM_HAPTICS_TOUCH;

    pal_devices = (struct pal_device *) calloc(no_of_devices, sizeof(struct pal_device));
    pal_devices[0].id = PAL_DEVICE_OUT_HAPTICS_DEVICE;
    pal_devices[0].config.bit_width = 16;
    pal_devices[0].config.sample_rate = 48000;
    pal_devices[0].config.ch_info.channels = 1;

    HapticsMutex.lock();
    ActiveUsecase = true;
    HapticsState = 0;
    cv.notify_all();

    if (pal_stream_handle_ == 0) {
        status = pal_stream_open(&stream_attributes, no_of_devices, pal_devices, 0, NULL,
              (pal_stream_callback) &VibratorCL::StreamHapticsCallback, 0, &pal_stream_handle_);
        if (status) {
            ALOGE("Error:Failed to open stream\n");
            goto exit;
        }
        ALOGD("Stream Opened successful\n");
    }

    payload.mode = PAL_STREAM_HAPTICS_TOUCH;
    payload.effect_id = effectId;
    payload.strength = strength;
    payload.time = timeoutMs;
    payload.amplitude = 0.5;
    payload.ch_mask = 1;
    status = HapticsSetParameters(PAL_PARAM_ID_HAPTICS_CNFG, payload);
    if (status) {
        ALOGD("Error:Failed to Set haptics wavegen param for haptics");
        goto exit;
    }

    status = pal_stream_start(pal_stream_handle_);
    if (status) {
        ALOGE("Error:Failed to Start haptics");
        goto close_stream;
    }
    goto exit;

close_stream:
    pal_stream_close(pal_stream_handle_);
    pal_stream_handle_ = NULL;

exit:
    HapticsMutex.unlock();
    return status;
}

void VibratorCL::offEffect() {
    int status = 0;

    if (pal_stream_handle_) {
       HapticsWait();
       if (!ActiveUsecase && pal_stream_handle_) {
          status = StopHapticsStream();
       }
    }
    OffThrdCreated = false;
    ALOGD("Offeffect exit");
}

int32_t VibratorCL::StopHapticsStream() {
    int status = 0;
    HapticsMutex.lock();
    status = pal_stream_stop(pal_stream_handle_);
    if (status) {
        ALOGE("Error:Failed to stop haptics stream");
    }
    status = pal_stream_close(pal_stream_handle_);
    if (status) {
        ALOGE("Error:Failed to close haptics stream");
    }
    pal_stream_handle_ = NULL;
    if (pal_devices)
       free(pal_devices);
    HapticsState = 2;
    HapticsMutex.unlock();
    return status;
}

int32_t HapticsSetParameters(uint32_t param_mode, pal_param_haptics_cnfg_t payload)
{
    int32_t status = -1;
    pal_param_payload *param_payload = NULL;

    switch (param_mode) {
       case PAL_PARAM_ID_HAPTICS_CNFG:
       {
          param_payload = (pal_param_payload *) calloc (1,
                                 sizeof(pal_param_payload) +
                                 sizeof(pal_param_haptics_cnfg_t));
          if (!param_payload)
              return status;

          param_payload->payload_size = sizeof(pal_param_haptics_cnfg_t);
          memcpy(param_payload->payload, &payload, param_payload->payload_size);
          status = pal_stream_set_param(pal_stream_handle_, param_mode, param_payload);
          break;
       }
       case PARAM_ID_HAPTICS_WAVE_DESIGNER_STOP_PARAM:
       {
           param_id_haptics_wave_designer_wave_designer_stop_param_t HapticsStopParam;
           param_payload = (pal_param_payload *) calloc (1,
                      sizeof(pal_param_payload) +
                      sizeof(param_id_haptics_wave_designer_wave_designer_stop_param_t));
           if (!param_payload)
              return status;

           HapticsStopParam.channel_mask = payload.ch_mask;
           param_payload->payload_size =
                          sizeof(param_id_haptics_wave_designer_wave_designer_stop_param_t);
           memcpy(param_payload->payload, &HapticsStopParam, param_payload->payload_size);
           status = pal_stream_set_param(pal_stream_handle_, param_mode, param_payload);
           break;
       }
       case PARAM_ID_HAPTICS_WAVE_DESIGNER_UPDATE_PARAM:
       {
           param_payload = (pal_param_payload *) calloc (1,
                               sizeof(pal_param_payload) +
                       sizeof(pal_param_haptics_cnfg_t));
           if (!param_payload)
              return status;

           param_payload->payload_size = sizeof(pal_param_haptics_cnfg_t);
           memcpy(param_payload->payload, &payload, param_payload->payload_size);
           status = pal_stream_set_param(pal_stream_handle_, param_mode, param_payload);
           break;
       }
       default:
             ALOGE("%s : Param_mode is undefined %d", __func__, param_mode);
          break;
   }
   if(param_payload)
      free(param_payload);
   return status;
}

int32_t VibratorCL::StreamHapticsCallback (uint64_t *stream_handle,
                uint32_t event_id, uint32_t *event_data, uint32_t event_size, uint64_t cookie)
{
    int32_t status = 0;
    ALOGE("event received from DSP %d", *event_data);
    Eventcv.notify_all();
    HapticsState = *event_data;
    return status;
}

int32_t VibratorCL::offCurrentEffect()
{
    int status = 0;
    pal_param_haptics_cnfg_t payload;

    if (pal_stream_handle_ && HapticsState == 0) {
        payload.ch_mask = 1;
        status = HapticsSetParameters(PARAM_ID_HAPTICS_WAVE_DESIGNER_STOP_PARAM,
                                       payload);
        if (status)
            ALOGD("Error:Failed to Set haptics stop param");
        else
            ALOGD("%s: stop effect successfull", __func__);
        HapticsState = 2;
    }
    else {
        ALOGD("%s: No current Effect is playing, skipping stop",__func__);
    }
    return status;
}

void VibratorCL::HapticsWait()
{
    std::unique_lock<std::mutex> lock(EventMutex);
    cv.wait_for(lock,
            std::chrono::milliseconds(WAKEUP_MIN_IDLE_CHECK));
}

void VibratorCL::HapticsWaitTillWaveformComp()
{
    std::unique_lock<std::mutex> eventlock(EventMutex);
    Eventcv.wait_for(eventlock,
            std::chrono::milliseconds(WAKEUP_MIN_IDLE_CHECK));

}

ndk::ScopedAStatus VibratorCL::getCapabilities(int32_t* _aidl_return) {
    *_aidl_return = IVibrator::CAP_ON_CALLBACK |IVibrator::CAP_PERFORM_CALLBACK |
                    IVibrator::CAP_AMPLITUDE_CONTROL |IVibrator::CAP_EXTERNAL_CONTROL;
    ALOGD("VibratorCL reporting capabilities: %d", *_aidl_return);

    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus VibratorCL::off() {
    int ret = 0;

    ALOGE("VibratorCL off ");
    ret = offCurrentEffect();
    if (ret)
        StopHapticsStream();

    cv.notify_all();
    ActiveUsecase = false;
    OffThread = std::thread (&VibratorCL::offEffect, this);
    OffThread.detach();
    OffThrdCreated = true;

exit:
    if (ret != 0)
        return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_SERVICE_SPECIFIC));

    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus VibratorCL::on(int32_t timeoutMs,
                                const std::shared_ptr<IVibratorCallback>& callback) {
    int ret = 0;

    if (ActiveUsecase) {
        ALOGE("VibratorCL ON: Haptics is already active skipping this instance");
        return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
    }

    ALOGE("VibratorCL on for timeoutMs %d", timeoutMs);

    ret = play(VIB_INVALID_VALUE, VIB_INVALID_VALUE, NULL, timeoutMs);

    if (ret != 0)
        return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_SERVICE_SPECIFIC));

    if (callback != nullptr) {
        std::thread([=] {
            ALOGD("Starting ON on another thread");
            HapticsWaitTillWaveformComp();
            ALOGD("Notifying on complete");
            if (!callback->onComplete().isOk()) {
                ALOGE("Failed to call onComplete");
            }
        }).detach();
    }

    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus VibratorCL::perform(Effect effect, EffectStrength es,
                       const std::shared_ptr<IVibratorCallback>& callback, int32_t* _aidl_return) {
    int ret;
    long playLengthMs;

    if (ActiveUsecase) {
        ALOGE("VibratorCL PERFORM: Haptics is already active skipping this instance");
        return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
    }

    ALOGE("VibratorCL perform effect %d", effect);

    if (effect < Effect::CLICK ||
            effect > Effect::HEAVY_CLICK)
        return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));

    if (es != EffectStrength::LIGHT && es != EffectStrength::MEDIUM && es != EffectStrength::STRONG)
        return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));

    ret = play((static_cast<int>(effect)), (static_cast<int>(es)), &playLengthMs, VIB_INVALID_VALUE);

    if (ret != 0)
        return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_SERVICE_SPECIFIC));

    if (callback != nullptr) {
        std::thread([=] {
            ALOGD("Starting perform on another thread");
            HapticsWaitTillWaveformComp();
            ALOGD("Notifying perform complete");
            callback->onComplete();
        }).detach();
    }

    *_aidl_return = MIN_EFFECT_TIME;
    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus VibratorCL::getSupportedEffects(std::vector<Effect>* _aidl_return) {
    *_aidl_return = {Effect::CLICK, Effect::DOUBLE_CLICK, Effect::TICK, Effect::THUD,
                     Effect::POP, Effect::HEAVY_CLICK};

    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus VibratorCL::setAmplitude(float amplitude) {

    pal_param_haptics_cnfg_t payload;
    int status = -1;
    ALOGD("VibratorCL set amplitude: %f", amplitude);

    if (amplitude <= 0.0f || amplitude > 1.0f)
        return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_ILLEGAL_ARGUMENT));

    payload.ch_mask = 1;
    payload.amplitude = amplitude;
    status = HapticsSetParameters(PARAM_ID_HAPTICS_WAVE_DESIGNER_UPDATE_PARAM, payload);
    if (status) {
        ALOGD("Error:Failed to Set update haptics param");
        return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_SERVICE_SPECIFIC));
    }

    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus VibratorCL::setExternalControl(bool enabled) {

    ALOGD("VibratorCL set external control: %d", enabled);
    if (!mSupportExternalControl)
        return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));

    return ndk::ScopedAStatus::ok();
}

ndk::ScopedAStatus VibratorCL::getCompositionDelayMax(int32_t* maxDelayMs  __unused) {
    return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
}

ndk::ScopedAStatus VibratorCL::getCompositionSizeMax(int32_t* maxSize __unused) {
    return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
}

ndk::ScopedAStatus VibratorCL::getSupportedPrimitives(std::vector<CompositePrimitive>* supported __unused) {
    return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
}

ndk::ScopedAStatus VibratorCL::getPrimitiveDuration(CompositePrimitive primitive __unused,
                                                  int32_t* durationMs __unused) {
    return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
}

ndk::ScopedAStatus VibratorCL::compose(const std::vector<CompositeEffect>& composite __unused,
                                     const std::shared_ptr<IVibratorCallback>& callback __unused) {
    return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
}

ndk::ScopedAStatus VibratorCL::getSupportedAlwaysOnEffects(std::vector<Effect>* _aidl_return __unused) {
    return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
}

ndk::ScopedAStatus VibratorCL::alwaysOnEnable(int32_t id __unused, Effect effect __unused,
                                            EffectStrength strength __unused) {
    return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
}

ndk::ScopedAStatus VibratorCL::alwaysOnDisable(int32_t id __unused) {
    return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
}

ndk::ScopedAStatus VibratorCL::getResonantFrequency(float *resonantFreqHz __unused) {
    return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
}

ndk::ScopedAStatus VibratorCL::getQFactor(float *qFactor __unused) {
    return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
}

ndk::ScopedAStatus VibratorCL::getFrequencyResolution(float *freqResolutionHz __unused) {
    return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
}

ndk::ScopedAStatus VibratorCL::getFrequencyMinimum(float *freqMinimumHz __unused) {
    return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
}

ndk::ScopedAStatus VibratorCL::getBandwidthAmplitudeMap(std::vector<float> *_aidl_return __unused) {
    return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
}

ndk::ScopedAStatus VibratorCL::getPwlePrimitiveDurationMax(int32_t *durationMs __unused) {
    return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
}

ndk::ScopedAStatus VibratorCL::getPwleCompositionSizeMax(int32_t *maxSize __unused) {
    return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
}

ndk::ScopedAStatus VibratorCL::getSupportedBraking(std::vector<Braking> *supported __unused) {
    return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
}

ndk::ScopedAStatus VibratorCL::composePwle(const std::vector<PrimitivePwle> &composite __unused,
                           const std::shared_ptr<IVibratorCallback> &callback __unused) {
    return ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_UNSUPPORTED_OPERATION));
}

}  // namespace vibrator
}  // namespace hardware
}  // namespace android
}  // namespace aidl