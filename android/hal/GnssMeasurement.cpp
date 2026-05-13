#define LOG_TAG "GPSd_HAL"

#include "GnssMeasurement.h"
#include <log/log.h>

namespace android {
namespace hardware {
namespace gnss {
namespace V1_1 {
namespace implementation {

sp<::android::hardware::gnss::V1_1::IGnssMeasurementCallback> GnssMeasurement::sGnssMeasurementCallback = nullptr;

// Methods from ::android::hardware::gnss::V1_0::IGnssMeasurement follow.
Return<::android::hardware::gnss::V1_0::IGnssMeasurement::GnssMeasurementStatus>
GnssMeasurement::setCallback(const sp<::android::hardware::gnss::V1_0::IGnssMeasurementCallback>&)
{
    ALOGD("GnssMeasurement::setCallback");

    return GnssMeasurementStatus::ERROR_GENERIC;
}

Return<void> GnssMeasurement::close()
{
    ALOGD("GnssMeasurement::close");

    std::unique_lock<std::mutex> lock(mMutex);
    sGnssMeasurementCallback = nullptr;

    // TODO implement
    return Void();
}

// Methods from ::android::hardware::gnss::V1_1::IGnssMeasurement follow.
Return<::android::hardware::gnss::V1_0::IGnssMeasurement::GnssMeasurementStatus>
GnssMeasurement::setCallback_1_1(
    const sp<::android::hardware::gnss::V1_1::IGnssMeasurementCallback>& callback,
    bool enable_full_tracking)
{
    ALOGD("GnssMeasurement::setCallback_1_1 full_tracking=%s",
          enable_full_tracking ? "enabled" : "disabled");

    std::unique_lock<std::mutex> lock(mMutex);
    sGnssMeasurementCallback = callback;

    return GnssMeasurementStatus::SUCCESS;
}

// Methods from ::android::hidl::base::V1_0::IBase follow.

}  // namespace implementation
}  // namespace V1_1
}  // namespace gnss
}  // namespace hardware
}  // namespace android
