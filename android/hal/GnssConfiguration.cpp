#define LOG_TAG "GPSd_HAL"

#include "GnssConfiguration.h"
#include <log/log.h>

namespace android {
namespace hardware {
namespace gnss {
namespace V1_1 {
namespace implementation {

// Methods from ::android::hardware::gnss::V1_0::IGnssConfiguration follow.
Return<bool> GnssConfiguration::setSuplEs(bool enabled) {
    ALOGD("GnssConfiguration::setSuplEs %s", enabled ? "enabled" : "disabled");
    // TODO implement
    return true;
}

Return<bool> GnssConfiguration::setSuplVersion(uint32_t version) {
    ALOGD("GnssConfiguration::setSuplVersion %d", version);
    // TODO implement
    return true;
}

Return<bool> GnssConfiguration::setSuplMode(hidl_bitfield<SuplMode> mode) {
    ALOGD("GnssConfiguration::setSuplMode 0x%02x", mode);
    // TODO implement
    return true;
}

Return<bool> GnssConfiguration::setGpsLock(hidl_bitfield<GpsLock> lock) {
    ALOGD("GnssConfiguration::setGpsLock 0x%02x", lock);
    // TODO implement
    return true;
}

Return<bool> GnssConfiguration::setLppProfile(hidl_bitfield<LppProfile> profile) {
    ALOGD("GnssConfiguration::setLppProfile 0x%02x", profile);
    // TODO implement
    return true;
}

Return<bool> GnssConfiguration::setGlonassPositioningProtocol(hidl_bitfield<GlonassPosProtocol> protocol) {
    ALOGD("GnssConfiguration::setGlonassPositioningProtocol 0x%02x", protocol);
    // TODO implement
    return true;
}

Return<bool> GnssConfiguration::setEmergencySuplPdn(bool enable) {
    ALOGD("GnssConfiguration::setEmergencySuplPdn %s", enable ? "enable" : "disable");
    // TODO implement
    return true;
}

// Methods from ::android::hardware::gnss::V1_1::IGnssConfiguration follow.
Return<bool> GnssConfiguration::setBlacklist(const hidl_vec<BlacklistedSource>& sourceList) {
    ALOGD("GnssConfiguration::setBlacklist");
    std::unique_lock<std::recursive_mutex> lock(mMutex);
    mBlacklistedConstellationSet.clear();
    mBlacklistedSourceSet.clear();
    for (auto source : sourceList) {
        if (source.svid == 0) {
            // Wildcard blacklist, i.e., blacklist entire constellation.
            mBlacklistedConstellationSet.insert(source.constellation);
            ALOGD("  blacklist constellation 0x%02hhx", source.constellation);
        } else {
            mBlacklistedSourceSet.insert(source);
            ALOGD("  blacklist SV 0x%02x", source.svid);
        }
    }
    return true;
}

Return<bool> GnssConfiguration::isBlacklisted(const GnssSvInfo& gnssSvInfo) const {
    std::unique_lock<std::recursive_mutex> lock(mMutex);
    if (mBlacklistedConstellationSet.find(gnssSvInfo.constellation) !=
        mBlacklistedConstellationSet.end()) {
        return true;
    }
    BlacklistedSource source = {.constellation = gnssSvInfo.constellation, .svid = gnssSvInfo.svid};
    return (mBlacklistedSourceSet.find(source) != mBlacklistedSourceSet.end());
}

// Methods from ::android::hidl::base::V1_0::IBase follow.

}  // namespace implementation
}  // namespace V1_1
}  // namespace gnss
}  // namespace hardware
}  // namespace android
