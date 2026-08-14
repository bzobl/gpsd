#define LOG_TAG "GPSd_HAL"

#include <android/hardware/gnss/1.0/types.h>
#include <log/log.h>
#include <cutils/properties.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "Gnss.h"

#include "include/timespec.h"

namespace android {
namespace hardware {
namespace gnss {
namespace V1_1 {
namespace implementation {

using GnssSvFlags = IGnssCallback::GnssSvFlags;

sp<::android::hardware::gnss::V1_1::IGnssCallback> Gnss::sGnssCallback = nullptr;

Gnss::Gnss() :
    mMinIntervalMs(1000),
    mGnssConfiguration{new GnssConfiguration()},
    mGnssMeasurement{new GnssMeasurement()},
    mIsActive(false) {}

Gnss::~Gnss() {
    mIsActive = false;
    if (mThread.joinable()) {
        mThread.join();
    }
}

// Methods from ::android::hardware::gnss::V1_0::IGnss follow.
Return<bool> Gnss::setCallback(const sp<::android::hardware::gnss::V1_0::IGnssCallback>&) {
    // Mock handles only new callback (see setCallback1_1) coming from Android P+
    return false;
}

Return<bool> Gnss::start() {
    ALOGD("Gnss::start()");

    if (mIsActive) {
        ALOGW("Gnss has started. Restarting...");
        stop();
    }

    mIsActive = true;
    mThread = std::thread([this]() {

        std::ostringstream oss;
        oss << std::this_thread::get_id();
        ALOGE("GPS THREAD STARTING %s", oss.str().c_str());

        struct gps_data_t gps_data;
        int gpsopen = -1;
        char gpsdhost[PROP_VALUE_MAX];
        char gpsdport[PROP_VALUE_MAX];
        char gpsdauto[PROP_VALUE_MAX];
        int is_automotive;
        char gpslat[PROP_VALUE_MAX];
        char gpslon[PROP_VALUE_MAX];
        time_t last_recorded_fix = 0;
        char dtos[100];
        GnssLocation location = {};
        time_t last_report_ms = 0;

        // Normally, GPSd will be running on localhost, but we can set a system property
        // "service.gpsd.host" to some other hostname in order to open a GPSd instance
        // running on a different host.
        property_get("service.gpsd.host", gpsdhost, "localhost");
        property_get("service.gpsd.port", gpsdport, "2947");
        is_automotive = (property_get("service.gpsd.automotive", gpsdauto, "") > 0);

        // Load coordinates stored in persist properties as current location
        // This is to provide instantaneous fix to the last good location
        // in order to provide instantaneous ability to begin navigator routing.
        if (is_automotive && property_get("persist.service.gpsd.latitude", gpslat, "") > 0
                          && property_get("persist.service.gpsd.longitude", gpslon, "") > 0){
              location.gnssLocationFlags = 0;
              location.gnssLocationFlags |= V1_0::GnssLocationFlags::HAS_LAT_LONG;
              location.latitudeDegrees = atof(gpslat);
              location.longitudeDegrees = atof(gpslon);
              location.altitudeMeters = 0.0;
              location.speedMetersPerSec = 0.0;
              location.bearingDegrees = 0.0;
              location.horizontalAccuracyMeters = 0.0;
              location.verticalAccuracyMeters = 0.0;
              location.speedAccuracyMetersPerSecond = 0.0;
              location.bearingAccuracyDegrees = 0.0;
              location.timestamp = (int64_t)time(NULL) * 1000;
              this->reportLocation(location);
          }

          memset(&gps_data, 0, sizeof(gps_data));

          while (mIsActive == true) {
              // If the connection to GPSd is not open, try to open it.
              // If the attempt to open it fails, sleep 5 seconds and try again.
              if (gpsopen != 0){
                  ALOGD("%s: gpsd_host: %s, gpsd_port: %s", __func__, gpsdhost, gpsdport);
                  gpsopen = gps_open(gpsdhost, gpsdport, &gps_data);
                  if (gpsopen != 0) {
                      ALOGW("%s: gps_open FAIL (%d). Trying again in 5 seconds.", __func__, gpsopen);
                      sleep(5);
                      continue;
                  }

                  ALOGV("%s: gps_open SUCCESS", __func__);
                  if (gps_stream(&gps_data, WATCH_ENABLE, NULL) != 0) {
                    ALOGW("gps_stream failed: %s", strerror(errno));
                    gps_close(&gps_data);
                    gpsopen = -1;
                    continue;
                  }
              }


              // Wait for data from gpsd, then process it.
              if (!gps_waiting(&gps_data, 2000000)) {
                  ALOGW("waiting for gps data timed out");
                  if (gps_data.set & ERROR_SET) {
                    ALOGE("gps_data error: %s", gps_data.error);
                  }
                  continue;
              }

              errno = 0;
              int read = gps_read(&gps_data, NULL, 0);
              if (read != 0) {
                char error_str[256];
                ALOGW("reading from gps socket failed: %s",
                      (read == -2) ? "EOF" : strerror_r(errno, error_str, sizeof(error_str)));

                gps_close(&gps_data);
                gpsopen = -1;
                continue;
              }

              ALOGD("set=0x%012lx, set_pending=0x%012lx. "
                    "Fix: status=%d, mode=%d, time=%ld, lat=%e, lon=%e, alt=%e, speed=%e, track=%e "
                    "Accuracy: h=%e, v=%e, speed=%e, track=%e "
                    "using %d/%d satellites. device used %.128s",
                    gps_data.set, gps_data.set_pending,
                    gps_data.fix.status, gps_data.fix.mode, gps_data.fix.time.tv_sec,
                    gps_data.fix.latitude, gps_data.fix.longitude, gps_data.fix.altHAE,
                    gps_data.fix.speed, gps_data.fix.track,
                    gps_data.fix.eph, gps_data.fix.epv, gps_data.fix.eps, gps_data.fix.epd,
                    gps_data.satellites_used, gps_data.satellites_visible,
                    gps_data.dev.path);

              if (gps_data.fix.mode >= MODE_2D) {

                  location.gnssLocationFlags = 0;
                  location.timestamp = 0;

                  if ((gps_data.set & TIME_SET)
                      && (gps_data.set & LATLON_SET)
                      && (gps_data.set & HERR_SET)) {
                      location.timestamp = (int64_t)gps_data.fix.time.tv_sec * 1000 +
                                           gps_data.fix.time.tv_nsec / 1000000;

                      location.gnssLocationFlags |= V1_0::GnssLocationFlags::HAS_LAT_LONG;
                      location.latitudeDegrees = gps_data.fix.latitude;
                      location.longitudeDegrees = gps_data.fix.longitude;

                      location.gnssLocationFlags |= V1_0::GnssLocationFlags::HAS_HORIZONTAL_ACCURACY;
                      location.horizontalAccuracyMeters = gps_data.fix.eph;

                      // Every 30 seconds, store current coordinates to persist property.
                      if (is_automotive &&
                          gps_data.fix.time.tv_sec > last_recorded_fix + 30){
                          last_recorded_fix = gps_data.fix.time.tv_sec;
                          snprintf(dtos, sizeof(dtos), "%lf", gps_data.fix.latitude);
                          property_set("persist.service.gpsd.latitude", dtos);
                          snprintf(dtos, sizeof(dtos), "%lf", gps_data.fix.longitude);
                          property_set("persist.service.gpsd.longitude", dtos);
                      }
                  }

                  if ((gps_data.set & SPEED_SET) && (gps_data.set & SPEEDERR_SET)) {
                      location.gnssLocationFlags |= V1_0::GnssLocationFlags::HAS_SPEED;
                      location.speedMetersPerSec = gps_data.fix.speed;

                      location.gnssLocationFlags |= V1_0::GnssLocationFlags::HAS_SPEED_ACCURACY;
                      location.speedAccuracyMetersPerSecond = gps_data.fix.eps;
                  }

                  if ((gps_data.set & TRACK_SET) && (gps_data.set & TRACKERR_SET)) {
                      location.gnssLocationFlags |= V1_0::GnssLocationFlags::HAS_BEARING;
                      location.bearingDegrees = gps_data.fix.track;

                      location.gnssLocationFlags |= V1_0::GnssLocationFlags::HAS_BEARING_ACCURACY;
                      location.bearingAccuracyDegrees = gps_data.fix.epd;
                  }

                  if ((gps_data.set & ALTITUDE_SET) && (gps_data.set & VERR_SET)) {
                      location.gnssLocationFlags |= V1_0::GnssLocationFlags::HAS_ALTITUDE;
                      location.altitudeMeters = gps_data.fix.altHAE;

                      location.gnssLocationFlags |= V1_0::GnssLocationFlags::HAS_VERTICAL_ACCURACY;
                      location.verticalAccuracyMeters = gps_data.fix.epv;
                  }

                  if ((location.timestamp == 0)
                      ||!(location.gnssLocationFlags & V1_0::GnssLocationFlags::HAS_LAT_LONG)
                      //|| !(location.gnssLocationFlags & V1_0::GnssLocationFlags::HAS_SPEED)
                      //|| !(location.gnssLocationFlags & V1_0::GnssLocationFlags::HAS_BEARING)
                      || !(location.gnssLocationFlags & V1_0::GnssLocationFlags::HAS_ALTITUDE)
                      //|| !(gps_data.set & SATELLITE_SET)
                      //|| (gps_data.satellites_used == 0)
                      ) {
                      ALOGI("will not report location: don't have all data (have 0x%02x)",
                            location.gnssLocationFlags);
                  } else if ((last_report_ms + mMinIntervalMs) > location.timestamp) {
                      ALOGI("will not report location: too early now: %ld, next report due: %ld",
                            location.timestamp, last_report_ms + mMinIntervalMs);
                  } else {
                      last_report_ms = location.timestamp;

                      gps_data.set &= ~(LATLON_SET | HERR_SET | SPEED_SET | SPEEDERR_SET
                                        | TRACK_SET | TRACKERR_SET | ALTITUDE_SET | VERR_SET
                                        | TIME_SET);
                      gps_clear_fix(&gps_data.fix);
                      this->reportLocation(location);
                  }
              }

              if ((gps_data.set & SATELLITE_SET) && (gps_data.satellites_visible > 0)) {
                  GnssSvStatus svStatus = { };
                  memset(&svStatus, 0, sizeof(svStatus));

                  svStatus.numSvs = gps_data.satellites_visible;
                  if (svStatus.numSvs > svStatus.gnssSvList.size()) {
                      svStatus.numSvs = svStatus.gnssSvList.size();
                  }

                  for (uint32_t i = 0; i < svStatus.numSvs; i++){
                      GnssSvInfo &sv_info = svStatus.gnssSvList[i];
                      const satellite_t &sat = gps_data.skyview[i];

                      ALOGV("Satelite[%d] svid=0x%02x, constellation=%d, used=%s, "
                            "elevation=%f, azimuth=%f, SNR=%f, carrier freq=%d",
                            i, sat.svid, sat.gnssid, sat.used ? "yes" : "no",
                            sat.elevation, sat.azimuth, sat.ss, sat.freqid);

                      sv_info.svid = sat.svid;
                      switch (sat.gnssid) {
                          case GNSSID_GPS:
                              sv_info.constellation = GnssConstellationType::GPS;
                              if ((sv_info.svid < 1) || (sv_info.svid > 32)) {
                                ALOGW("svid 0x%02x for GPS out of range", sv_info.svid);
                              }
                              break;
                          case GNSSID_SBAS:
                              sv_info.constellation = GnssConstellationType::SBAS;
                              if ((sv_info.svid < 120) || (sv_info.svid > 192)
                                  || ((sv_info.svid > 151) && (sv_info.svid < 183))) {
                                ALOGW("svid 0x%02x for SBAS out of range", sv_info.svid);
                              }
                              break;
                          case GNSSID_GAL:
                              sv_info.constellation = GnssConstellationType::GALILEO;
                              if ((sv_info.svid < 1) || (sv_info.svid > 36)) {
                                ALOGW("svid 0x%02x for GALILEO out of range", sv_info.svid);
                              }
                              break;
                          case GNSSID_BD:
                              sv_info.constellation = GnssConstellationType::BEIDOU;
                              if ((sv_info.svid < 1) || (sv_info.svid > 37)) {
                                ALOGW("svid 0x%02x for BEIDOU out of range", sv_info.svid);
                              }
                              break;
                          case GNSSID_QZSS:
                              sv_info.constellation = GnssConstellationType::QZSS;
                              if ((sv_info.svid < 193) || (sv_info.svid > 200)) {
                                ALOGW("svid 0x%02x for QZSS out of range", sv_info.svid);
                              }
                              break;
                          case GNSSID_GLO:
                              sv_info.constellation = GnssConstellationType::GLONASS;
                              if ((sv_info.svid < 1) || (sv_info.svid > 106)
                                  || ((sv_info.svid > 24) && (sv_info.svid < 93))) {
                                ALOGW("svid 0x%02x for GLONASS out of range", sv_info.svid);
                              }
                              break;
                          case GNSSID_IMES:
                          case GNSSID_IRNSS:
                          case GNSSID_CNT:
                          default:
                              ALOGW("satellite with unknown constellation %d", sat.gnssid);
                              sv_info.constellation = GnssConstellationType::UNKNOWN;
                              break;
                      }
                      sv_info.cN0Dbhz = sat.ss > 0 ? sat.ss : 0;
                      sv_info.elevationDegrees = 0;
                      sv_info.azimuthDegrees = 0;
                      sv_info.carrierFrequencyHz = 0;
                      sv_info.svFlag = 0;

                      if (!std::isnan(sat.elevation) && !std::isnan(sat.azimuth)) {
                          sv_info.elevationDegrees = sat.elevation;
                          sv_info.azimuthDegrees = sat.azimuth;
                          sv_info.svFlag |= GnssSvFlags::HAS_EPHEMERIS_DATA
                                            | GnssSvFlags::HAS_ALMANAC_DATA;
                      }

                      if (mGnssConfiguration->isBlacklisted(sv_info)) {
                          ALOGI("SV 0x%02x is blacklisted", sat.svid);
                      } else if (sat.used) {
                          sv_info.svFlag |= GnssSvFlags::USED_IN_FIX;
                      }
                      ALOGV("SvInfo[%d] svid=0x%02x, constellation=%hhd, used=%s, "
                            "elevation=%f, azimuth=%f, SNR=%f, carrier freq=%f",
                            i, sv_info.svid, sv_info.constellation,
                            sv_info.svFlag & GnssSvFlags::USED_IN_FIX ? "yes" : "no",
                            sv_info.elevationDegrees, sv_info.azimuthDegrees,
                            sv_info.cN0Dbhz, sv_info.carrierFrequencyHz);
                  }
                  this->reportSvStatus(svStatus);
                  gps_data.set &= ~SATELLITE_SET;
              }
          }

          // Close the GPS if it was successfully opened.
          if (gpsopen == 0) {
              gps_stream(&gps_data, WATCH_DISABLE, NULL);
              gps_close(&gps_data);
          }
          ALOGE("GPS THREAD STOPPED %s", oss.str().c_str());
    });

    std::ostringstream oss;
    oss << mThread.get_id();
    ALOGE("started thread %s", oss.str().c_str());

    return true;
}

Return<bool> Gnss::stop() {
    ALOGD("Gnss::stop()");
    mIsActive = false;
    if (mThread.joinable()) { 
        std::ostringstream oss;
        oss << mThread.get_id();
        ALOGE("joining thread %s", oss.str().c_str());
        mThread.join();
    }

    return true;
}

Return<void> Gnss::cleanup() {
    ALOGD("Gnss::cleanup()");

    if (mIsActive) {
      auto status = this->stop();
      if (!status.isOk()) {
        ALOGE("stopping thread failed");
      }
    }

    std::unique_lock<std::mutex> lock(mMutex);
    sGnssCallback = nullptr;

    return Void();
}

Return<bool> Gnss::injectTime(int64_t timeMs, int64_t timeReferenceMs, int32_t uncertaintyMs) {
    ALOGD("not yet implemented: Gnss::injectTime(time: %ldms, reference: %ldms, uncertainty: %dms)",
          timeMs, timeReferenceMs, uncertaintyMs);
    return false;
}

Return<bool> Gnss::injectLocation(double latitudeDegrees, double longitudeDegrees,
                                  float accuracyMeters) {
    ALOGD("not implemented: Gnss::injectLocation(lat: %f, lon: %f, accuracy: %f)",
          latitudeDegrees, longitudeDegrees, accuracyMeters);
    return false;
}

Return<void> Gnss::deleteAidingData(::android::hardware::gnss::V1_0::IGnss::GnssAidingData) {
    ALOGD("not implemented: Gnss::deleteAidingData()");
    return Void();
}

Return<bool> Gnss::setPositionMode(GnssPositionMode mode, GnssPositionRecurrence recurrence,
                                   uint32_t minIntervalMs, uint32_t preferredAccuracyMeters,
                                   uint32_t preferredTimeMs) {
    ALOGD("Gnss::setPositionMode(mode: %s, recurrence: %s, minInterval: %dms, prefAccuracy=%dm, "
          "prefTime=%dms",
          mode == GnssPositionMode::MS_BASED ? "MS_BASED"
          : mode == GnssPositionMode::MS_ASSISTED ? "MS_ASSISTED" : "STANDALONE", 
          recurrence == GnssPositionRecurrence::RECURRENCE_SINGLE ? "SINGLE" : "PERIODIC",
          minIntervalMs, preferredAccuracyMeters, preferredTimeMs);

    std::unique_lock<std::mutex> lock(mMutex);
    mMinIntervalMs = minIntervalMs;

    return true;
}

Return<sp<::android::hardware::gnss::V1_0::IAGnssRil>> Gnss::getExtensionAGnssRil() {
    ALOGD("not implemented: Gnss::getExtensionAGnssRil()");
    return ::android::sp<::android::hardware::gnss::V1_0::IAGnssRil>{};
}

Return<sp<::android::hardware::gnss::V1_0::IGnssGeofencing>> Gnss::getExtensionGnssGeofencing() {
    ALOGD("not implemented: Gnss::getExtensionGnssGeofencing()");
    return ::android::sp<::android::hardware::gnss::V1_0::IGnssGeofencing>{};
}

Return<sp<::android::hardware::gnss::V1_0::IAGnss>> Gnss::getExtensionAGnss() {
    ALOGD("not implemented: Gnss::getExtensionAGnss()");
    return ::android::sp<::android::hardware::gnss::V1_0::IAGnss>{};
}

Return<sp<::android::hardware::gnss::V1_0::IGnssNi>> Gnss::getExtensionGnssNi() {
    ALOGD("not implemented: Gnss::getExtensionGnssNi()");
    return ::android::sp<::android::hardware::gnss::V1_0::IGnssNi>{};
}

Return<sp<::android::hardware::gnss::V1_0::IGnssMeasurement>> Gnss::getExtensionGnssMeasurement() {
    ALOGD("not implemented: Gnss::getExtensionGnssMeasurement()");
    return mGnssMeasurement;
}

Return<sp<::android::hardware::gnss::V1_0::IGnssNavigationMessage>>
Gnss::getExtensionGnssNavigationMessage() {
    ALOGD("not implemented: Gnss::getExtensionGnssNavigationMessage()");
    return ::android::sp<::android::hardware::gnss::V1_0::IGnssNavigationMessage>{};
}

Return<sp<::android::hardware::gnss::V1_0::IGnssXtra>> Gnss::getExtensionXtra() {
    ALOGD("not implemented: Gnss::getExtensionXtra()");
    return ::android::sp<::android::hardware::gnss::V1_0::IGnssXtra>{};
}

Return<sp<::android::hardware::gnss::V1_0::IGnssConfiguration>>
Gnss::getExtensionGnssConfiguration() {
    ALOGD("Gnss::getExtensionGnssConfiguration()");
    return mGnssConfiguration;
}

Return<sp<::android::hardware::gnss::V1_0::IGnssDebug>> Gnss::getExtensionGnssDebug() {
    ALOGD("not implemented: Gnss::getExtensionGnssDebug()");
    return ::android::sp<::android::hardware::gnss::V1_0::IGnssDebug>{};
}

Return<sp<::android::hardware::gnss::V1_0::IGnssBatching>> Gnss::getExtensionGnssBatching() {
    ALOGD("not implemented: Gnss::getExtensionGnssBatching()");
    return ::android::sp<::android::hardware::gnss::V1_0::IGnssBatching>{};
}

// Methods from ::android::hardware::gnss::V1_1::IGnss follow.
Return<bool> Gnss::setCallback_1_1(
    const sp<::android::hardware::gnss::V1_1::IGnssCallback>& callback) {
    ALOGD("Gnss::setCallback_1_1()");
    if (callback == nullptr) {
        ALOGE("%s: null callback ignored", __func__);
        return false;
    }

    std::unique_lock<std::mutex> lock(mMutex);
    sGnssCallback = callback;

    uint32_t capabilities = 0x0;
    auto ret = sGnssCallback->gnssSetCapabilitesCb(capabilities);
    if (!ret.isOk()) {
        ALOGE("%s: unable to invoke callback", __func__);
        return false;
    }

    IGnssCallback::GnssSystemInfo gnssInfo = {.yearOfHw = 2018};

    ret = sGnssCallback->gnssSetSystemInfoCb(gnssInfo);
    if (!ret.isOk()) {
        ALOGE("%s: unable to invoke callback", __func__);
        return false;
    }

    auto gnssName = "GPSd GNSS Implementation v1.1";
    ret = sGnssCallback->gnssNameCb(gnssName);
    if (!ret.isOk()) {
        ALOGE("%s: unable to invoke callback", __func__);
        return false;
    }

    return true;
}

Return<bool> Gnss::setPositionMode_1_1(GnssPositionMode mode, GnssPositionRecurrence recurrence,
    uint32_t minIntervalMs, uint32_t preferredAccuracyMeters, uint32_t preferredTimeMs,
    bool low_power_mode) {
    ALOGD("Gnss::setPositionMode_1_1(mode: %s, recurrence: %s, minInterval: %dms, "
          "prefAccuracy: %dm, prefTime: %dms, low_power_mode: %s",
          mode == GnssPositionMode::MS_BASED ? "MS_BASED"
          : mode == GnssPositionMode::MS_ASSISTED ? "MS_ASSISTED" : "STANDALONE", 
          recurrence == GnssPositionRecurrence::RECURRENCE_SINGLE ? "SINGLE" : "PERIODIC",
          minIntervalMs, preferredAccuracyMeters, preferredTimeMs, low_power_mode ? "on" : "off");

    std::unique_lock<std::mutex> lock(mMutex);
    mMinIntervalMs = minIntervalMs;
    return true;
}

Return<sp<::android::hardware::gnss::V1_1::IGnssConfiguration>>
Gnss::getExtensionGnssConfiguration_1_1() {
    ALOGD("Gnss::getExtensionGnssConfiguration_1_1()");
    return mGnssConfiguration;
}

Return<sp<::android::hardware::gnss::V1_1::IGnssMeasurement>>
Gnss::getExtensionGnssMeasurement_1_1() {
    ALOGD("Gnss::getExtensionGnssMeasurement_1_1()");
    return mGnssMeasurement;
}

Return<bool> Gnss::injectBestLocation(const GnssLocation&) {
    ALOGD("Gnss::injectBestLocation()");
    return true;
}

void Gnss::reportLocation(const GnssLocation& location) const {
    if (!mIsActive) {
        ALOGI("will not report location, GPS inactive");
        return;
    }

    std::unique_lock<std::mutex> lock(mMutex);
    if (sGnssCallback == nullptr) {
        ALOGE("will not report location, sGnssCallback is null");
        return;
    }

    ALOGV("calling gnssLocationCb() callback to report location");
    auto status = sGnssCallback->gnssLocationCb(location);
    if (!status.isOk()) {
        ALOGE("reporting location failed");
    }
}

void Gnss::reportSvStatus(const GnssSvStatus& svStatus) const {
    if (!mIsActive) {
        ALOGI("will not report SV status, GPS inactive");
        return;
    }

    std::unique_lock<std::mutex> lock(mMutex);
    if (sGnssCallback == nullptr) {
        ALOGI("will not report SV status, sGnssCallback is null");
        return;
    }

    ALOGV("calling gnssSvStatusCb() callback to report SV status");
    auto status = sGnssCallback->gnssSvStatusCb(svStatus);
    if (!status.isOk()) {
        ALOGE("reporting SV status failed");
    }
}

}  // namespace implementation
}  // namespace V1_1
}  // namespace gnss
}  // namespace hardware
}  // namespace android
