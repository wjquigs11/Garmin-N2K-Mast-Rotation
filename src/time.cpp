#ifdef NTP
#include "include.h"

bool ntpSyncSuccessful = false;

void setupTime() {
    // Configure NTP
    configTime(0, 0, "pool.ntp.org", "time.nist.gov", "time.google.com");
    
    // Set timezone - adjust for your location
    // See https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv for TZ strings
    setenv("TZ", "PST8PDT,M3.2.0,M11.1.0", 1); // Pacific Time
    tzset();
}

bool waitForTimeSync(int timeoutSeconds) {
    log::toAll("Waiting for NTP time sync...");
    time_t now = time(nullptr);
    int retry = 0;
    const int maxRetries = timeoutSeconds * 2; // 500ms delay per retry
    
    // Wait until we get a time after Jan 1, 2020
    while (now < 1577836800 && retry < maxRetries) {
        log::toAll("Waiting for NTP time sync...");
        delay(500);
        now = time(nullptr);
        retry++;
    }
    
    if (now > 1577836800) {
        struct tm timeinfo;
        localtime_r(&now, &timeinfo);
        char timeStringBuff[50];
        strftime(timeStringBuff, sizeof(timeStringBuff), "%A, %B %d %Y %H:%M:%S", &timeinfo);
        log::toAll("Current time: " + String(timeStringBuff));
        ntpSyncSuccessful = true;
        return true;
    } else {
        log::toAll("Failed to get time from NTP server");
        ntpSyncSuccessful = false;
        return false;
    }
}

String getFormattedTime() {
    time_t now;
    time(&now);
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    char timeStringBuff[50];
    strftime(timeStringBuff, sizeof(timeStringBuff), "%Y-%m-%d %H:%M:%S", &timeinfo);
    return String(timeStringBuff);
}

unsigned long getEpochTime() {
    time_t now;
    time(&now);
    return now;
}

bool isNtpSyncSuccessful() {
    return ntpSyncSuccessful;
}

void resyncNTP() {
    log::toAll("Resyncing NTP time...");
    configTime(0, 0, "pool.ntp.org", "time.nist.gov", "time.google.com");
    
    // Re-apply timezone settings to prevent defaulting to GMT
    setenv("TZ", "PST8PDT,M3.2.0,M11.1.0", 1); // Pacific Time
    tzset();
    
    waitForTimeSync(5); // Try for 5 seconds
}
#endif