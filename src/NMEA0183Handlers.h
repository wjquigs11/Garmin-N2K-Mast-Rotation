#ifdef NMEA0183
 
#ifndef _NMEA0183Handlers_H_
#define _NMEA0183Handlers_H_
#include <Arduino.h>
#include <Time.h>
#include <NMEA0183.h>
#include <NMEA0183Msg.h>
#include <NMEA2000.h>

void InitNMEA0183Handlers(tNMEA2000 *_NMEA2000, tBoatData *_BoatData);
void DebugNMEA0183Handlers(Stream* _stream);

void HandleNMEA0183Msg(const tNMEA0183Msg &NMEA0183Msg);

#endif
#endif

