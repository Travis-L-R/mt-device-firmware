#pragma once
#include "NodeDB.h"

class DisplayFormatters
{
  public:
    static const char *getModemPresetDisplayName(meshtastic_LoRaConfig_ModemPreset preset, bool useShortName);
};
