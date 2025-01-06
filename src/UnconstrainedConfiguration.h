// With LEGACY_PREFERENCES defined, selected older default setting will be reverted (or adjusted)
#define LEGACY_PREFERENCES

// Enable all 
#ifdef LEGACY_PREFERENCES
#define ENABLE_NEIGHBOR_INFO_BY_DEFAULT
#endif

// In UNCONSTRAINED_MODE, some ordinarily-appropriate limitations will be removed
#define UNCONSTRAINED_MODE

// Set all controls used for UNCONSTRAINED_MODE
#ifdef UNCONSTRAINED_MODE
#define ALLOW_NEIGHBOR_INFO_ON_DEFAULT_CHANNEL
#define ALLOW_NODENUM_ASSIGNMENT  // Allow devices to set the nodenum on outgoing packets
#define TRANSGRESS_OK_TO_MQTT  // Disregard OK_TO_MQTT bitfield flag when not using default MQTT
#endif

// In DATA_LOGGING_MODE, remove additional limitations so that we can get more info for analytics
#define DATA_LOGGING_MODE


// Set all controls used for DATA_LOGGING_MODE
#ifdef DATA_LOGGING_MODE

#define UPLINK_ALL_CHANNELS

#ifndef TRANSGRESS_OK_TO_MQTT
#define TRANSGRESS_OK_TO_MQTT
#endif

#endif
