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
#endif

