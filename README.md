## Overview

This repository contains a modified version of the [Meshtastic project's](https://meshtastic.org/) device [firmware](https://github.com/meshtastic/firmware), to allow for:

- Removing certain constraints that may get in the way of development and analytic purposes.
- NeighborInfo broadcasts going out on the default channel, and the option to enable them by default.
- Having reduced position precision for specified private locations.


## Usage

Modify the **userPrefs.jsonc** file to uncomment and set the options that you want enabled.


## Option description

### Analytics options

* USERPREFS_ALLOW_NODENUM_ASSIGNMENT: Set to true if you have some app that needs to be able to set node IDs on sent packets. E.g. [meshpipe](https://github.com/armooo/meshpipe).
* USERPREFS_DISABLE_TRACEROUTE_THROTTLE: Removes the restriction on sending traceroutes too frequently.
* USERPREFS_DISABLE_POSITION_THROTTLE": Removes the restriction on sending position data too frequently.
* USERPREFS_TRANSGRESS_OK_TO_MQTT: Disregards the OK_TO_MQTT bitfield flag (unless you're using the default MQTT server, in which case it will still respect it).
* USERPREFS_UPLINK_ALL_CHANNELS: Normally only decrypted packets that match a channel are uplinked to MQTT. This option will uplink all decrypted channels, even if unknown. Though if it's a known channel and you don't have uplink_enabled then it shouldn't uplink it (so that you can still have private, non-uplinked channels).
* USERPREFS_UPLINK_ALL_PACKETS: Undecrypted packets to broadcast are not normally uplinked, nor are messages "from broadcast". This option allows that.
* USERPREFS_UPLINK_REPEAT_PACKET: Nodes ordinarily only uplink the first transmission of a packet that they see. With this option, each rebroadcast is uplinked (without getting rebroadcast or re-handled by our node again). Useful for identifying repeated broadcasts and the RSSI/SNR of them.

### NeighborInfo options

* USERPREFS_ENABLE_NEIGHBOR_INFO_BY_DEFAULT: As the name suggests, devices installed with this firmware will have neighbor info enabled by default, but also it will be enabled over lora by default.
* USERPREFS_ALLOW_NEIGHBOR_INFO_ON_DEFAULT_CHANNEL: Normally NeighborInfo is disabled on the default channel, this option alters that.

### Private position masks

Up to 8 hard-coded locations can be set to have a lower level of precision than what you have set generally for position data precision.

With a basic configuration of
```
"USERPREFS_APPLY_POSITION_MASKS": "true",
"USERPREFS_POSITION_MASK_0_LAT": "-12.3456789",
"USERPREFS_POSITION_MASK_0_LON": "123.4567890",
"USERPREFS_POSITION_MASK_0_BITS": "13",
```

if you go within 13-bits of precision of -12.3456789, 123.4567890 then your reported precision data will be reduced to 13-bits.

This allows you to reduce the precision when you are near private locations (e.g. home, work, houses of friends and family).

By default, these masks apply for *all* channels. The additional USERPREFS_MASK_POS_EVEN_WHEN_PRECISE setting can be set to false if you only want to blur the precision when your channel is not set to precise positioning. This way, you can have precise positioning on a private channel with no masks and at the same time have another imprecise channel where these additional masks will be applied.


**Warning**: These masks are only applied to position packets. They are not applied to MQTT map reports.

**Note**: Since the default MQTT server drops high-precision position data, using these masks may have an unintended effect: positions away from your specified locations may not be seen on maps that use data from the default server, but positions within the masked areas might.






