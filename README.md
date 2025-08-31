# NTRIP NMEA Bridge

This package provides a bridge between NTRIP (Networked Transport of RTCM via Internet Protocol) and NMEA (National Marine Electronics Association) sentences. It allows for the parsing and publishing of NMEA sentences received over a TCP connection.

## Features

- Connect to an NTRIP server and receive RTCM corrections
- Parse NMEA sentences (GGA, RMC, etc.) from the NTRIP stream
- Publish parsed NMEA sentences as ROS 2 messages

## Prerequisites

Before using this package, ensure you have the following:

- ROS 2 installed (Humble Hawksbill or later)
- A working NTRIP server with valid credentials

## Usage

In my case, I run this command for starting NTRIP server:

```bash
str2str -in ntrip://*****:*****@nrtk.big.go.id:2001/Nearest-rtcm3 -out serial://ttyUSB0:230400#52001 -b 1
```

> Make sure to replace the `*****` with your actual username and password.

And run the following command to start the NTRIP bridge:

```bash
ros2 launch ntrip_nmea_bridge ntrip_nmea_bridge.launch.py
```

## Data List

### nmea_sentence

The NMEA sentence received from the NTRIP stream.
This sentence is published as a ROS 2 message. On the topic `/nmea_sentence`.

```json
{
  "header": {
    "stamp": {
      "sec": 0,
      "nanosec": 0
    },
    "frame_id": ""
  },
  "sentence": "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"
}
```

### fix

The fix status of the GPS receiver. This message is published as a ROS 2 message. On the topic `/fix`.

```json
{
  "header": {
    "stamp": {
      "sec": 0,
      "nanosec": 0
    },
    "frame_id": ""
  },
  "fix": 1
}
```

### ground_speed

The ground speed of the GPS receiver. This message is published as a ROS 2 message. On the topic `/ground_speed`.

```json
{
  "header": {
    "stamp": {
      "sec": 0,
      "nanosec": 0
    },
    "frame_id": ""
  },
  "ground_speed": 0.0
}
```
