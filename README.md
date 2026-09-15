# Autonomous Vehicle Platooning Research

Software developed for autonomous vehicle and platooning research at the University of Washington. The project explores decentralized vehicle communication, GPS-based navigation, vehicle-to-vehicle data transmission, and road-network pathfinding.

The system integrates Python-based navigation software with embedded hardware using CAN and nRF24L01+ wireless communication. Vehicle position and velocity data are collected from a rover, transferred to an embedded controller over CAN, and broadcast between platoon nodes using time-slotted wireless communication.

## System Overview

The project contains two primary components:

### 1. Vehicle-to-Vehicle Platooning Communication

The platooning communication pipeline transfers vehicle state through several hardware and software layers:

```text
Rover / Pixhawk
      |
      | GPS + ground speed
      | DroneKit / MAVLink
      v
Jetson Nano
      |
      | CAN (500 kbps)
      v
Arduino
      |
      | nRF24L01+
      v
Other Platoon Nodes
```

`gps.py` connects to the rover through DroneKit and retrieves its GPS position and ground speed. The data is serialized into two 32-bit floats and transmitted over CAN using arbitration ID `0x100`.

The Arduino receives the CAN message and packages the vehicle state into a platoon packet containing:

```text
sender ID
position
velocity
timestamp
```

Packets are then transmitted wirelessly using an nRF24L01+ radio.

### 2. GPS Pathfinding and Navigation

The pathfinding component converts geographic road data stored in GeoJSON into a weighted graph.

Each road segment becomes an edge weighted by its Haversine distance. NetworkX then computes the shortest path between the graph nodes nearest to the requested start and destination coordinates.

The resulting path is converted into GPS waypoints and sent sequentially to the vehicle using DroneKit.

## Platooning Communication

### GPS and CAN Interface

`av-platooning/gps.py`

The Python navigation interface:

- Connects to a rover using DroneKit
- Reads GPS coordinates and ground speed
- Calculates distance to a target waypoint using the Haversine formula
- Calculates the heading toward the target
- Sends latitude and velocity over CAN
- Continues monitoring the vehicle until it reaches the target waypoint

CAN messages use arbitration ID:

```text
0x100
```

with an 8-byte payload:

```text
4 bytes: latitude
4 bytes: velocity
```

Both values are encoded as little-endian floating-point values.

### Embedded Platoon Node

`av-platooning/JetsonNano_nRF.ino`

The embedded component connects the CAN bus to an nRF24L01+ radio.

Each wireless packet contains:

```cpp
struct PlatoonPacket {
    uint8_t senderID;
    float position;
    float velocity;
    unsigned long timestamp;
};
```

The current implementation configures communication around a three-node TDMA cycle:

```text
Slot duration:  10 ms
Cycle duration: 30 ms
Nodes:          3
```

Each node is assigned a transmission window to reduce simultaneous transmissions between platoon members.

The radio operates at:

```text
Data rate: 1 Mbps
Channel:   108
```

### Reliability and Latency Logging

The embedded program records communication results to an SD card in:

```text
tdma_log.csv
```

Each record contains:

```text
Trial, FromID, PayloadSize, Latency, Success
```

For received packets, latency is calculated using the packet timestamp and local microsecond timer.

The current program is configured for 100 transmission trials.

Failed wireless transmissions are also recorded so communication reliability can be evaluated after testing.

## Pathfinding

`pathfinding/geojson_uwb_pathfinder.py`

The pathfinding system loads road-network geometry from:

```text
pathfinding/uwb.geojson
```

and converts each `LineString` into a NetworkX graph.

For each road segment:

1. GPS coordinates are extracted from the GeoJSON geometry.
2. Geographic distance is calculated using the Haversine formula.
3. An edge is added to the graph with distance as its weight.
4. The graph nodes nearest the requested start and destination are selected.
5. NetworkX computes a weighted shortest path.
6. The path is converted into DroneKit GPS waypoints.

The vehicle is placed into `GUIDED` mode and follows the generated waypoints before entering `HOLD` mode when the path is complete.

## Repository Structure

```text
platooning-research/
│
├── av-platooning/
│   ├── gps.py
│   └── JetsonNano_nRF.ino
│
├── pathfinding/
│   ├── geojson_uwb_pathfinder.py
│   └── uwb.geojson
│
└── README.md
```

### `gps.py`

Handles rover communication, GPS monitoring, heading and distance calculations, and transmission of vehicle position/velocity over CAN.

### `JetsonNano_nRF.ino`

Receives vehicle state over CAN, transmits platoon packets through nRF24L01+, implements time-slotted communication, and records communication test results.

### `geojson_uwb_pathfinder.py`

Builds a road-network graph from GeoJSON data, computes shortest paths, and sends generated GPS waypoints to a connected vehicle.

### `uwb.geojson`

Geographic road-network data used by the pathfinding implementation.

## Technologies

### Languages

- C++
- Python

### Autonomous Vehicle Software

- ROS
- DroneKit
- MAVLink

### Communication

- CAN
- nRF24L01+
- SPI

### Hardware

- NVIDIA Jetson Nano
- Pixhawk
- Arduino

### Navigation and Data Processing

- NetworkX
- GeoPandas
- GeoJSON

## Python Dependencies

The Python source uses the following packages:

```text
dronekit
python-can
networkx
geopandas
pymavlink
```

They can be installed with:

```bash
pip install dronekit python-can networkx geopandas pymavlink
```

## Running the CAN / GPS Interface

The system expects a SocketCAN interface named `can0`.

Run the GPS interface with a DroneKit connection string:

```bash
python gps.py --connect <connection-string>
```

For example, the connection string should point to the MAVLink endpoint used by the rover.

The script initializes:

```python
can.interface.Bus(channel='can0', bustype='socketcan')
```

and transmits current position and ground speed once valid GPS data is available.

## Running the Pathfinding Prototype

From the `pathfinding` directory:

```bash
python geojson_uwb_pathfinder.py
```

The current prototype connects to:

```text
127.0.0.1:14550
```

and uses example start and destination coordinates defined directly in the script.

After generating the route, the vehicle is armed in `GUIDED` mode and receives each computed waypoint sequentially.

## Research Goals

This code was developed as part of research into software and communication systems for autonomous vehicle platooning, with emphasis on:

- Decentralized vehicle-to-vehicle communication
- Hardware/software integration
- Low-latency transmission of vehicle state
- Communication reliability measurement
- GPS-based autonomous navigation
- Road-network pathfinding
- Cross-component debugging and testing

## Notes

This repository contains the primary code used during the research project rather than a packaged production system. Hardware configuration, CAN interfaces, radio wiring, vehicle connection strings, and geographic test coordinates may need to be changed for a different test environment.
