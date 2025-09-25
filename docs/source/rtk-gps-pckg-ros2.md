# RTK GPS package: receive and publish data via ROS 2

To collect data from RTK GNSS receivers, in particular Emlid Reach, here are two packages, `gps_msg_pckg` and `gps_rtk_pckg`. Here are the files that will interest us: 

```yaml
gps_msg_pckg
└── msg
    └── SatMsgRcv.msg
gps_rtk_pckg
└── gps_rtk_pckg
    ├── gps_talker_usb.py
    └── gps_talker_wifi.py
```

## Custom message: gpsSatMsgRcv

In the `gps_msg_pckg` package, we create a new custom message: the `gps_msg_pckg/msg/gpsSatMsgRcv.msg` format:

```
#### Obtain with NMEA and or ENU data. 

std_msgs/Header header

# Satelite time [yyyy/mm/dd HH:MM:SS.SSS]
string timer

# Latitude [degrees] [-90,90]. Positive is north of equator; negative is south.
float64 latitude

# Longitude [degrees] [-180,180]. Positive is east of prime meridian; negative is west.
float64 longitude

# Altitude [m]. Positive is above the WGS 84 ellipsoid
# (quiet NaN if no altitude is available).
float64 altitude

# Standard deviation [meter] (radius not diameter)
# Fr: écart-type
float64 sdn
float64 sde
float64 sdu

# Solution status: 1 Single (no RTK), 4 (Fix), 5 (Float)
uint8 status

# Course over ground [degrees] [0,360]
# Fr: Route sur le fond (cap vrai car pas de dérive)
# Received in reverse trigonometric sense, but transformed into trigonometric sense.
float64 course

# Speed [Knots] (2,69 nd = 3,10 mph = 4,98 km/h)
# Fr: Vitesse sur le fond en nœuds
float64 speed

# Local tangent plane coordinates : East, North, Up (ENU) [m]
float64 enu_east
float64 enu_north
float64 enu_up

# Base coordinates in NMEA via EBP
float64 base_latitude
float64 base_longitude
float64 base_altitude
```

## Using USB ports

For the ROS 2 node in the next section, we need access to the USB ports. Here are some explanations and problems encountered.

We can use the library in Python [**PySerial**](https://pyserial.readthedocs.io/en/latest/shortintro.html) easy to [install](https://pypi.org/project/pyserial/). We can test the usb connection with a **simple example** file and the `pySerial` Python library:

```python
# Simple example

import serial
port = None
try:
    port = serial.Serial("/dev/ttyS4", baudrate=9600, timeout=0.1)
    # To find the USB port name via terminal: python -m serial.tools.list_ports
except Exception as error:
    print('Error creation:',error)
while True:
    try:
        rcv = port.read(10000)
        print('==RECU== : ',rcv)
    except Exception as error:
        print('Error reception:',error)
```

Whether for the Emlid GPS output parameters to be set via the firmware or the `pySerial` library, the baudrate must be chosen. The baudrate is the transmission speed in serial links and corresponds to the number of bits transmitted per second. The most common baud rates are 9600 and 115200. We choose the higher **115200 bauds**.
 
To display the **list of available ports**, and more precisely the names of USB-connected devices, simply enter in the terminal:
```shell
python3 -m serial.tools.list_ports
```
For example, each GPS has a name: the first GPS connected will be called _/dev/ttyACM0_, then the second _/dev/ttyACM1_, etc.

For the `pySerial` library, the `read` function takes the number of bytes to be read as a parameter. The function is blocked if a non-zero value is given for the `timeout` argument of the `serial.serial` class. We set a large value (10000) so that the message is read all at once. Also, in the event of traffic congestion (the USB port hasn't been read for a while), to clear the channel more quickly.

```{error}
**Permission denied errors**

When trying to access the USB port, we get an access error, so we need to execute `sudo` before the Python code: `sudo python3 port_usb_lecture.py`. However, for ROS 2 code, we can't do this.

The solution is as follows: On Debian based systems, serial ports are usually in the group `dialout`, so run `sudo adduser $USER dialout` (and then logging-out and -in) enables the user to access the port. (For your information, the `$USER` environment variable can be displayed with: `printenv USER`)
```

## GPS talker via USB

Here's the `gps_rtk_pckg/gps_rtk_pckg/gps_talker_usb` node to use to retrieve RTK GPS data via USB, and publish it in a topic `topic_gps`.

### How the node works

After reading the port, we'll process and extract the GPS data received. We'll separate the data, since we know the format of NMEA frames. We'll perform 4 tests conditions:
- If the message is empty, i.e. no data is received via USB
- A first test is carried out to determine whether the message received is not offset, starting with `b'$GNRMC`.
- Next, the second item in the list must not be empty, otherwise it means that no GPS signal is being received from the satellites.
- Finally, the length of the complete data is 266 characters maximum, with the selected frames (see section on NMEA format, in thesis). So the size received must not exceed 270 (to be a little wider). This avoids having to take old frames into account when receiving several frames at once, so we choose to ignore them.

If the GPS is connected to a base station, the coordinates of the base station can be retrieved, otherwise a warning appears.

```{important}
The frequency used to read the USB port must be lower than the frequency used to receive messages from the GNSS receiver. For the Reach M2, the frequency is 10 Hz, so there's a message every 100 ms. We'll therefore read the port every 10 ms (10%) in our code. In detail: the class `serial.Serial` has the attribute: `timeout`, which we set to `1/(freq*10)`. This directly influences the function `read(size=10000)`. We read 10000 bytes because the reception size can vary depending on the rounding, for example.
```

### Node description 

The `gps_rtk_pckg/gps_rtk_pckg/gps_talker_usb` node is used to retrieve RTK GPS data via USB, and publish it in a topic `topic_gps`.

```{autodoc2-object} gps_rtk_pckg.gps_talker_usb.GpsTalkerUsb
render_plugin = "myst"
```

Improvement possible: for locations other than data[4]=North and data[6]=East, add their signs in the node.

## GPS talker via Wi-Fi (not recommended)

```{warning}
We recommend receiving data via USB, with the previous node `gps\rtk_pckg/gps\rtk_pckg/gps_talker_usb`.
```

This `gps_rtk_pckg/gps_rtk_pckg/gps_talker_wifi` node is not recommended, as the sensor must be mounted on an on-board system, and only requires a USB connection between the GNSS receiver and the on-board card. In addition, the Wi-Fi connection is often lost, resulting in real-time data loss.

```{autodoc2-object} gps_rtk_pckg.gps_talker_wifi.GpsTalker
render_plugin = "myst"
```