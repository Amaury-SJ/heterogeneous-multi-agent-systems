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
