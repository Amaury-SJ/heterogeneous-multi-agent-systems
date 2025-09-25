#### Obtain with NMEA and ENU data.
# == WARNING OLD FORMAT ==

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

# Standard deviation (écart type) [meter]. (le rayon pas le diamètre)
float64 sdn
float64 sde
float64 sdu

# Solution status : 1 Fix, 2 Float, 5 Single
uint8 status

# Course over ground [degrees]. Route sur le fond (cap vrai car pas de dérive)
# [0,360] Reçu en sens trigonométrique inverse, mais transformé en sens trigo
float64 course

# Speed [Knots] Vitesse sur le fond en nœuds (2,69 nd = 3,10 mph = 4,98 km/h)
float64 speed

# Local tangent plane coordinates : East, North, Up (ENU) [m]
float64 enu_east
float64 enu_north
float64 enu_up
