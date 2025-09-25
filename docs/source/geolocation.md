# Geolocation distance calculation

To achieve collaboration and interaction between agents in the system, we need to be estimating distances in the environment between agents, but also with points of interest. Using GNSS positions (latitude and longitude), we show how to estimate distances. The library we're using, `GeographicLib`, is detailed in the second part.

## Calculations with formulas, or online

### Using various formulas

Simple formulas can be used to calculate distances between two geographical points known by their latitudes and longitudes, such as: Pythagoras, the law of sines, haversine. Examples are given [here](http://villemin.gerard.free.fr/aGeograp/Distance.htm#top).

Here's another method and detailed calculations from IGN, to obtain the distance between two known points in longitude and latitude on a sphere, available [here](https://geodesie.ign.fr/contenu/fichiers/Distance_longitude_latitude.pdf).

### On a website

Here's a site to calculate the distance between two geographic coordinates using the Vincenty formula, [here](http://mwlandry.ca/famille/etudes/calcul_distance.htm).

## GeographicLib library (recommended)

Available in C++, Python and other languages, the `GeographicLib` library solves two well-known geodesic problems. The Python library is [downloadable in Python](https://pypi.org/project/geographiclib/), and a [**documentation**](https://geographiclib.sourceforge.io/1.52/python/) is also available, containing some [examples](https://geographiclib.sourceforge.io/1.52/python/examples.html).

Here are the two functions for solving the two main problems:
- The `inverse` function, to calculate the distance between 2 points.
- The `direct` function, to find a point B from a point A, a direction and an angle.

For your information, this package has been taken over by a well-known Python library, [GeoPy](https://geopy.readthedocs.io/en/stable/index.html?highlight=geodesic#).