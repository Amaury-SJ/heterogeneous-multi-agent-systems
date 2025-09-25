# RViz2 package: system observer via ROS 2

To visualize the various data from our system as an observer, we have created a `display_rviz2_pckg` package. It consists of a launch file, and two nodes to display and visualize in RViz2 the RTK GPS of each agent, as well as a map of the local area.

```{figure} /photos/rviz2_display/rosgraph_display_rviz2.png
:align: center
:alt: rosgraph_display_rviz2
Graphical representation of our RViz2 display system. Here we use the default value of the namespace and the agent with RTK GPS.
```

## Launch file

As a system observer, this launch file `src/display_rviz2_pckg/launch/display.launch.py` can be used to launch `rviz2`, `rviz2mapvisualization` and `rviz2gpsvisualization`. The default name of the namespace is `display_default`.

```{autodoc2-object} display.launch.generate_launch_description
render_plugin = "myst"
```

## GPS display

We recommend using this `rviz2_gps_visualization` node with a launch file, because to run directly, you need to pass an ROS 2 argument in the form of a list of character strings, which is not trivial.

```{figure} /photos/rviz2_display/rviz2_marker_gps.png
:align: center
:alt: rviz2_marker_gps
Our 3D representation of an RTK GPS in RViz2 in real time.
```

```{note}
All our 4 publishers are in the **same callback**, so execution time is higher, but remains reasonable for display (5 ms). So, there's no need to make multiple separate nodes in parallel.
```

```{figure} /photos/rviz2_display/rviz2_marker_gps_list.png
:align: center
:alt: rviz2_marker_gps_list
List of our objects displayed in RViz2 for RTK GPS human_1. Each object in the rviz node subscribes to a published topic.
```

We use the `rviz2_gps_visualization` node from the `display_rviz2_pckg` package:

```{autodoc2-object} display_rviz2_pckg.rviz2_gps_visualization.Rviz2GpsVisualization
render_plugin = "myst"
```

### Tip: Create several publishers from the same callback

In this node we use a tip, we're going to use the `partial` function in the `functools` library to pass a callback listener with a different topic name as a subscriber parameter: this will identify which agent the method corresponds to.

## Map display

To achieve this mapping, different maps exist linked to APIs such as Google Maps, BingMaps, plotly, however this requires an API key and/or authentication. We're going to use a map under a free license: `OpenStreetMap`. If the area of operation is known or even approximate, you can download the map directly from [OpenStreetMap](https://www.openstreetmap.org/export) and then work offline. If you're working in an unknown area, you can automate map retrieval.

```{note}
In addition of OpenStreetMap, there are geographic information systems (GIS) free to create, edit, visualize, analyze and publish geospatial data, as [QGIS](https://www.qgis.org/).
```

For the 3D object, a COLLADA file format is used, with extension `.dae`. The physical object is 1 metre by 1 metre in size, centred at zero, and of zero thickness. La texture déposée dessus est donc déformée. Il faut maintenant redimensionner l'objet grâce à la bibliothèque `geographiclib.geodesic` et sa focntion `geod.Inverse`.

```{figure} /photos/rviz2_display/blender_map_representation.png
:align: center
:alt: blender_map_representation
Representation of the 3D map object in Blender, with a side length of 1 meter and a thickness of zero.
```

The idea is to have a single object `display_rviz2_pckg/images/object_map.dae`, and only change its texture `map.png` according to the map selected. The old texture will be deleted, then the new image will be copied and renamed `map.png`. This saves us having to create a map object for each tile.

We use the `rviz2_map_visualization` node from the `display_rviz2_pckg` package:

```{autodoc2-object} display_rviz2_pckg.rviz2_map_visualization.Rviz2MapVisualization
render_plugin = "myst"
```

### Alternatives: map display without RViz2

Once the map and its coordinates have been retrieved, several libraries are available to handle map display. However, not all of them are capable of real-time display (Cartopy, Folium, GeoPandas), and they also require JavaScript display, which is not necessarily necessary (Leaflet, mapboxgl). We will use `Matplotlib` which works with a number of user interface toolkits (wxpython, tkinter, qt, gtk, and macosx). Here are a few tips on how to create a real-time display with Matplotlib.

In Matplotlib, the `draw` (and `draw_idle`) function is not required with `ion`: This is used to update a figure that has been altered, but not automatically re-drawn. If interactive mode is on (via `ion()`), this should be only rarely needed, but there may be ways to modify the state of a figure without marking it as `stale`. `Draw` is equivalent to calling `fig.canvas.draw_idle()`, where `fig` is the current figure.

## Blender for 3D objects

```{note}
To display the textures of an object in Blender, such as an image on a surface, click on shading, then on color, and select texture.
```

```{figure} /photos/rviz2_display/blender_texture_display.png
:align: center
:alt: blender_texture_display
Menu for displaying textures in Blender.
```

```{figure} /photos/rviz2_display/blender_axis.png
:align: center
:alt: blender_axis
3D representation of the axes in Blender, enabling correspondence with RViz2.
```