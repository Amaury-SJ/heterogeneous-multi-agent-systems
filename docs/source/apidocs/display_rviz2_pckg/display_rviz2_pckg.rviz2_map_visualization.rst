:py:mod:`display_rviz2_pckg.rviz2_map_visualization`
====================================================

.. py:module:: display_rviz2_pckg.rviz2_map_visualization

.. autodoc2-docstring:: display_rviz2_pckg.rviz2_map_visualization
   :allowtitles:

Module Contents
---------------

Classes
~~~~~~~

.. list-table::
   :class: autosummary longtable
   :align: left

   * - :py:obj:`Rviz2MapVisualization <display_rviz2_pckg.rviz2_map_visualization.Rviz2MapVisualization>`
     - .. autodoc2-docstring:: display_rviz2_pckg.rviz2_map_visualization.Rviz2MapVisualization
          :summary:

Functions
~~~~~~~~~

.. list-table::
   :class: autosummary longtable
   :align: left

   * - :py:obj:`main <display_rviz2_pckg.rviz2_map_visualization.main>`
     - .. autodoc2-docstring:: display_rviz2_pckg.rviz2_map_visualization.main
          :summary:

Data
~~~~

.. list-table::
   :class: autosummary longtable
   :align: left

   * - :py:obj:`geod <display_rviz2_pckg.rviz2_map_visualization.geod>`
     - .. autodoc2-docstring:: display_rviz2_pckg.rviz2_map_visualization.geod
          :summary:

API
~~~

.. py:data:: geod
   :canonical: display_rviz2_pckg.rviz2_map_visualization.geod
   :value: None

   .. autodoc2-docstring:: display_rviz2_pckg.rviz2_map_visualization.geod

.. py:class:: Rviz2MapVisualization()
   :canonical: display_rviz2_pckg.rviz2_map_visualization.Rviz2MapVisualization

   Bases: :py:obj:`rclpy.node.Node`

   .. autodoc2-docstring:: display_rviz2_pckg.rviz2_map_visualization.Rviz2MapVisualization

   .. rubric:: Initialization

   .. autodoc2-docstring:: display_rviz2_pckg.rviz2_map_visualization.Rviz2MapVisualization.__init__

   .. py:method:: listener_gps_for_publish_map(msg_gps_rcv)
      :canonical: display_rviz2_pckg.rviz2_map_visualization.Rviz2MapVisualization.listener_gps_for_publish_map

      .. autodoc2-docstring:: display_rviz2_pckg.rviz2_map_visualization.Rviz2MapVisualization.listener_gps_for_publish_map

   .. py:method:: select_coordinate_map(name)
      :canonical: display_rviz2_pckg.rviz2_map_visualization.Rviz2MapVisualization.select_coordinate_map

      .. autodoc2-docstring:: display_rviz2_pckg.rviz2_map_visualization.Rviz2MapVisualization.select_coordinate_map

.. py:function:: main(args=None)
   :canonical: display_rviz2_pckg.rviz2_map_visualization.main

   .. autodoc2-docstring:: display_rviz2_pckg.rviz2_map_visualization.main
