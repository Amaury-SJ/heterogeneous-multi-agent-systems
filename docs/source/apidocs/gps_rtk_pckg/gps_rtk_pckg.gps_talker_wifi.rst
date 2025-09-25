:py:mod:`gps_rtk_pckg.gps_talker_wifi`
======================================

.. py:module:: gps_rtk_pckg.gps_talker_wifi

.. autodoc2-docstring:: gps_rtk_pckg.gps_talker_wifi
   :allowtitles:

Module Contents
---------------

Classes
~~~~~~~

.. list-table::
   :class: autosummary longtable
   :align: left

   * - :py:obj:`GpsTalker <gps_rtk_pckg.gps_talker_wifi.GpsTalker>`
     - .. autodoc2-docstring:: gps_rtk_pckg.gps_talker_wifi.GpsTalker
          :summary:

Functions
~~~~~~~~~

.. list-table::
   :class: autosummary longtable
   :align: left

   * - :py:obj:`main <gps_rtk_pckg.gps_talker_wifi.main>`
     - .. autodoc2-docstring:: gps_rtk_pckg.gps_talker_wifi.main
          :summary:

API
~~~

.. py:class:: GpsTalker()
   :canonical: gps_rtk_pckg.gps_talker_wifi.GpsTalker

   Bases: :py:obj:`rclpy.node.Node`

   .. autodoc2-docstring:: gps_rtk_pckg.gps_talker_wifi.GpsTalker

   .. rubric:: Initialization

   .. autodoc2-docstring:: gps_rtk_pckg.gps_talker_wifi.GpsTalker.__init__

   .. py:method:: timer_callback()
      :canonical: gps_rtk_pckg.gps_talker_wifi.GpsTalker.timer_callback

      .. autodoc2-docstring:: gps_rtk_pckg.gps_talker_wifi.GpsTalker.timer_callback

   .. py:method:: connect_wifi_nmea()
      :canonical: gps_rtk_pckg.gps_talker_wifi.GpsTalker.connect_wifi_nmea

      .. autodoc2-docstring:: gps_rtk_pckg.gps_talker_wifi.GpsTalker.connect_wifi_nmea

   .. py:method:: connect_wifi_enu()
      :canonical: gps_rtk_pckg.gps_talker_wifi.GpsTalker.connect_wifi_enu

      .. autodoc2-docstring:: gps_rtk_pckg.gps_talker_wifi.GpsTalker.connect_wifi_enu

   .. py:method:: extraction_donnees_gps_nmea(raw_d)
      :canonical: gps_rtk_pckg.gps_talker_wifi.GpsTalker.extraction_donnees_gps_nmea

      .. autodoc2-docstring:: gps_rtk_pckg.gps_talker_wifi.GpsTalker.extraction_donnees_gps_nmea

   .. py:method:: extraction_donnees_gps_enu(raw_d)
      :canonical: gps_rtk_pckg.gps_talker_wifi.GpsTalker.extraction_donnees_gps_enu

      .. autodoc2-docstring:: gps_rtk_pckg.gps_talker_wifi.GpsTalker.extraction_donnees_gps_enu

.. py:function:: main(args=None)
   :canonical: gps_rtk_pckg.gps_talker_wifi.main

   .. autodoc2-docstring:: gps_rtk_pckg.gps_talker_wifi.main
