:py:mod:`gps_rtk_pckg.gps_talker_usb`
=====================================

.. py:module:: gps_rtk_pckg.gps_talker_usb

.. autodoc2-docstring:: gps_rtk_pckg.gps_talker_usb
   :allowtitles:

Module Contents
---------------

Classes
~~~~~~~

.. list-table::
   :class: autosummary longtable
   :align: left

   * - :py:obj:`GpsTalkerUsb <gps_rtk_pckg.gps_talker_usb.GpsTalkerUsb>`
     - .. autodoc2-docstring:: gps_rtk_pckg.gps_talker_usb.GpsTalkerUsb
          :summary:

Functions
~~~~~~~~~

.. list-table::
   :class: autosummary longtable
   :align: left

   * - :py:obj:`main <gps_rtk_pckg.gps_talker_usb.main>`
     - .. autodoc2-docstring:: gps_rtk_pckg.gps_talker_usb.main
          :summary:

API
~~~

.. py:class:: GpsTalkerUsb()
   :canonical: gps_rtk_pckg.gps_talker_usb.GpsTalkerUsb

   Bases: :py:obj:`rclpy.node.Node`

   .. autodoc2-docstring:: gps_rtk_pckg.gps_talker_usb.GpsTalkerUsb

   .. rubric:: Initialization

   .. autodoc2-docstring:: gps_rtk_pckg.gps_talker_usb.GpsTalkerUsb.__init__

   .. py:method:: base_position_srv_callback(request, response)
      :canonical: gps_rtk_pckg.gps_talker_usb.GpsTalkerUsb.base_position_srv_callback

      .. autodoc2-docstring:: gps_rtk_pckg.gps_talker_usb.GpsTalkerUsb.base_position_srv_callback

   .. py:method:: timer_callback()
      :canonical: gps_rtk_pckg.gps_talker_usb.GpsTalkerUsb.timer_callback

      .. autodoc2-docstring:: gps_rtk_pckg.gps_talker_usb.GpsTalkerUsb.timer_callback

   .. py:method:: connection()
      :canonical: gps_rtk_pckg.gps_talker_usb.GpsTalkerUsb.connection

      .. autodoc2-docstring:: gps_rtk_pckg.gps_talker_usb.GpsTalkerUsb.connection

   .. py:method:: extraction_donnees_gps_nmea(raw_d)
      :canonical: gps_rtk_pckg.gps_talker_usb.GpsTalkerUsb.extraction_donnees_gps_nmea

      .. autodoc2-docstring:: gps_rtk_pckg.gps_talker_usb.GpsTalkerUsb.extraction_donnees_gps_nmea

.. py:function:: main(args=None)
   :canonical: gps_rtk_pckg.gps_talker_usb.main

   .. autodoc2-docstring:: gps_rtk_pckg.gps_talker_usb.main
