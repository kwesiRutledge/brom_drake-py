.. brom_drake-py documentation master file, created by
   sphinx-quickstart on Tue Dec 16 22:27:52 2025.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

brom_drake-py
=============

Introduction
------------

Brom is a helper library for the `Drake`_ robotics simulation and verification library.
Its goal is to simplify common debugging and testing activities in Drake (for example, logging the outputs of systems in your block diagrams).

.. _Drake: https://drake.mit.edu

.. note::

   This documentation is a work in progress. 
   If you feel that anything is missing, then create an Issue on the
   `brom_drake-py GitHub repository <https://github.com/kwesirutledge/brom_drake-py>`_ for the latest updates.

Installation
------------

To install brom_drake-py, use pip:

.. code-block::

   pip install brom_drake-py

Extras
------

Some features need optional dependencies, installed as pip extras:

- ``video``: recording MP4 videos of simulations (see :doc:`features/recording_videos`).

  .. code-block::

     pip install "brom-drake[video]"

- ``ros2``: resolving ROS 2 ``package://`` URIs with ``ament_index_python``.

  .. code-block::

     pip install "brom-drake[ros2]"


.. toctree::
   :maxdepth: 2
   :caption: Contents:
   :glob:

   features/index
   generated/api

