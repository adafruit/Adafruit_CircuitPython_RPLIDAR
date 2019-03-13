Introduction
============

.. image:: https://readthedocs.org/projects/adafruit-circuitpython-circuitpython-rplidar/badge/?version=latest
    :target: https://circuitpython.readthedocs.io/projects/rplidar/en/latest/
    :alt: Documentation Status

.. image:: https://img.shields.io/discord/327254708534116352.svg
    :target: https://discord.gg/nBQh6qu
    :alt: Discord

.. image:: https://travis-ci.com/adafruit/Adafruit_circuitpython_CircuitPython_RPLIDAR.svg?branch=master
    :target: https://travis-ci.com/adafruit/Adafruit_circuitpython_CircuitPython_RPLIDAR
    :alt: Build Status

.. Provide a convienent interface to the Slamtec RPLidar.

Dependencies
=============
This driver depends on:

* `Adafruit CircuitPython <https://github.com/adafruit/circuitpython>`_
* `Bus Device <https://github.com/adafruit/Adafruit_CircuitPython_BusDevice>`_
* `Register <https://github.com/adafruit/Adafruit_CircuitPython_Register>`_

Please ensure all dependencies are available on the CircuitPython filesystem.
This is easily achieved by downloading
`the Adafruit library and driver bundle <https://github.com/adafruit/Adafruit_CircuitPython_Bundle>`_.

Usage Example
=============

.. code-block:: python

    import os
    from math import cos, sin, pi, floor
    import pygame
    from adafruit_circuitpython_rplidar import RPLidar

    # Set up pygame and the display
    os.putenv('SDL_FBDEV', '/dev/fb1')
    pygame.init()
    lcd = pygame.display.set_mode((320,240))
    pygame.mouse.set_visible(False)
    lcd.fill((0,0,0))
    pygame.display.update()

    # Setup the RPLidar
    PORT_NAME = '/dev/ttyUSB0'
    lidar = RPLidar(None, PORT_NAME)

    # used to scale data to fit on the screen
    max_distance = 0

    def process_data(data):
        # Do something useful with the data
        pass

    scan_data = [0]*360

    try:
        print(lidar.get_info())
        for scan in lidar.iter_scans():
            for (_, angle, distance) in scan:
                scan_data[min([359, floor(angle)])] = distance
            process_data(scan_data)

    except KeyboardInterrupt:
        print('Stoping.')
    lidar.stop()
    lidar.disconnect()


Contributing
============

Contributions are welcome! Please read our `Code of Conduct
<https://github.com/adafruit/Adafruit_circuitpython_CircuitPython_RPLIDAR/blob/master/CODE_OF_CONDUCT.md>`_
before contributing to help this project stay welcoming.

Building locally
================

Zip release files
-----------------

To build this library locally you'll need to install the
`circuitpython-build-tools <https://github.com/adafruit/circuitpython-build-tools>`_ package.

.. code-block:: shell

    python3 -m venv .env
    source .env/bin/activate
    pip install circuitpython-build-tools

Once installed, make sure you are in the virtual environment:

.. code-block:: shell

    source .env/bin/activate

Then run the build:

.. code-block:: shell

    circuitpython-build-bundles --filename_prefix adafruit_circuitpython-circuitpython-rplidar --library_location .

Sphinx documentation
-----------------------

Sphinx is used to build the documentation based on rST files and comments in the code. First,
install dependencies (feel free to reuse the virtual environment from above):

.. code-block:: shell

    python3 -m venv .env
    source .env/bin/activate
    pip install Sphinx sphinx-rtd-theme

Now, once you have the virtual environment activated:

.. code-block:: shell

    cd docs
    sphinx-build -E -W -b html . _build/html

This will output the documentation to ``docs/_build/html``. Open the index.html in your browser to
view them. It will also (due to -W) error out on any warning like Travis will. This is a good way to
locally verify it will pass.
