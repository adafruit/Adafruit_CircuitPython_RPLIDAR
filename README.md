## Introduction ##

Provide an interface to the SlamTec RPLidar A1.

Eventual goal is for the single module to work with both Linux (via /dev/USB0, etc) and CircuitPython (via a UART instance)

## Usage Example ##

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


## Contributing ##

Contributions are welcome! Please read our [Code of Conduct](https://github.com/adafruit/Adafruit_circuitpython_CircuitPython_RPLIDAR/blob/master/CODE_OF_CONDUCT.md)
before contributing to help this project stay welcoming.
