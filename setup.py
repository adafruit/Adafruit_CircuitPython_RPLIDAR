import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="adafruit-rplidar",
    version="0.0.1",
    author="Dave Astels",
    author_email="dastels@daveastels.com",
    description="Slamtec RPLIDAR A1 interface",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/adafruit/Adafruit_CircuitPython_RPLIDAR",
    py_modules=['adafruit_rplidar'],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
)
