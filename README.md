### Tech
List of a projects for system to work properly:
* [OpenCV](https://opencv.org/) - is an open source computer vision and machine learning software library 
* [Demo People Counting](http://www.ti.com/tool/tidep-01000) - demo for counting people app - radar

And of course this app itself is open source with a [public repository](https://github.com/jurajkula/IBT)
 on GitHub.

### Installation

This system requires [OpenCV](https://opencv.org/) v3.4.3 to run. This library is downloaded from install script. Library for radar (demo people counting) has to be downloaded from [official web](https://training.ti.com/people-counting-demonstration-using-ti-mmwave-sensors)

Install the dependencies and devDependencies and start the server.

```sh
$ cd IBT
$ sh install.sh
$ cd libraries
$ mkdir build
$ cmake . ../CMakeList.txt
$ make
$ make install
$ make clear
```
### Configuration
config files:
* global.cfg - global config
* mmw_pplcount_demo_default_cfg - config for senzor radar

Config for senzor radar is explained at manufacturer [websites](http://www.ti.com/tool/tidep-01000)
Global config:
* FusionDelimiter - number of sectors for radar and camera data
* WinStride - step size of sliding window for detection method
* Scale - number of layouts
* ImageSice - size of image
* 

### Start
```sh
$ python3 main.py
```
**Params:**
* [-h/--help] - print help message
* [-d/--debug] - enable log mesages
* [-m/--mode] - specifies mode
* [-i/--id] - specifies ID of data folder (path: data/records/record-ID)

**Supported modes:**
* 'run' - start default program
* 'save' - start program and enable saving data
* 'load' - load data from inserted ID (argument -i/--id)


License
----

MIT
