# Gazebo Plugin ReadMe

## World Creation Plugin ReadMe

### Quick exec : 
Both the plugin and the necessary world file are proposed.
To create a new set of forests you will need to call the gazebo plugin "world_creation_plugin"
But in order to do so you will need to compile and link it to gazebo : 
- mkdir build
- cd build
- cmake ..
- make
- cd ..
- GAZEBO_PLUGIN_PATH = ${GAZEBO_PLUGIN_PATH}:$(pwd)/build #TODO check that this is working but anyway you need to source the build directory

and finally : 
- gzserver generate_forests.worlds

It will generate worlds in ../worlds using the models in ../models and therefore needs both those dierctories. 

If you feel this plugin been useful and want to give credit, please cite the ROAST benchmark paper.

### What it does in detail
This plugin generates forests by spawning trees in a heightmap. 
It creates forests by swpaning trees following multiple gaussian distributions resulting in multiple tree density in the same forest. 
It require a number of heightmap files and a number of tree models. 

The configuration of the plugin is to be made directly on the world_creation.cc file.
This is a list of what can be changed followed by their default value and the exact name of the parameter.
- heightmap size (82m*82m, heightMapSize)
- number of forests to generate (10, forestMaxNumber)
- number of tree per generated forest (100,tree_number)
- number of heightmap model (5, heightMapModelNumber)
- number of tree model (20, treeModelNumber)
- number of gaussian distributions in which to split the total number of tree to generate (3, foretGaussianceCenterNumber) 
- standard deviation for each gaussian distribution (12m,30m,60m, std::vect forestGaussianceSTDDEV)
- area where tree can spawn (everywhere but 5meter on the side of the map, using safePoints, safeMargin, x_min, x_max, y_min and y_max )

### Heigthmaps
They are DEM files. DEM files from the United States can be downloaded at "The National Map" from the US Geological Survey at :
https://viewer.nationalmap.gov/basic/

To creates worlds that are 100m by 100m, I advise to select the most precise availables DEM files which are 1meter DEM files and to download them as "IMG".
Those files are still 100km² which is way to big for light gazebo worlds and they will need to be modified to be in a format that gazebo can read.

Using the software QGIS (which is a GUI for the GTAL tool), the following actions needs to be taken :
Add raster layer
Raster->extraction->clipper->
Extraction in Gtiff ( erase the -of of the gtal command line)

Due to Gazebo limitations, at least in 7.0.0, you MUST extract a SQUARE. This square can be bigger than your target in gazebo but can NOT BE SMALLER.
Another gazebo limitation will force you to have your heightmap centered around (0,0)
	
	
## Wind Plugin

Not Much to say here.

The RotorS wind plugin is pretty solid but for this benchmark it lacked some randomness and some duration in its effect. 
I simply propose a version that will be composed of multiple random wind gust instead of a simple one. 

