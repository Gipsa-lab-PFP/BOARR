# This script has two purposes : 
# - delete the world_creation_plugin from the world
# - insert the ros_interface_plugin that is needed in rotorS

for ((i=0; i<5000; i++))
    do
    	sed -i '/world_creation/d' ../worlds/forest$i.world > /dev/null 2>&1
    	sed -i '4i\
  <plugin name="ros_interface_plugin" filename="librotors_gazebo_ros_interface_plugin.so"/>
' ../worlds/forest$i.world > /dev/null 2>&1
    done

echo Script Completed

# see https://github.com/JenniferBuehler/gazebo-pkgs/tree/master/gazebo_world_plugin_loader
# to include the ros interface plugin without editing the world file
# anyway the edition of the world file is needed to delete the world creation plugin
