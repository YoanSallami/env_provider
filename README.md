# Env Provider

![EnvProviderExample](/img/env_provider.png)

## Installation

### Dependencies

To install Underworlds please refer to [installation documentation](http://underworlds.readthedocs.io/en/latest/installation.html?highlight=installation)

To install ROS please refer to [installation documentation](https://wiki.ros.org/kinetic/Installation)

### Package installation

```
cd catkin_ws/src
git clone https://github.com/YoanSallami/env_provider.git
cd ..
catkin_make
```

## Usage

### Argparse parameters

- The Underworlds world to create
- The path used to localize the static geometric description file
- The path used to localize the static facts description file
- The path to mesh directory

### Make your own geometric description
To build your own environment you need to define an YAML file with the following informations for each object :

1. The name of the object
2. The path of the 3D file relative to the mesh directory
3. The scale of the mesh
4. The position of the object (m)
5. The orientation of the object (euler angles in rad)

See example file [here](https://github.com/underworlds-robot/env_provider/blob/master/param/static_geometric_description.yaml)

### Make your own static facts description

You can add static facts by defining a YAML file with the following information for each one :

1. The string description of the fact

See example file [here](https://github.com/underworlds-robot/env_provider/blob/master/param/static_facts.yaml)

### Launch

First, launch the Underworlds server deamon with :

`uwds start`

Then :

`roslaunch env_provider env_provider.launch`
