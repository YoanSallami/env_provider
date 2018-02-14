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

### Make your own description
To build your own environment you need to define an YAML file with the following informations for each object :

1. The name of the object
2. The path of the 3D file relative to the mesh directory (see next section)
3. The scale of the mesh
4. The position of the object (m)
5. The orientation of the object (euler angles in rad)

See example file [here](https://github.com/YoanSallami/env_provider/blob/master/param/static_geometric_description.yaml)

### Launch

First, launch the Underworlds server deamon with :

`uwds start`

Then :

`roslaunch env_provider env_provider.launch`
