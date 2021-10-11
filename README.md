# object-descriptions

This is a repository to collect and host URDF descriptions of generic objects. With a setup script, a functional ROS
package for a desired version of ROS can be created.

## Usage

To create the ROS package containing all the object descriptions for the version of your choice `<ros-version>`, run

```console
./setup.sh <ros-version>
```

Currently, supported versions are:

- melodic, noetic
- foxy, galactic

## Objects

Currently, this package contains three objects:

- Table
- Box
- Soccerball

## Visualization

To visualize the objects in RViz, use the provided launch scripts by either installing the package on your machine or
using Docker. To use Docker, run

```console
./build-run.sh <ros-version>
```

Note that this depends on [aica-docker](https://github.com/aica-technology/docker-images#scripts).

Once set up, launch with

- `roslaunch object_descriptions <object>.launch` in ROS, or
- `ros2 launch object_descriptions <object>.launch.py` in ROS2