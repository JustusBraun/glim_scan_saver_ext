# Introduction

This project contains an extension module for the [GLIM](https://github.com/koide3/glim) SLAM system, which exports the lidar scans received by GLIM to the dump directory for later use.
The lidar scans are extracted when they are inserted into the submap using the `SubMappingCallbacks::on_insert_frame` callback.
Each scan is deskewed using the final odometry estimation result before writing to a `ply` file.

# Compilation

This plugin can be build directly using CMake or by cloning the repository into a colcon workspace.

## Colcon

```bash
# Create a new workspace (skip this if you are using an existing workspace)
mkdir -p glim_ws/src
cd glim_ws/src

# Clone this repository
git clone https://github.com/JustusBraun/glim_scan_saver_ext.git

# Return to the workspace directory and build the workspace
cd ..
colcon build

```

## CMake

```bash
# Clone this repository
git clone https://github.com/JustusBraun/glim_scan_saver_ext.git

# Build the plugin
mkdir build
cd build
cmake ..
make

# Ensure that the build library `libglim_scan_saver_ext.so` is in your LD_LIBRARY_PATH!
# This can be done by installing the library
sudo make install
sudo ldconfig

# Or by prepending the build directory to the path
export LD_LIBRARY_PATH=$PWD:$LD_LIBRARY_PATH

```

# Usage

To activate the plugin, add the library name to the `extension_modules` list in GLIMs [config_ros.json](https://github.com/koide3/glim/blob/638ab8e26e8834d976b5efe6815e450b717bd916/config/config_ros.json) configuration file.

```json
"glim_ros": {
    ...
    // Extension modules
    "extension_modules": [
      "libmemory_monitor.so",
      "libstandard_viewer.so",
      "librviz_viewer.so",
      "libglim_scan_saver_ext.so" // THIS IS THE IMPORTANT LINE
    ],
    ...
}
```

