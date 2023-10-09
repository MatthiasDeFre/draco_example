# Draco encoding / decoding example
This repository contains a small example application to encode/decode a supplied .ply file. There are two options to read to read the .ply file, either directly with draco or by using PCL.
The PCL example is meant to give an idea on how to convert existing formats (PCL point clouds or depth/color frames) into a draco compatible point cloud. 

## Dependencies
The application relies on two dependencies [Draco](https://github.com/google/draco) and [Point Cloud Library (PCL)](https://github.com/PointCloudLibrary/pcl). 
You can either install these manually or by using vcpkg in order to build / install them locally.

### Draco build / installation
```
git submodule update --init
mkdir build_dir && cd build_dir
cmake ../ -DDRACO_TRANSCODER_SUPPORTED=ON -DCMAKE_BUILD_TYPE=Release
```
On Windows this will generate a Visual Studio solution which you should then build (make sure to also select the Release candiate here)

On Linux you just need to run following commands:
```
cmake --build .
sudo cmake --install .
```

### PCL installation
You can choose to manually build PCL, however building takes a long time so just installing might be the better option.

On Windows you can download the .exe installer from the [release page](https://github.com/PointCloudLibrary/pcl/releases) on Github. 

On Linux you can just run following command:
```
sudo apt install libpcl-dev
```

## Application CMD options

| **Parameter** | **Name**    | **Description**                                                       | **Example**   |
|---------------|-------------|-----------------------------------------------------------------------|---------------|
| -i            | Input file  | The .ply used as an input to the encoder (required)                   | frame_0000.ply|
| -o            | Output file | If you want the decoded PC to be saved you should supply this         | output.ply    |
| -q            | QP          | Quantisation parameter of the position attribute                      | 11            |
| -e            | Encoder speed | Speed of the encoder, enables/disables certain compression optimisation: 0 (slow, better compression) to 10 (fast, worse compression) | 10            |
| -d            | Decoder speed | Speed of the decoder, same principle as -e. However, influence seems to be more limited | 10            |
| -p            | Use PCL     | Should PCL be used to load / save the point cloud. When not using this, draco functions are used to load / save the PC | n.a.            |

