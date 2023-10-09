#pragma once
#include <vector>
#include <string.h>
#include <fstream>
#include <chrono>
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

// Draco
#include <draco/io/ply_encoder.h>
#include <draco/core/vector_d.h>
#include <draco/core/draco_types.h>
#include <draco/io/point_cloud_io.h>


class DecoderDraco 
{
public:
    int Decode(const char*  buffer, uint32_t encoded_size);

    void SaveWithPCL(std::string file_name);
    void SaveWithDraco(std::string file_name);
    private:
    std::unique_ptr<draco::PointCloud> decoded_pc;
};

