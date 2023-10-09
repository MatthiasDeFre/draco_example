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
#include <draco/compression/encode.h>
#include <draco/point_cloud/point_cloud_builder.h>
#include <draco/core/vector_d.h>
#include <draco/core/draco_types.h>
#include <draco/io/ply_decoder.h>
#include <draco/io/point_cloud_io.h>


class EncoderDraco 
{
public:
    int EncodePcl(std::string file_name, uint8_t q_pos, uint8_t enc_speed, uint8_t dec_speed);
    int EncodePly(std::string file_name,  uint8_t q_pos, uint8_t enc_speed, uint8_t dec_speed);
    draco::EncoderBuffer& GetBuffer();
    uint32_t GetEncodedSize();
private:
    draco::EncoderBuffer buffer;
    uint32_t encodedSize;

    int encode(draco::PointCloud& pc, uint8_t q_pos, uint8_t enc_speed, uint8_t dec_speed);
};

