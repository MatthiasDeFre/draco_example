#include "encoder_draco.h"

int EncoderDraco::EncodePcl(std::string file_name, uint8_t q_pos, uint8_t enc_speed, uint8_t dec_speed) {
    std::chrono::time_point<std::chrono::high_resolution_clock> start_clock_pcl = std::chrono::high_resolution_clock::now();

    // Load PC with PCL
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    int pcl_status = pcl::io::loadPLYFile(file_name, *input_cloud);
    if(pcl_status == -1) {
        std::cout << "PCL load .ply error " << std::endl;
        return -1;
    }   
   
    // Use draco builder to construct a draco compatible PC
    draco::PointCloudBuilder builder;
    builder.Start(input_cloud->size());
    // 3 bytes for position and 1 byte for color
    const int32_t att_id_pos = builder.AddAttribute(draco::GeometryAttribute::POSITION, 3, draco::DT_FLOAT32);
    const int32_t att_id_col = builder.AddAttribute(draco::GeometryAttribute::COLOR, 3, draco::DT_UINT8);

    // Set attributes using point cloud values
    for (int i = 0; i < input_cloud->size(); ++i) {
        std::array<float, 3> pos {input_cloud->points[i].x, input_cloud->points[i].y, input_cloud->points[i].z};
        std::array<uint8_t, 3> col {input_cloud->points[i].r, input_cloud->points[i].g, input_cloud->points[i].b};
        builder.SetAttributeValueForPoint(att_id_pos, draco::PointIndex(i), &pos);
        builder.SetAttributeValueForPoint(att_id_col, draco::PointIndex(i), &col);

    }
    std::unique_ptr<draco::PointCloud> pc = builder.Finalize(false);
    std::chrono::time_point<std::chrono::high_resolution_clock> end_clock_pcl = std::chrono::high_resolution_clock::now();
    int duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_clock_pcl - start_clock_pcl).count();
    std::cout << "Time to load/construct point cloud with PCL: " << duration << "ms" << std::endl;
    return encode(*pc, q_pos, enc_speed, dec_speed);
}

int EncoderDraco::EncodePly(std::string file_name,  uint8_t q_pos, uint8_t enc_speed, uint8_t dec_speed) {
    std::chrono::time_point<std::chrono::high_resolution_clock> start_clock_ply = std::chrono::high_resolution_clock::now();
    draco::PointCloud pc;
    draco::PlyDecoder ply_dec;
    
    auto ply_status = ply_dec.DecodeFromFile(file_name, &pc);
    if (!ply_status.ok()) {
        std::cout << "Decoding from file failed: " << file_name << ply_status.error_msg_string() << std::endl;
        return -1;
    }
    std::chrono::time_point<std::chrono::high_resolution_clock> end_clock_ply = std::chrono::high_resolution_clock::now();
    int duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_clock_ply - start_clock_ply).count();
    std::cout << "Time to load/construct point cloud with draco .ply loader: " << duration << "ms" << std::endl;
    return encode(pc, q_pos, enc_speed, dec_speed);
}

int EncoderDraco::encode(draco::PointCloud& pc, uint8_t q_pos, uint8_t enc_speed, uint8_t dec_speed) {
    std::chrono::time_point<std::chrono::high_resolution_clock> start_clock_enc = std::chrono::high_resolution_clock::now();
    buffer.Clear();
    draco::Encoder encoder;
    // Best encoding method of the two available
    encoder.SetEncodingMethod(draco::POINT_CLOUD_KD_TREE_ENCODING);
    encoder.SetSpeedOptions(enc_speed,dec_speed);
    // Can also set COLOR quantization but not sure if that actually does anything
    encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, q_pos);;
    auto enc_status = encoder.EncodePointCloudToBuffer(pc, &buffer);
    if (!enc_status.ok()) {
        std::cout << "Encoding Failed: " << enc_status.error_msg_string() << std::endl;
        return -1;
    }
    encodedSize = buffer.size();
    std::chrono::time_point<std::chrono::high_resolution_clock> end_clock_enc = std::chrono::high_resolution_clock::now();
    int duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_clock_enc - start_clock_enc).count();
    std::cout << "Time to encode point cloud with draco: " << duration << "ms" << std::endl;
    // Assuming 3x3 bytes for position and 3x1 byte for color
    std::cout << "Raw PC size: " << pc.num_points() * 15 << " | Encoded PC size: " << buffer.size() << std::endl;
    return 1;
}
draco::EncoderBuffer& EncoderDraco::GetBuffer()
{
    return buffer;
}


uint32_t EncoderDraco::GetEncodedSize() {
    return encodedSize;
}