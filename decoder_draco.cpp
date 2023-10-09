#include "decoder_draco.h"

int DecoderDraco::Decode(const char* buffer, uint32_t encoded_size) {
    std::chrono::time_point<std::chrono::high_resolution_clock> start_clock_dec = std::chrono::high_resolution_clock::now();
    draco::Decoder decoder;
    draco::DecoderBuffer dec_buffer;
    dec_buffer.Init(buffer, encoded_size);
    auto dec_status = decoder.DecodePointCloudFromBuffer(&dec_buffer);

    if(!dec_status.ok()) {
        std::cout << "Decoding failed" << std::endl;
        return 0;
    }

    // Save PC back to a file, either with PCL or draco
    decoded_pc = std::move(dec_status).value();

    std::chrono::time_point<std::chrono::high_resolution_clock> end_clock_dec = std::chrono::high_resolution_clock::now();
    int duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_clock_dec - start_clock_dec).count();
    std::cout << "Time to decode point cloud with draco: " << duration << "ms" << std::endl;
    return 1;
}


void DecoderDraco::SaveWithPCL(std::string file_name) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl_cloud->resize(decoded_pc->num_points());
    // Examples of accessing draco PC data
    // Based on https://github.com/google/draco/blob/master/src/draco/io/ply_encoder.cc#L64
    const int pos_att_id = decoded_pc->GetNamedAttributeId(draco::GeometryAttribute::POSITION);
    const int col_att_id = decoded_pc->GetNamedAttributeId(draco::GeometryAttribute::COLOR);
    
    // Reconstruct a PCL point cloud
    // Ideally it would be nice if there was a way to convert all points at once but haven't found such a method
    for (int v = 0; v < decoded_pc->num_points(); ++v) {
        const auto *const pos_att = decoded_pc->attribute(pos_att_id);
        const auto *const col_att = decoded_pc->attribute(col_att_id);
        std::vector<float> pos(3);
        std::vector<uint8_t> col(3);
        pos_att->ConvertValue<float, 3>(pos_att->mapped_index(draco::PointIndex(v)), pos.data());
        col_att->ConvertValue<uint8_t, 3>(col_att->mapped_index(draco::PointIndex(v)), col.data());
        
        pcl_cloud->points[v].x = pos[0];
        pcl_cloud->points[v].y = pos[1];
        pcl_cloud->points[v].z = pos[2];
    
        pcl_cloud->points[v].r = col[0];
        pcl_cloud->points[v].g = col[1]; 
        pcl_cloud->points[v].b = col[2]; 
    }
    pcl::io::savePLYFileBinary(file_name, *pcl_cloud);
}
void DecoderDraco::SaveWithDraco(std::string file_name) {
    draco::PlyEncoder ply_enc;
    ply_enc.EncodeToFile(*decoded_pc, file_name);
}
