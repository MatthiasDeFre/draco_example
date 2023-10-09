#include "camera.h"
#include <string>
#include <pcl/point_types.h>
#include <vector>
#include <pcl/filters/passthrough.h>

Camera::Camera(int width, int height, int fps) {
    std::cout << width << height << fps;
    current_frame = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
}

cloud_pointer Camera::getCurrentFrame() {
    return current_frame;
}

void Camera::setFramesToSkip(int _frames_to_skip) {
    frames_to_skip = _frames_to_skip;
}