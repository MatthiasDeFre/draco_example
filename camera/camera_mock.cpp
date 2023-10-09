#include "camera_mock.h"
CameraMock::CameraMock(std::string _directory, int width, int height, int fps) : Camera(width, height, fps) {
    target_fps = fps;
    interframe_delay = std::chrono::milliseconds(1000 / target_fps);
    previous_time = std::chrono::steady_clock::now();
    directory = _directory;
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    try {
        for (const auto& entry : std::filesystem::directory_iterator(directory)) {
            if (std::filesystem::is_regular_file(entry)) {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::io::loadPLYFile(entry.path().string(), *cloud);
                frames.push_back(std::move(cloud));
            }
            n_files++;
        }
    }
    catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
    }

}
void CameraMock::captureFrame(bool filterBackground) {
    std::cout << "";
    current_frame = frames[rr_counter];
    // Need this function to enable precise sleep
    //timeBeginPeriod(1);
    std::this_thread::sleep_until(previous_time + std::chrono::duration<double, std::ratio<1>>(0.03333333333));
    previous_time = std::chrono::steady_clock::now();
    
    // Need to call end here for optimisation
    //timeEndPeriod(1);
    rr_counter = (rr_counter + 1) % n_files;
}

int CameraMock::getCurrentFrameNr() {
    return rr_counter;
}
int CameraMock::getMaxFrames() {
    return n_files;
}
void CameraMock::resetCamera() {
    rr_counter = 0;
}
int CameraMock::getFrameSize() {
    return current_frame->size();
}