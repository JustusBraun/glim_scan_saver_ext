#include <glim/util/extension_module.hpp>
#include <glim/util/concurrent_vector.hpp>
#include <glim/util/logging.hpp>
#include <glim/common/cloud_deskewing.hpp>
#include <glim/mapping/callbacks.hpp>

#include <thread>
#include <filesystem>
#include <format>

#include <happly.h>

namespace glim_scan_saver_ext
{

class GlimScanSaver: public glim::ExtensionModule
{

public:
    GlimScanSaver();

    /**
    * @brief New odometry estimation inserted into submap
    */
    void onInsertFrameIntoSubmap(const glim::EstimationFrame::ConstPtr& frame);

    virtual void at_exit(const std::string& dump_path);

    virtual ~GlimScanSaver();

private:

    /**
    *   @brief Deskews and saves frames from the queue_
    */
    void threadRun();

private:
    // Queue to store the incomming frames
    glim::ConcurrentVector<glim::EstimationFrame::Ptr> queue_;

    // Thread to handle the deskewing and saving to prevent blocking GLIM
    std::thread thread_;

    // TMP directory to store the frames in
    std::filesystem::path output_dir_;

    // First received timestamp
    std::optional<double> first_stamp_;

    // Last reveived timestamp
    double last_stamp_;

    std::shared_ptr<spdlog::logger> logger_;
};


GlimScanSaver::GlimScanSaver()
: output_dir_(std::filesystem::temp_directory_path() / "glim_scan_saver")
{
    logger_ = glim::create_module_logger("glim_scan_saver_ext");
    // Register the callback
    glim::SubMappingCallbacks::on_insert_frame.add(
        std::bind(&GlimScanSaver::onInsertFrameIntoSubmap, this, std::placeholders::_1)
    );

    thread_ = std::thread(std::bind(&GlimScanSaver::threadRun, this));
}

GlimScanSaver::~GlimScanSaver()
{
    queue_.submit_end_of_data();
}


void GlimScanSaver::at_exit(const std::string& dump_path)
{
    queue_.submit_end_of_data();
    thread_.join();

    // Move our temp directory to the dump directory
    std::filesystem::rename(output_dir_, std::filesystem::path(dump_path) / "deskewed_scans");

    logger_->info("First frame timestamp: {} Last frame timestamp: {}", first_stamp_.value_or(0), last_stamp_);
}

void GlimScanSaver::onInsertFrameIntoSubmap(const glim::EstimationFrame::ConstPtr& frame)
{
    // Create a clone to access the frame safely from a different thread
    const auto clone = frame->clone();

    // Save the timestamp
    if (!first_stamp_)
    {
        first_stamp_ = clone->stamp;
    }
    last_stamp_ = clone->stamp;

    // Push to thread safe container
    queue_.push_back(clone);
}

void GlimScanSaver::threadRun()
{
    // Create the output directory
    namespace fs = std::filesystem;
    
    // Remove directory from previous aborted runs
    if (fs::exists(output_dir_))
    {
        if (fs::remove_all(output_dir_))
        {
            logger_->debug("Removed old temp directory {}", output_dir_.string());
        }
        else
        {
            logger_->error("Failed to remove old temp directory {}", output_dir_.string());
            return;
        }
    }

    // Create the temp directory
    if (!fs::create_directories(output_dir_))
    {
        logger_->error("Could not create output directory {}", output_dir_.string());
        return;
    }
    
    glim::CloudDeskewing deskew;
    size_t i = 0;
    // pop_wait waits until a frame comes or end of data flag is set on the queue
    while(auto opt = queue_.pop_wait())
    {
        glim::EstimationFrame::ConstPtr frame = opt.value();

        // Taken from glim/mapping/sub_mapping.cpp
        std::vector<double> imu_pred_times(frame->imu_rate_trajectory.cols());
        std::vector<Eigen::Isometry3d> imu_pred_poses(frame->imu_rate_trajectory.cols());
        for (int i = 0; i < frame->imu_rate_trajectory.cols(); i++) {
          const Eigen::Matrix<double, 8, 1> imu = frame->imu_rate_trajectory.col(i).transpose();
          imu_pred_times[i] = imu[0];
          imu_pred_poses[i].setIdentity();
          imu_pred_poses[i].translation() << imu[1], imu[2], imu[3];
          imu_pred_poses[i].linear() = Eigen::Quaterniond(imu[7], imu[4], imu[5], imu[6]).toRotationMatrix();
        }
        
        std::vector<Eigen::Vector4d> deskewed;
        if (frame->raw_frame->raw_points)
        {
            deskewed = deskew.deskew(
                frame->T_lidar_imu.inverse(),
                imu_pred_times, imu_pred_poses,
                frame->raw_frame->raw_points->stamp,
                frame->raw_frame->raw_points->times,
                frame->raw_frame->raw_points->points
            );
        }
        else
        {
            deskewed = deskew.deskew(
                frame->T_lidar_imu.inverse(),
                imu_pred_times, imu_pred_poses,
                frame->raw_frame->stamp,
                frame->raw_frame->times,
                frame->raw_frame->points
            );
        }

        // Write the deskewed frame
        const fs::path filename = output_dir_ / std::format("{}.ply", i);
        
        // Convert points to happly format
        std::vector<std::array<double, 3>> vertices(deskewed.size());
        std::transform(
            deskewed.begin(), deskewed.end(), vertices.begin(),
            [](const Eigen::Vector4d& point) -> std::array<double, 3>
            {
                return {point.x(), point.y(), point.z()};
            }
        );

        happly::PLYData ply_out;
        ply_out.addVertexPositions(vertices);
        ply_out.write(filename, happly::DataFormat::Binary);
        i++;
    }
}

} // glim_scan_saver_ext

// Factory for extension modules
extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim_scan_saver_ext::GlimScanSaver();
}
