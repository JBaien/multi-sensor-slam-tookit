#include <ros/ros.h>
#include <cmath>
#include <sensor_msgs/Imu.h>
#include <modbus/modbus.h>
#include <thread>
#include <chrono>
#include <vector>
#include <cstring>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>

class IMUModbusNode {
private:
    ros::NodeHandle nh_;
    ros::Publisher imu_pub_;

    modbus_t* ctx_;
    std::string ip_address_;
    int port_;
    int start_register_;
    int num_registers_;
    bool connected_;
    int max_reconnect_attempts_;
    int reconnect_delay_ms_;
    std::vector<uint16_t> register_data_;

    // IMU 标定协方差
    double gyr_cov_x_, gyr_cov_y_, gyr_cov_z_;
    double acc_cov_x_, acc_cov_y_, acc_cov_z_;

    // 性能统计
    int publish_count_;
    ros::Time last_stats_time_;
    constexpr static double STATS_INTERVAL = 5.0; // 统计间隔5秒

public:
    IMUModbusNode() : nh_("~"), connected_(false), publish_count_(0) {
        nh_.param<std::string>("ip_address", ip_address_, "192.168.188.105");
        nh_.param<int>("port", port_, 502);
        nh_.param<int>("start_register", start_register_, 30);
        nh_.param<int>("num_registers", num_registers_, 56);
        nh_.param<int>("max_reconnect_attempts", max_reconnect_attempts_, 10);
        nh_.param<int>("reconnect_delay_ms", reconnect_delay_ms_, 1000);

        imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu_raw", 10);

        register_data_.resize(num_registers_);
        ctx_ = nullptr;

        last_stats_time_ = ros::Time::now();

        // 初始化协方差参数
        gyr_cov_x_ = 1.2968807717194922e-05 * 1.2968807717194922e-05;
        gyr_cov_y_ = 6.0873535629375565e-06 * 6.0873535629375565e-06;
        gyr_cov_z_ = 1.5389301887171400e-05 * 1.5389301887171400e-05;

        acc_cov_x_ = 3.0335331998782933e-03 * 3.0335331998782933e-03;
        acc_cov_y_ = 2.7319572898178790e-03 * 2.7319572898178790e-03;
        acc_cov_z_ = 3.2680420545776476e-03 * 3.2680420545776476e-03;

    }

    ~IMUModbusNode() {
        if (ctx_) {
            modbus_close(ctx_);
            modbus_free(ctx_);
        }
    }

    bool connect() {
        if (ctx_) {
            modbus_close(ctx_);
            modbus_free(ctx_);
        }
        ctx_ = modbus_new_tcp(ip_address_.c_str(), port_);
        if (!ctx_) {
            ROS_ERROR("Unable to allocate libmodbus context");
            return false;
        }
        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        modbus_set_response_timeout(ctx_, timeout.tv_sec, timeout.tv_usec);
        modbus_set_byte_timeout(ctx_, timeout.tv_sec, timeout.tv_usec);
        if (modbus_connect(ctx_) == -1) {
            ROS_ERROR("Connection failed: %s", modbus_strerror(errno));
            modbus_free(ctx_);
            ctx_ = nullptr;
            return false;
        }
        connected_ = true;
        ROS_INFO("Connected to IMU at %s:%d", ip_address_.c_str(), port_);
        return true;
    }

    bool reconnect() {
        ROS_WARN("Reconnecting...");
        connected_ = false;
        for (int attempt = 1; attempt <= max_reconnect_attempts_; ++attempt) {
            if (connect()) return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_delay_ms_));
        }
        return false;
    }

    float registersToIEEEFloat(uint16_t low, uint16_t high) {
        uint32_t combined = (static_cast<uint32_t>(high) << 16) | low;
        float result;
        std::memcpy(&result, &combined, sizeof(result));
        return result;
    }

    int32_t registersToInt32(uint16_t low, uint16_t high) {
        uint32_t combined = (static_cast<uint32_t>(high) << 16) | low;
        return static_cast<int32_t>(combined);
    }

    bool readRegisters() {
        if (!connected_ || !ctx_) return false;
        int ret = modbus_read_registers(ctx_, start_register_, num_registers_, register_data_.data());
        if (ret == -1) {
            connected_ = false;
            return false;
        }
        return true;
    }

    // 数据有效性检查
    bool isDataValid(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro) {
        // 使用isfinite检查所有元素（SIMD优化）
        if (!std::isfinite(acc(0)) || !std::isfinite(acc(1)) || !std::isfinite(acc(2)) ||
            !std::isfinite(gyro(0)) || !std::isfinite(gyro(1)) || !std::isfinite(gyro(2))) {
            return false;
        }

        // 检查加速度是否在合理范围内
        const double acc_sq_norm = acc.squaredNorm();
        if (acc_sq_norm < 0.9604 || acc_sq_norm > 240100.0) { // 避免sqrt调用
            return false;
        }

        // 检查角速度是否在合理范围内 (±2000 deg/s)
        constexpr double max_gyro_sq = 2000.0 * M_PI / 180.0;
        const double limit = max_gyro_sq * max_gyro_sq;
        if (gyro.squaredNorm() > limit) { // 使用squaredNorm避免sqrt
            return false;
        }

        return true;
    }

    // 四元数归一化函数
    inline void normalizeQuaternion(tf2::Quaternion& q) {
        double norm = std::sqrt(q.w()*q.w() + q.x()*q.x() + q.y()*q.y() + q.z()*q.z());
        if (norm > 1e-6) {
            q.setW(q.w() / norm);
            q.setX(q.x() / norm);
            q.setY(q.y() / norm);
            q.setZ(q.z() / norm);
        } else {
            q.setW(1.0);
            q.setX(0.0);
            q.setY(0.0);
            q.setZ(0.0);
            ROS_WARN("Quaternion norm too small, reset to identity");
        }
    }

    void processAndPublishData() {
        if (register_data_.size() < num_registers_) return;

        // 读取加速度计数据（INT32格式）
        Eigen::Vector3d acc(
            registersToInt32(register_data_[72 - start_register_], register_data_[73 - start_register_]) / 1e6,
            registersToInt32(register_data_[74 - start_register_], register_data_[75 - start_register_]) / 1e6,
            registersToInt32(register_data_[76 - start_register_], register_data_[77 - start_register_]) / 1e6
        );

        // 读取陀螺仪数据（INT32格式）
        Eigen::Vector3d gyro(
            (registersToInt32(register_data_[78 - start_register_], register_data_[79 - start_register_]) * M_PI / 180.0) / 1e6,
            (registersToInt32(register_data_[80 - start_register_], register_data_[81 - start_register_]) * M_PI / 180.0) / 1e6,
            (registersToInt32(register_data_[82 - start_register_], register_data_[83 - start_register_]) * M_PI / 180.0) / 1e6
        );

        // 读取原始姿态角（float格式）
        // 航向角：寄存器40032和40033
        Eigen::Vector3d rpy_rad;
        rpy_rad(0) = registersToIEEEFloat(register_data_[31 - start_register_], register_data_[32 - start_register_]) * M_PI / 180.0;  // 航向 (yaw)
        rpy_rad(1) = registersToIEEEFloat(register_data_[33 - start_register_], register_data_[34 - start_register_]) * M_PI / 180.0; // 横摇 (roll)
        rpy_rad(2) = registersToIEEEFloat(register_data_[35 - start_register_], register_data_[36 - start_register_]) * M_PI / 180.0; // 纵摇 (pitch)

        // 数据有效性检查
        if (!isDataValid(acc, gyro)) {
            ROS_WARN_THROTTLE(1.0, "Invalid IMU data detected, skipping this sample");
            return;
        }

        // 使用当前时间戳（直接使用，不需要dt计算）
        ros::Time now = ros::Time::now();

        // 将原始RPY角转换为四元数（用于LIO-SAM）
        tf2::Quaternion q;
        q.setRPY(rpy_rad(1), rpy_rad(2), rpy_rad(0)); // roll, pitch, yaw

        // 归一化四元数
        normalizeQuaternion(q);

        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = now;
        imu_msg.header.frame_id = "base_link";

        imu_msg.linear_acceleration.x = acc(0);
        imu_msg.linear_acceleration.y = acc(1);
        imu_msg.linear_acceleration.z = acc(2);

        imu_msg.angular_velocity.x = gyro(0);
        imu_msg.angular_velocity.y = gyro(1);
        imu_msg.angular_velocity.z = gyro(2);

        // 使用原始姿态角转换的四元数
        imu_msg.orientation.w = q.w();
        imu_msg.orientation.x = q.x();
        imu_msg.orientation.y = q.y();
        imu_msg.orientation.z = q.z();

        // 正确设置协方差矩阵（9元素数组，行主序）
        // 姿态协方差矩阵
        imu_msg.orientation_covariance[0] = 0.001; // xx
        imu_msg.orientation_covariance[1] = 0.0;
        imu_msg.orientation_covariance[2] = 0.0;
        imu_msg.orientation_covariance[3] = 0.0;
        imu_msg.orientation_covariance[4] = 0.001; // yy
        imu_msg.orientation_covariance[5] = 0.0;
        imu_msg.orientation_covariance[6] = 0.0;
        imu_msg.orientation_covariance[7] = 0.0;
        imu_msg.orientation_covariance[8] = 0.001; // zz

        // 角速度协方差矩阵
        imu_msg.angular_velocity_covariance[0] = gyr_cov_x_; // xx
        imu_msg.angular_velocity_covariance[1] = 0.0;
        imu_msg.angular_velocity_covariance[2] = 0.0;
        imu_msg.angular_velocity_covariance[3] = 0.0;
        imu_msg.angular_velocity_covariance[4] = gyr_cov_y_; // yy
        imu_msg.angular_velocity_covariance[5] = 0.0;
        imu_msg.angular_velocity_covariance[6] = 0.0;
        imu_msg.angular_velocity_covariance[7] = 0.0;
        imu_msg.angular_velocity_covariance[8] = gyr_cov_z_; // zz

        // 线性加速度协方差矩阵
        imu_msg.linear_acceleration_covariance[0] = acc_cov_x_; // xx
        imu_msg.linear_acceleration_covariance[1] = 0.0;
        imu_msg.linear_acceleration_covariance[2] = 0.0;
        imu_msg.linear_acceleration_covariance[3] = 0.0;
        imu_msg.linear_acceleration_covariance[4] = acc_cov_y_; // yy
        imu_msg.linear_acceleration_covariance[5] = 0.0;
        imu_msg.linear_acceleration_covariance[6] = 0.0;
        imu_msg.linear_acceleration_covariance[7] = 0.0;
        imu_msg.linear_acceleration_covariance[8] = acc_cov_z_; // zz

        // 发布完整的LIO-SAM专用IMU数据
        imu_pub_.publish(imu_msg);

        // 性能统计
        publish_count_++;
        if ((now - last_stats_time_).toSec() >= STATS_INTERVAL) {
            double freq = publish_count_ / STATS_INTERVAL;
            ROS_INFO_THROTTLE(STATS_INTERVAL, "IMU publish rate: %.1f Hz", freq);
            publish_count_ = 0;
            last_stats_time_ = now;
        }
    }

    void run() {
        ros::Rate rate(400); // 400 Hz
        if (!connect()) return;
        while (ros::ok()) {
            if (!connected_ && !reconnect()) break;
            if (readRegisters()) {
                processAndPublishData();
            } else {
                connected_ = false;
            }
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_modbus_node");
    IMUModbusNode node;
    node.run();
    return 0;
}
