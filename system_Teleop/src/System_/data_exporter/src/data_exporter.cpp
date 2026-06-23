#include <rclcpp/rclcpp.hpp>
#include <system_interface/msg/frame_aligned_state.hpp>

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>
#include <chrono>
#include <functional>   // std::bind
#include <stdexcept>    // std::runtime_error
#include <filesystem>

#pragma pack(push, 1)
struct StateRecordBin
{
    char side[8];
    int64_t t_ns;                 // [ns]

    uint64_t frame_index;
    uint64_t seq;                  // sequence

    float gripper_sensor[30];
    float gripper_joint[20];
    float target_gripper_joint[20];

    float robot_joint[6];       // robot joints
    float target_robot_joint[6];
    float robot_pose[6];
    float robot_ft[6];
};
#pragma pack(pop)

static_assert(sizeof(StateRecordBin) == (8 + 8 + 8 + 8 + 30*4 + 20*4 + 20*4 + 6*4 + 6*4 + 6*4 + 6*4), "Unexpected size");

template <typename T>
class FrameRecordBuffer
{
public:
    explicit FrameRecordBuffer(size_t capacity)
    {
        if (capacity == 0 || (capacity & (capacity - 1)) != 0)
            throw std::runtime_error("Capacity must be power of two.");

        capacity_ = capacity;
        mask_ = capacity_ - 1;
        buffer_.resize(capacity_);
    }

    bool push(const T& v)
    {
        const size_t head = head_.load(std::memory_order_acquire);
        const size_t next = (head + 1) & mask_;

        if (next == tail_.load(std::memory_order_acquire))
            return false; // full

        buffer_[head] = v;
        head_.store(next, std::memory_order_release);
        return true;
    }

    bool pop(T& out)
    {
        const size_t tail = tail_.load(std::memory_order_acquire);

        if (tail == head_.load(std::memory_order_acquire))
            return false; // empty

        out = buffer_[tail];
        tail_.store((tail + 1) & mask_, std::memory_order_release);
        return true;
    }

    bool isEmpty()
    {
        const size_t tail = tail_.load(std::memory_order_acquire);
        return tail == head_.load(std::memory_order_acquire);
    }

private:
    size_t capacity_{0};
    size_t mask_{0};
    std::vector<T> buffer_;

    alignas(64) std::atomic<size_t> head_{0};
    alignas(64) std::atomic<size_t> tail_{0};
};

class DataExporterNode : public rclcpp::Node
{
public:
    DataExporterNode()
    : Node("data_exporter"),
      rBuf_(capacity_buf) // (1) 버퍼는 런타임 capacity로 생성
    {
        this->declare_parameter<std::string>("side", "left");
        this->declare_parameter<std::string>("output_path", "Record/");

        this->declare_parameter<int>("batch_size", 1024);
        this->declare_parameter<int>("flush_period_ms", 1000);

        side_ = this->get_parameter("side").as_string();
        output_path_ = this->get_parameter("output_path").as_string();
        batch_size_ = this->get_parameter("batch_size").as_int();
        flush_period_ms_ = this->get_parameter("flush_period_ms").as_int();

        topic_name_ = "/system_"+side_+"/frame_aligned_state";
        output_path_ = output_path_ + side_ +"/frame_data";

        std::string target_file;

        std::filesystem::path target_path(output_path_);
        std::filesystem::path output_dir = target_path.parent_path();

        if(!output_dir.empty())
        {
            std::filesystem::create_directories(output_dir);
        }

        while (true)
        {
            target_file = output_path_ + "_" + std::to_string(file_count++) + file_extension_;
            if(!std::filesystem::exists(target_file))      // 파일이 존재하지 않으면 break
            {
                output_path_ = target_file;
                break;
            }
        }

        fp_ = std::fopen(output_path_.c_str(), "wb");
        if (!fp_)
            throw std::runtime_error("Failed to open output file: " + output_path_ + file_extension_);

        thread_running_.store(true, std::memory_order_release);
        writer_thread_ = std::thread([this]() { this->loop_writer(); });

        using MsgT = system_interface::msg::FrameAlignedState;

        sub_ = this->create_subscription<MsgT>(
            topic_name_,
            rclcpp::QoS(500).best_effort(),
            std::bind(&DataExporterNode::recv_frame_data_callback, this, std::placeholders::_1)
        );

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&DataExporterNode::buf_state_callback, this)
        );

        RCLCPP_INFO(this->get_logger(),
                    "Exporting from topic '%s' to '%s'",
                    topic_name_.c_str(), output_path_.c_str());
    }

    ~DataExporterNode() override
    {
        thread_running_.store(false, std::memory_order_release);
        if (writer_thread_.joinable())
            writer_thread_.join();

        if (fp_)
        {
            std::fflush(fp_);
            std::fclose(fp_);
            fp_ = nullptr;
        }
    }

private:
    void recv_frame_data_callback(const system_interface::msg::FrameAlignedState::SharedPtr msg)
    {
        StateRecordBin rec{};
        // rec.side = msg->side;

        strncpy(rec.side, msg->side.c_str(), sizeof(rec.side));

        rec.t_ns  = msg->t_ns;
        rec.seq   = msg->seq;
        rec.frame_index = msg->frame_index;

        // RCLCPP_INFO(get_logger(), "Recv side is %s", rec.side);
        if(std::string(rec.side) == "left")
        {
            for (size_t i = 0; i < 5; i++)
            {
                for(size_t j = 0; j < 4; j++)
                {
                    rec.target_gripper_joint[4*i+j] = msg->target_gripper_joint[4*i+j];
                    rec.gripper_joint[4*i+j] = msg->gripper_joint[4*i+j];
                }

                for(size_t j = 0; j < 6; j++)
                {
                    rec.gripper_sensor[6*i+j] = msg->gripper_sensor[6*i+j];
                }
            }
        }
        else if (std::string(rec.side) == "right")
        {
            for (size_t i = 0; i < 6; ++i)
            {
                rec.target_gripper_joint[i] = msg->target_gripper_joint[i];
                rec.gripper_joint[i] = msg->gripper_joint[i];

                for(size_t j = 0; j < 5; j++)
                {
                    rec.gripper_sensor[5*i+j] = msg->gripper_sensor[5*i+j];
                }
            }
        }

        for (size_t i = 0; i < 6; ++i)
        { 
            rec.target_robot_joint[i] = msg->target_robot_joint[i];
            rec.robot_joint[i] = msg->robot_joint[i];
            rec.robot_pose[i] = msg->robot_pose[i];
            rec.robot_ft[i] = msg->robot_ft[i];
        }

        if (!rBuf_.push(rec))
            dropped_.fetch_add(1, std::memory_order_release);

        received_.fetch_add(1, std::memory_order_release);
    }

    void buf_state_callback()
    {
        const auto r = received_.load(std::memory_order_acquire);
        const auto d = dropped_.load(std::memory_order_acquire);

        RCLCPP_INFO(this->get_logger(),
                    "\nsub='%s' -> '%s' \nreceived=%llu \ndropped=%llu",
                    topic_name_.c_str(), output_path_.c_str(),
                    (unsigned long long)r, (unsigned long long)d);
    }

    void loop_writer()
    {
        std::vector<StateRecordBin> batch;
        batch.reserve(static_cast<size_t>(batch_size_));

        auto last_flush = std::chrono::steady_clock::now();

        while (thread_running_.load(std::memory_order_acquire) || !rBuf_.isEmpty())
        {
            StateRecordBin rec{};
            bool got_any = false;

            while (rBuf_.pop(rec))
            {
                got_any = true;
                batch.push_back(rec);
                if ((int)batch.size() >= batch_size_)
                    break;
            }

            if (!batch.empty())
            {
                std::fwrite(batch.data(), sizeof(StateRecordBin), batch.size(), fp_);
                batch.clear();
            }

            const auto now = std::chrono::steady_clock::now();
            if (now - last_flush > std::chrono::milliseconds(flush_period_ms_))
            {
                std::fflush(fp_);
                last_flush = now;
            }

            if (!got_any)
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        StateRecordBin rec{};
        while (rBuf_.pop(rec))
        {
            batch.push_back(rec);
            if ((int)batch.size() >= batch_size_)
            {
                std::fwrite(batch.data(), sizeof(StateRecordBin), batch.size(), fp_);
                batch.clear();
            }
        }
        if (!batch.empty())
            std::fwrite(batch.data(), sizeof(StateRecordBin), batch.size(), fp_);

        std::fflush(fp_);
    }

private:
    // ROS handles
    rclcpp::Subscription<system_interface::msg::FrameAlignedState>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // params
    std::string side_;
    std::string topic_name_;
    std::string output_path_;
    int batch_size_{1024};
    int flush_period_ms_{1000};

    // file
    std::string file_extension_ = ".bin";
    std::FILE* fp_{nullptr};

    static constexpr size_t capacity_buf = 16384;
    int file_count = 1;

    // buffer + worker
    FrameRecordBuffer<StateRecordBin> rBuf_;
    std::atomic<bool> thread_running_{false};
    std::thread writer_thread_;

    // stats
    std::atomic<uint64_t> received_{0};
    std::atomic<uint64_t> dropped_{0};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DataExporterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}