#include <dobot_bringup/command.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <rclcpp/rclcpp.hpp>
CRCommanderRos2::CRCommanderRos2(
    const std::string &lan1_ip,
    const std::string &lan2_ip,
    uint32_t connection_timeout_ms,
    const std::shared_ptr<dobot_bringup::EventLogger> &event_logger)
    : current_joint_{}, tool_vector_{}, is_running_(false),
      lan1_ip_(lan1_ip), lan2_ip_(lan2_ip), connection_timeout_ms_(connection_timeout_ms),
      event_logger_(event_logger)
{
    if (!event_logger_) {
        throw std::invalid_argument("Dobot commander requires a datalog logger");
    }
    if (connection_timeout_ms_ < 100 || connection_timeout_ms_ > 60000) {
        throw std::invalid_argument("Dobot connection timeout must be from 100 through 60000 ms");
    }
    real_time_data_ = std::make_shared<RealTimeData>();
}

CRCommanderRos2::~CRCommanderRos2()
{
    is_running_ = false;
    if (thread_ && thread_->joinable()) {
        thread_->join();
    }
    disconnectActive();
}

bool CRCommanderRos2::tryConnect(const std::string &ip, const std::string &interface_name)
{
    disconnectActive();
    const std::string attempt_message = interface_name + " ip=" + ip +
        " timeout_ms=" + std::to_string(connection_timeout_ms_);
    event_logger_->record("INFO", "robot_connection_attempt", attempt_message);
    RCLCPP_INFO(
        rclcpp::get_logger("CRCommanderRos2"),
        "Attempting Dobot connection via %s (%s), timeout %u ms per TCP channel",
        interface_name.c_str(), ip.c_str(), connection_timeout_ms_);

    auto realtime = std::make_shared<TcpClient>(ip, 30004);
    auto dashboard = std::make_shared<TcpClient>(ip, 29999);
    try {
        realtime->connect(connection_timeout_ms_);
        dashboard->connect(connection_timeout_ms_);
    } catch (const TcpClientException &err) {
        realtime->close();
        dashboard->close();
        const std::string message = interface_name + " ip=" + ip + " error=" + err.what();
        event_logger_->record("WARN", "robot_connection_attempt_failed", message);
        RCLCPP_WARN(rclcpp::get_logger("CRCommanderRos2"), "%s", message.c_str());
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(connection_mutex_);
        real_time_tcp_ = realtime;
        dash_board_tcp_ = dashboard;
        active_ip_ = ip;
    }
    event_logger_->record(
        "INFO", "robot_connection_result",
        "status=connected interface=" + interface_name + " ip=" + ip);
    RCLCPP_INFO(rclcpp::get_logger("CRCommanderRos2"),
        "Connected to Dobot via %s (%s)", interface_name.c_str(), ip.c_str());
    return true;
}

void CRCommanderRos2::disconnectActive()
{
    std::shared_ptr<TcpClient> realtime;
    std::shared_ptr<TcpClient> dashboard;
    {
        std::lock_guard<std::mutex> lock(connection_mutex_);
        realtime.swap(real_time_tcp_);
        dashboard.swap(dash_board_tcp_);
        active_ip_.clear();
    }
    if (realtime) {
        realtime->close();
    }
    if (dashboard) {
        dashboard->close();
    }
}

std::shared_ptr<TcpClient> CRCommanderRos2::dashboardClient() const
{
    std::lock_guard<std::mutex> lock(connection_mutex_);
    return dash_board_tcp_;
}

std::shared_ptr<TcpClient> CRCommanderRos2::realTimeClient() const
{
    std::lock_guard<std::mutex> lock(connection_mutex_);
    return real_time_tcp_;
}

void CRCommanderRos2::getCurrentJointStatus(double *joint)
{
    mutex_.lock();
    memcpy(joint, current_joint_, sizeof(current_joint_));
    mutex_.unlock();
}

void CRCommanderRos2::getToolVectorActual(double *val)
{
    mutex_.lock();
    memcpy(val, tool_vector_, sizeof(tool_vector_));
    mutex_.unlock();
}

void CRCommanderRos2::recvTask()
{
    bool outage_logged = false;
    while (is_running_)
    {
        if (!isConnected()) {
            bool connected = tryConnect(lan1_ip_, "LAN1");
            if (!connected) {
                connected = tryConnect(lan2_ip_, "LAN2");
            }
            if (!connected) {
                if (!outage_logged) {
                    const std::string message =
                        "status=failed LAN1_ip=" + lan1_ip_ + " LAN2_ip=" + lan2_ip_ +
                        " timeout_ms=" + std::to_string(connection_timeout_ms_);
                    event_logger_->record("ERROR", "robot_connection_result", message);
                    RCLCPP_ERROR(rclcpp::get_logger("CRCommanderRos2"),
                        "Dobot connection failed on both configured interfaces: %s",
                        message.c_str());
                    outage_logged = true;
                }
                std::this_thread::sleep_for(std::chrono::seconds(3));
                continue;
            }
            if (outage_logged) {
                event_logger_->record("INFO", "robot_connection_recovered", "configured interface connected");
                RCLCPP_INFO(rclcpp::get_logger("CRCommanderRos2"), "Dobot connection recovered");
                outage_logged = false;
            }
        }

        const auto realtime = realTimeClient();
        if (!realtime || !realtime->isConnect()) {
            disconnectActive();
            continue;
        }

        try {
            uint32_t has_read;
            uint8_t *tmpData = reinterpret_cast<uint8_t *>(real_time_data_.get());
            if (realtime->tcpRecv(tmpData, sizeof(RealTimeData), has_read, 5000)) {
                if (real_time_data_->len != 1440) {
                    continue;
                }

                std::lock_guard<std::mutex> lock(mutex_);
                for (uint32_t i = 0; i < 6; i++) {
                    current_joint_[i] = deg2Rad(real_time_data_->q_actual[i]);
                }
                memcpy(tool_vector_, real_time_data_->tool_vector_actual, sizeof(tool_vector_));
            } else {
                RCLCPP_WARN(rclcpp::get_logger("CRCommanderRos2"), "tcp recv timeout");
            }
        } catch (const TcpClientException &err) {
            const std::string message = std::string("active robot connection error: ") + err.what();
            event_logger_->record("WARN", "robot_connection_lost", message);
            RCLCPP_ERROR(rclcpp::get_logger("CRCommanderRos2"), "%s", message.c_str());
            disconnectActive();
        }
    }
}

void CRCommanderRos2::init()
{
    try
    {
        is_running_ = true;
        thread_ = std::unique_ptr<std::thread>(new std::thread(&CRCommanderRos2::recvTask, this));
    }
    catch (const TcpClientException &err)
    {
        RCLCPP_ERROR(rclcpp::get_logger("CRCommanderRos2"), "Commander: %s", err.what());
    }
}
int stringToInt(const std::string& str) {
    return std::atoi(str.c_str());
}
void CRCommanderRos2::doTcpCmd(std::shared_ptr<TcpClient> &tcp, const char *cmd, int32_t &err_id,
                               std::vector<std::string> &result)
{
    std::ignore = result;
    try
    {
        uint32_t has_read;
        char buf[1024];
        memset(buf, 0, sizeof(buf));
        auto currentTime = std::chrono::system_clock::now();
        auto currentTime_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(currentTime);
        auto valueMS = currentTime_ms.time_since_epoch().count();
        RCLCPP_INFO(rclcpp::get_logger("CRCommanderRos2"), "time: %ld  tcp send cmd : %s", valueMS, cmd);

        tcp->tcpSend(cmd, strlen(cmd));
        char *recv_ptr = buf;
        while (true)
        {
            bool err = tcp->tcpRecv(recv_ptr, 1024, has_read, 0);
            if (!err)
            {
                sleep(0.01);
                continue;
            }
            if (*(recv_ptr + strlen(recv_ptr) - 1) == ';')
                break;

            recv_ptr = recv_ptr + strlen(recv_ptr);
        }
        int data_len = strlen(buf);
        for (int i = 0; i < data_len; i++)
        {
            if (buf[i] == '{')
            {
                std::string str(buf);
                std::string result = str.substr(0, i-1);
                int num = stringToInt(result);
                err_id = num;
                RCLCPP_INFO(rclcpp::get_logger("CRCommanderRos2"), "ErrorID: %s", result.c_str());
                break;
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("CRCommanderRos2"), "tcp recv feedback : %s", buf);
    }
    catch (const TcpClientException &err)
    {
        RCLCPP_ERROR(rclcpp::get_logger("CRCommanderRos2"), "tcpDoCmd failed: %s", err.what());
    }
}


void CRCommanderRos2::doTcpCmd_f(std::shared_ptr<TcpClient> &tcp, const char *cmd, int32_t &err_id,std::string &mode_id,
                               std::vector<std::string> &result)
{
    std::ignore = result;
    try
    {
        uint32_t has_read;
        char buf[1024];
        memset(buf, 0, sizeof(buf));
        auto currentTime = std::chrono::system_clock::now();
        auto currentTime_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(currentTime);
        auto valueMS = currentTime_ms.time_since_epoch().count();
        RCLCPP_INFO(rclcpp::get_logger("CRCommanderRos2"), "time: %ld  tcp send cmd : %s", valueMS, cmd);
        tcp->tcpSend(cmd, strlen(cmd));
        char *recv_ptr = buf;
        while (true)
        {
            bool err = tcp->tcpRecv(recv_ptr, 1024, has_read, 0);
            if (!err)
            {
                sleep(0.01);
                continue;
            }
            if (*(recv_ptr + strlen(recv_ptr) - 1) == ';')
                break;

            recv_ptr = recv_ptr + strlen(recv_ptr);
        }
        int pose1 = 0;
        int data_len = strlen(buf);
        for (int i = 0; i < data_len; i++)
        {
            if (buf[i] == '{')
            {
                std::string str(buf);
                std::string result = str.substr(0, i-1);
                int num = stringToInt(result);
                err_id = num;
                RCLCPP_INFO(rclcpp::get_logger("CRCommanderRos2"), "ErrorID: %d", num);
                pose1 = i;
            }
            if (buf[i] == '}')
            {
                std::string str(buf);
                std::string result = str.substr(pose1, i-pose1+1);
                mode_id = result;
                break;
            }
        }
        RCLCPP_INFO(rclcpp::get_logger("CRCommanderRos2"), "tcp recv feedback : %s", buf);
    }
    catch (const TcpClientException &err)
    {
        RCLCPP_ERROR(rclcpp::get_logger("CRCommanderRos2"), "tcpDoCmd_f failed: %s", err.what());
    }
}

bool CRCommanderRos2::callRosService(const std::string cmd, int32_t &err_id)
{
    try
    {
        std::vector<std::string> result_;
        auto dashboard = dashboardClient();
        if (!dashboard) {
            throw TcpClientException("robot dashboard connection is not selected");
        }
        doTcpCmd(dashboard, cmd.c_str(), err_id, result_);
        return true;
    }
    catch (const TcpClientException &err)
    {
        RCLCPP_ERROR(rclcpp::get_logger("CRCommanderRos2"), "callRosService: %s", err.what());
        err_id = -1;
        return false;
    }
}
bool CRCommanderRos2::callRosService_f(const std::string cmd, int32_t &err_id,std::string &mode_id)
{
    try
    {
        std::vector<std::string> result_;
        auto dashboard = dashboardClient();
        if (!dashboard) {
            throw TcpClientException("robot dashboard connection is not selected");
        }
        doTcpCmd_f(dashboard, cmd.c_str(), err_id,mode_id, result_);
        return true;
    }
    catch (const TcpClientException &err)
    {
        RCLCPP_ERROR(rclcpp::get_logger("CRCommanderRos2"), "callRosService_f: %s", err.what());
        err_id = -1;
        return false;
    }
}
bool CRCommanderRos2::callRosService(const std::string cmd, int32_t &err_id, std::vector<std::string> &result_)
{
    try
    {
        auto dashboard = dashboardClient();
        if (!dashboard) {
            throw TcpClientException("robot dashboard connection is not selected");
        }
        doTcpCmd(dashboard, cmd.c_str(), err_id, result_);
        return true;
    }
    catch (const TcpClientException &err)
    {
        RCLCPP_ERROR(rclcpp::get_logger("CRCommanderRos2"), "callRosService: %s", err.what());
        err_id = -1;
        return false;
    }
}

bool CRCommanderRos2::isEnable() const
{
    return real_time_data_->robot_mode == 5;
}

bool CRCommanderRos2::isConnected() const
{
    const auto dashboard = dashboardClient();
    const auto realtime = realTimeClient();
    return dashboard && realtime && dashboard->isConnect() && realtime->isConnect();
}

uint16_t CRCommanderRos2::getRobotMode() const
{
    return real_time_data_->robot_mode;
}

std::shared_ptr<RealTimeData> CRCommanderRos2::getRealData() const
{
    return real_time_data_;
}
