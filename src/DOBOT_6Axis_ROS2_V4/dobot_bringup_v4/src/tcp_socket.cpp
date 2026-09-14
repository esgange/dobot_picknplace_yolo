#include <dobot_bringup/tcp_socket.h>
#include <chrono>
#include <limits>

TcpClient::TcpClient(std::string ip, uint16_t port) : fd_(-1), port_(port), ip_(std::move(ip)), is_connected_(false)
{
}

TcpClient::~TcpClient()
{
    close();
}

void TcpClient::close()
{
    if (fd_ >= 0)
    {
        ::close(fd_);
        is_connected_ = false;
        fd_ = -1;
    }
}

void TcpClient::connect(uint32_t timeout_ms)
{
    if (timeout_ms == 0 || timeout_ms > static_cast<uint32_t>(std::numeric_limits<int>::max()))
        throw TcpClientException(toString() + " invalid connection timeout");

    if (fd_ < 0)
    {
        fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (fd_ < 0)
            throw TcpClientException(toString() + std::string(" socket : ") + strerror(errno));
    }

    sockaddr_in addr = {};

    memset(&addr, 0, sizeof(addr));
    if (::inet_pton(AF_INET, ip_.c_str(), &addr.sin_addr) != 1)
        throw TcpClientException(toString() + " invalid IPv4 address");
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port_);

    const int original_flags = ::fcntl(fd_, F_GETFL, 0);
    if (original_flags < 0)
        throw TcpClientException(toString() + std::string(" fcntl(F_GETFL) : ") + strerror(errno));
    if (::fcntl(fd_, F_SETFL, original_flags | O_NONBLOCK) < 0)
        throw TcpClientException(toString() + std::string(" fcntl(F_SETFL) : ") + strerror(errno));

    const int connect_result = ::connect(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
    if (connect_result < 0 && errno != EINPROGRESS)
    {
        const int error_code = errno;
        throw TcpClientException(toString() + std::string(" connect : ") + strerror(error_code));
    }

    if (connect_result < 0)
    {
        pollfd descriptor{};
        descriptor.fd = fd_;
        descriptor.events = POLLOUT;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

        while (true)
        {
            const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now()).count();
            if (remaining <= 0)
                throw TcpClientException(
                    toString() + " connect timed out after " + std::to_string(timeout_ms) + " ms");

            const int poll_result = ::poll(&descriptor, 1, static_cast<int>(remaining));
            if (poll_result == 0)
                throw TcpClientException(
                    toString() + " connect timed out after " + std::to_string(timeout_ms) + " ms");
            if (poll_result < 0)
            {
                if (errno == EINTR)
                    continue;
                const int error_code = errno;
                throw TcpClientException(toString() + std::string(" poll : ") + strerror(error_code));
            }

            int socket_error = 0;
            socklen_t error_length = sizeof(socket_error);
            if (::getsockopt(fd_, SOL_SOCKET, SO_ERROR, &socket_error, &error_length) < 0)
            {
                const int error_code = errno;
                throw TcpClientException(
                    toString() + std::string(" getsockopt(SO_ERROR) : ") + strerror(error_code));
            }
            if (socket_error != 0)
                throw TcpClientException(
                    toString() + std::string(" connect : ") + strerror(socket_error));
            break;
        }
    }

    if (::fcntl(fd_, F_SETFL, original_flags) < 0)
        throw TcpClientException(toString() + std::string(" restore socket flags : ") + strerror(errno));
    is_connected_ = true;

    RCLCPP_INFO(rclcpp::get_logger("TcpClient"), "connect successfully: %s", toString().c_str());
}

void TcpClient::disConnect()
{
    if (is_connected_)
    {
        is_connected_ = false;
        ::close(fd_);
        fd_ = -1;
    }
}

bool TcpClient::isConnect() const
{
    return is_connected_;
}

void TcpClient::tcpSend(const void *buf, uint32_t len)
{
    if (!is_connected_)
        throw TcpClientException("tcp is disconnected");

    const auto *tmp = (const uint8_t *)buf;
    while (len)
    {
        int err = (int)::send(fd_, tmp, len, MSG_NOSIGNAL);
        if (err < 0)
        {
            disConnect();
            throw TcpClientException(toString() + std::string(" ::send() ") + strerror(errno));
        }
        len -= err;
        tmp += err;
    }
}

bool TcpClient::tcpRecv(void *buf, uint32_t len, uint32_t &has_read, uint32_t timeout)
{
    uint8_t *tmp = (uint8_t *)buf;
    fd_set read_fds;
    timeval tv = {0, 0};

    has_read = 0;
    while (len > 0)
    {
        FD_ZERO(&read_fds);
        FD_SET(fd_, &read_fds);

        tv.tv_sec = timeout / 1000;
        tv.tv_usec = (timeout % 1000) * 1000;
        int err = ::select(fd_ + 1, &read_fds, nullptr, nullptr, &tv);
        if (err < 0)
        {
            disConnect();
            throw TcpClientException(toString() + std::string(" select() : ") + strerror(errno));
        }
        else if (err == 0)
        {
            return false;
        }

        err = (int)::read(fd_, tmp, len);
        if (err < 0)
        {
            disConnect();
            throw TcpClientException(toString() + std::string(" ::read() ") + strerror(errno));
        }
        else if (err == 0)
        {
            disConnect();
            throw TcpClientException(toString() + std::string(" tcp server has disconnected"));
        }

        has_read += err;
        
        for (int i = 0; i < err; ++i)
        {
            if (tmp[i] == ';')
            {
                return true;
            }
        }

        len -= err;
        tmp += err;
    }
    return true;
}

std::string TcpClient::toString()
{
    return ip_ + ":" + std::to_string(port_);
}
