#ifndef DOBOT_BRINGUP__EVENT_LOGGER_HPP_
#define DOBOT_BRINGUP__EVENT_LOGGER_HPP_

#include <chrono>
#include <cctype>
#include <cstddef>
#include <ctime>
#include <cerrno>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>

namespace dobot_bringup
{

class EventLogger
{
public:
  static constexpr std::size_t kMaxEvents = 1000;

  EventLogger(const std::string & package_name, const std::string & log_root)
  : package_name_(package_name), log_path_(log_root + "/" + package_name + "/events.jsonl")
  {
    validatePackageName(package_name_);
    makeDirectory(log_root);
    makeDirectory(log_root + "/" + package_name_);
  }

  bool record(
    const std::string & level,
    const std::string & event,
    const std::string & message) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    try {
      if (countEvents() >= kMaxEvents) {
        std::ofstream reset(log_path_, std::ios::out | std::ios::trunc);
        if (!reset) {
          return false;
        }
      }

      std::ofstream output(log_path_, std::ios::out | std::ios::app);
      if (!output) {
        return false;
      }
      output << "{\"timestamp\":\"" << timestamp() << "\","
             << "\"package\":\"" << escapeJson(package_name_) << "\","
             << "\"level\":\"" << escapeJson(level) << "\","
             << "\"event\":\"" << escapeJson(event) << "\","
             << "\"message\":\"" << escapeJson(message) << "\"}\n";
      return static_cast<bool>(output);
    } catch (...) {
      return false;
    }
  }

  const std::string & path() const
  {
    return log_path_;
  }

private:
  static void validatePackageName(const std::string & package_name)
  {
    if (package_name.empty()) {
      throw std::invalid_argument("datalog package name cannot be empty");
    }
    for (const char character : package_name) {
      const auto value = static_cast<unsigned char>(character);
      if (!(std::isalnum(value) || character == '_' || character == '-' || character == '.')) {
        throw std::invalid_argument("datalog package name contains a path character");
      }
    }
  }

  static void makeDirectory(const std::string & directory)
  {
    if (directory.empty()) {
      throw std::invalid_argument("datalog directory cannot be empty");
    }

    std::string current;
    std::size_t start = 0;
    if (directory.front() == '/') {
      current = "/";
      start = 1;
    }

    while (start <= directory.size()) {
      const std::size_t end = directory.find('/', start);
      const std::string component = directory.substr(
        start, end == std::string::npos ? std::string::npos : end - start);
      if (!component.empty()) {
        if (!current.empty() && current.back() != '/') {
          current += '/';
        }
        current += component;
        if (::mkdir(current.c_str(), 0755) < 0 && errno != EEXIST) {
          throw std::runtime_error("cannot create datalog directory: " + current);
        }
      }
      if (end == std::string::npos) {
        break;
      }
      start = end + 1;
    }
  }

  std::size_t countEvents() const
  {
    std::ifstream input(log_path_);
    if (!input && errno != ENOENT) {
      return kMaxEvents;
    }
    std::size_t count = 0;
    std::string line;
    while (std::getline(input, line)) {
      ++count;
    }
    return count;
  }

  static std::string timestamp()
  {
    const auto now = std::chrono::system_clock::now();
    const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
      now.time_since_epoch()) % 1000;
    const std::time_t seconds = std::chrono::system_clock::to_time_t(now);
    std::tm utc{};
    gmtime_r(&seconds, &utc);

    std::ostringstream output;
    output << std::put_time(&utc, "%Y-%m-%dT%H:%M:%S")
           << '.' << std::setfill('0') << std::setw(3) << milliseconds.count() << 'Z';
    return output.str();
  }

  static std::string escapeJson(const std::string & value)
  {
    std::ostringstream output;
    output << std::hex << std::setfill('0');
    for (const unsigned char character : value) {
      switch (character) {
        case '"': output << "\\\""; break;
        case '\\': output << "\\\\"; break;
        case '\b': output << "\\b"; break;
        case '\f': output << "\\f"; break;
        case '\n': output << "\\n"; break;
        case '\r': output << "\\r"; break;
        case '\t': output << "\\t"; break;
        default:
          if (character < 0x20) {
            output << "\\u" << std::setw(4) << static_cast<int>(character);
          } else {
            output << character;
          }
      }
    }
    return output.str();
  }

  std::string package_name_;
  std::string log_path_;
  mutable std::mutex mutex_;
};

}  // namespace dobot_bringup

#endif  // DOBOT_BRINGUP__EVENT_LOGGER_HPP_
