// Copyright 2026 SABI AGRI

#ifndef UM982_GNSS__SERIAL_HPP_
#define UM982_GNSS__SERIAL_HPP_

#include <functional>
#include <atomic>
#include <thread>
#include <vector>
#include <string>

class Serial
{
public:
  Serial()
  {
  }
  ~Serial()
  {
    close();
  }

  bool open(const std::string& device, int baudrate,
            const std::function<void(const std::vector<uint8_t>&)>& callback = nullptr);
  template <class T>
  bool open(const std::string& device, int baudrate, void (T::*callback)(const std::vector<uint8_t>&), T* obj)
  {
    return open(device, baudrate, std::bind(callback, obj, std::placeholders::_1));
  }
  bool write(const std::string& data);
  bool write(const std::vector<uint8_t>& data);
  void close();

private:
  std::atomic<int> fd_{ -1 };
  std::atomic<bool> stop_thread_{ true };
  std::function<void(const std::vector<uint8_t>&)> callback_;
  std::thread thread_;
  void read();
};

#endif  // UM982_GNSS__SERIAL_HPP_
