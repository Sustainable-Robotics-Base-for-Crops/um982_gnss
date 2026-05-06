// Copyright 2026 SABI AGRI

#ifndef UM982_GNSS__TCP_HPP_
#define UM982_GNSS__TCP_HPP_

#include <functional>
#include <atomic>
#include <thread>
#include <vector>
#include <string>

class TCP
{
public:
  TCP()
  {
  }
  ~TCP()
  {
    close();
  }

  bool open(int port, const std::string& host, const std::function<void(const std::vector<uint8_t>&)>& callback);
  template <class T>
  bool open(int port, const std::string& host, void (T::*callback)(const std::vector<uint8_t>&), T* obj)
  {
    return open(port, host, std::bind(callback, obj, std::placeholders::_1));
  }
  bool send(const std::string& msg);
  void close();

private:
  std::atomic<int> sockfd_{ -1 };
  std::atomic<bool> stop_thread_{ true };
  std::function<void(const std::vector<uint8_t>&)> callback_;
  std::thread thread_;
  void receive();
};

#endif  // UM982_GNSS__TCP_HPP_
