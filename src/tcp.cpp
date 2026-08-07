// Copyright 2026 SABI AGRI

#include "um982_gnss/tcp.hpp"

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <string.h>

void TCP::close()
{
  stop_thread_ = true;

  if (sockfd_ >= 0)
  {
    // Interrupt the blocking receive without relying on a system-time polling timeout.
    ::shutdown(sockfd_, SHUT_RDWR);
  }

  if (thread_.joinable())
  {
    thread_.join();
  }

  if (sockfd_ < 0)
  {
    return;
  }

  ::close(sockfd_);
  sockfd_ = -1;
}

bool TCP::open(int port, const std::string& host, const std::function<void(const std::vector<uint8_t>&)>& callback)
{
  close();

  hostent* server = gethostbyname(host.c_str());

  if (!server)
  {
    return false;
  }

  if ((sockfd_ = socket(AF_INET, SOCK_STREAM, 0)) < 0)
  {
    return false;
  }

  sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  memcpy(&addr.sin_addr.s_addr, server->h_addr, server->h_length);

  if (connect(sockfd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
  {
    close();
    return false;
  }

  callback_ = callback;
  stop_thread_ = false;
  thread_ = std::thread(&TCP::receive, this);

  return true;
}

void TCP::receive()
{
  fd_set descriptors;

  while (!stop_thread_)
  {
    FD_ZERO(&descriptors);
    FD_SET(sockfd_, &descriptors);

    // Socket shutdown wakes this blocking wait when the client must stop or reconnect.
    const int select_result = select(sockfd_ + 1, &descriptors, nullptr, nullptr, nullptr);

    if (stop_thread_)
    {
      break;
    }

    if (select_result == 1)
    {
      std::vector<uint8_t> msg(4096);

      ssize_t nbytes = ::recv(sockfd_, msg.data(), msg.size(), 0);

      if (nbytes <= 0)
      {
        msg.resize(0);
      }
      else
      {
        msg.resize(nbytes);
      }

      callback_(msg);

      if (nbytes <= 0)
      {
        // EOF or a socket error cannot produce more data before a reconnection.
        break;
      }
    }
    else if (select_result < 0)
    {
      // Notify the owner about an unexpected select failure and terminate this receiver.
      callback_(std::vector<uint8_t>());
      break;
    }
  }
}

bool TCP::send(const std::string& msg)
{
  if (sockfd_ < 0)
  {
    return false;
  }

  // A disconnected peer must be reported to the caller instead of terminating the process with SIGPIPE.
  if (::send(sockfd_, msg.c_str(), msg.length(), MSG_NOSIGNAL) < 0)
  {
    return false;
  }

  return true;
}
