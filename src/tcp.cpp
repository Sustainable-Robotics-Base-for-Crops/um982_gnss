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
  timeval timeout;

  while (!stop_thread_)
  {
    FD_ZERO(&descriptors);
    FD_SET(sockfd_, &descriptors);

    timeout.tv_sec = 1;
    timeout.tv_usec = 0;

    if (select(sockfd_ + 1, &descriptors, nullptr, nullptr, &timeout) == 1)
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
    }
  }
}

bool TCP::send(const std::string& msg)
{
  if (sockfd_ < 0)
  {
    return false;
  }

  if (::send(sockfd_, msg.c_str(), msg.length(), 0) < 0)
  {
    return false;
  }

  return true;
}
