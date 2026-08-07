// Copyright 2026 SABI AGRI

#include "um982_gnss/tcp.hpp"

#include <sys/socket.h>
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <unistd.h>
#include <string.h>

void TCP::interrupt()
{
  stop_thread_ = true;

  if (sockfd_ >= 0)
  {
    // Wake a blocking connect/receive operation without joining its thread.
    ::shutdown(sockfd_, SHUT_RDWR);
  }
}

void TCP::close()
{
  interrupt();

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

bool TCP::open(int port, const std::string& host, double timeout_seconds,
               const std::function<void(const std::vector<uint8_t>&)>& callback)
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

  // A non-blocking connect allows an explicit upper bound on unreachable servers.
  const int original_flags = fcntl(sockfd_, F_GETFL, 0);
  if (original_flags < 0 || fcntl(sockfd_, F_SETFL, original_flags | O_NONBLOCK) < 0)
  {
    close();
    return false;
  }

  const int connect_result = connect(sockfd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
  if (connect_result < 0 && errno != EINPROGRESS)
  {
    close();
    return false;
  }

  if (connect_result < 0)
  {
    fd_set writable;
    FD_ZERO(&writable);
    FD_SET(sockfd_, &writable);

    timeval timeout;
    timeout.tv_sec = static_cast<time_t>(timeout_seconds);
    timeout.tv_usec = static_cast<suseconds_t>((timeout_seconds - timeout.tv_sec) * 1000000.0);

    const int select_result = select(sockfd_ + 1, nullptr, &writable, nullptr, &timeout);
    int socket_error = 0;
    socklen_t socket_error_length = sizeof(socket_error);

    if (select_result != 1 ||
        getsockopt(sockfd_, SOL_SOCKET, SO_ERROR, &socket_error, &socket_error_length) < 0 ||
        socket_error != 0)
    {
      close();
      return false;
    }
  }

  // Receive uses blocking I/O and is interrupted explicitly by shutdown().
  if (fcntl(sockfd_, F_SETFL, original_flags) < 0)
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
