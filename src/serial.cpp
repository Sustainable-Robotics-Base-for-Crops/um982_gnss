// Copyright 2026 SABI AGRI

#include "um982_gnss/serial.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <termios.h>

void Serial::close()
{
  stop_thread_ = true;

  if (thread_.joinable())
  {
    thread_.join();
  }

  if (fd_ < 0)
  {
    return;
  }

  ::close(fd_);
  fd_ = -1;
}

bool Serial::open(const std::string& device, int baudrate,
                  const std::function<void(const std::vector<uint8_t>&)>& callback)
{
  close();

  if ((fd_ = ::open(device.c_str(), O_RDWR)) < 0)
  {
    return false;
  }

  // Get the current serial port settings
  termios port_settings;
  memset(&port_settings, 0, sizeof(port_settings));

  if (tcgetattr(fd_, &port_settings) < 0)
  {
    return false;
  }

  // Input modes
  port_settings.c_iflag = IGNBRK;                    // Ignore BREAK condition on input
  port_settings.c_iflag |= IGNPAR;                   // Ignore framing errors and parity errors
  port_settings.c_iflag &= ~INPCK;                   // Disable input parity checking
  port_settings.c_iflag &= ~ISTRIP;                  // Not strip off eighth bit
  port_settings.c_iflag &= ~(IXON | IXOFF | IXANY);  // Disable flow control

  // Output modes
  port_settings.c_oflag = 0;

  // Control modes
  port_settings.c_cflag &= ~CSIZE;          // Clear size mask
  port_settings.c_cflag |= CS8;             // 8 bits per byte
  port_settings.c_cflag &= ~CSTOPB;         // One stop bit
  port_settings.c_cflag &= ~PARENB;         // Disable parity
  port_settings.c_cflag |= CREAD | CLOCAL;  // Enable receiver & ignore modem control lines
  port_settings.c_cflag &= ~CRTSCTS;        // Disable RTS/CTS (hardware) flow control

  // Local modes
  port_settings.c_lflag = 0;

  // Special characters
  port_settings.c_cc[VTIME] = 10;  // Wait for up to 10 deciseconds
  port_settings.c_cc[VMIN] = 0;    // Returning as soon as any data is received

  // Baud rate
  speed_t speed;

  switch (baudrate)
  {
    case 50:
      speed = B50;
      break;

    case 75:
      speed = B75;
      break;

    case 110:
      speed = B110;
      break;

    case 134:
      speed = B134;
      break;

    case 150:
      speed = B150;
      break;

    case 200:
      speed = B200;
      break;

    case 300:
      speed = B300;
      break;

    case 600:
      speed = B600;
      break;

    case 1200:
      speed = B1200;
      break;

    case 1800:
      speed = B1800;
      break;

    case 2400:
      speed = B2400;
      break;

    case 4800:
      speed = B4800;
      break;

    case 9600:
      speed = B9600;
      break;

    case 19200:
      speed = B19200;
      break;

    case 38400:
      speed = B38400;
      break;

    case 57600:
      speed = B57600;
      break;

    case 115200:
      speed = B115200;
      break;

    case 230400:
      speed = B230400;
      break;

    case 460800:
      speed = B460800;
      break;

    case 500000:
      speed = B500000;
      break;

    default:
      return false;
  }

  cfsetispeed(&port_settings, speed);
  cfsetospeed(&port_settings, speed);

  // Apply the modified settings
  if (tcsetattr(fd_, TCSANOW, &port_settings) < 0)
  {
    return false;
  }

  if (callback)
  {
    callback_ = callback;
    stop_thread_ = false;
    thread_ = std::thread(&Serial::read, this);
  }

  return true;
}

bool Serial::write(const std::string& data)
{
  if (fd_ < 0)
  {
    return false;
  }

  if (::write(fd_, data.c_str(), data.size()) < 0)
  {
    return false;
  }

  return true;
}

bool Serial::write(const std::vector<uint8_t>& data)
{
  if (fd_ < 0)
  {
    return false;
  }

  if (::write(fd_, data.data(), data.size()) < 0)
  {
    return false;
  }

  return true;
}

void Serial::read()
{
  while (!stop_thread_)
  {
    std::vector<uint8_t> data(256);

    ssize_t nbytes = ::read(fd_, data.data(), data.size());

    if (nbytes > 0)
    {
      data.resize(nbytes);
      callback_(data);
    }
  }
}
