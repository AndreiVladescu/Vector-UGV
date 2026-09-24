#include "vector_hw/socketcan.hpp"

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace vector::can
{

void SocketCan::open(const std::string & ifname)
{
  close();
  fd_ = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
  if (fd_ < 0) {
    throw std::runtime_error(std::string("socket: ") + std::strerror(errno));
  }
  ifreq ifr{};
  std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
  if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
    const std::string err = std::strerror(errno);
    close();
    throw std::runtime_error(ifname + ": " + err);
  }
  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    const std::string err = std::strerror(errno);
    close();
    throw std::runtime_error("bind " + ifname + ": " + err);
  }
}

void SocketCan::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SocketCan::send(const Frame & f)
{
  can_frame cf{};
  cf.can_id = f.id;
  cf.len = f.len;
  std::memcpy(cf.data, f.data.data(), f.len);
  return ::write(fd_, &cf, sizeof(cf)) == static_cast<ssize_t>(sizeof(cf));
}

std::optional<Frame> SocketCan::receive()
{
  can_frame cf{};
  if (::read(fd_, &cf, sizeof(cf)) != static_cast<ssize_t>(sizeof(cf))) {
    return std::nullopt;
  }
  if (cf.can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG | CAN_EFF_FLAG)) {
    return std::nullopt;
  }
  Frame f;
  f.id = cf.can_id & CAN_SFF_MASK;
  f.len = cf.len;
  std::memcpy(f.data.data(), cf.data, cf.len);
  return f;
}

}  // namespace vector::can
