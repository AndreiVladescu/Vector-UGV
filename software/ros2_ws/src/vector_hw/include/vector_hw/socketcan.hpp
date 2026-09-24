// Minimal non-blocking SocketCAN (classic CAN) socket.
#pragma once

#include <optional>
#include <string>

#include "vector_hw/protocol.hpp"

namespace vector::can
{

class SocketCan
{
public:
  SocketCan() = default;
  ~SocketCan() {close();}
  SocketCan(const SocketCan &) = delete;
  SocketCan & operator=(const SocketCan &) = delete;

  // Throws std::runtime_error with the reason if the interface can't be opened.
  void open(const std::string & ifname);
  void close();
  bool is_open() const {return fd_ >= 0;}

  bool send(const Frame & f);
  std::optional<Frame> receive();  // nothing waiting -> empty

private:
  int fd_ = -1;
};

}  // namespace vector::can
