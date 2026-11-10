/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: UdpLoopbackIo.h                                 */
/*    DATE: October 29, 2026                                */
/************************************************************/
#pragma once
#include "DatagramIo.h"
#include <netinet/in.h>

namespace mavmoos::egress {
// Lab-only: both local and peer addresses are hard-restricted to 127.0.0.1.
// Destination port is immutable; receive traffic cannot learn/redirect a peer.
class UdpLoopbackIo final : public DatagramIo {
public:
  explicit UdpLoopbackIo(uint16_t peer_port, uint16_t bind_port = 0);
  ~UdpLoopbackIo() override;
  UdpLoopbackIo(const UdpLoopbackIo&) = delete;
  UdpLoopbackIo& operator=(const UdpLoopbackIo&) = delete;
private:
  WriteResult tryWrite(const Frame& frame) noexcept override;
  ReadResult tryRead() noexcept override;
  bool isOpen() const noexcept override { return m_socket >= 0; }
  uint16_t localPort() const noexcept override { return m_port; }
  void close() noexcept override;
  int m_socket = -1;
  uint16_t m_port = 0;
  const uint16_t m_peer_port;
  sockaddr_in m_peer{};
};
} // namespace mavmoos::egress
