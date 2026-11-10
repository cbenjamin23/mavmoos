/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: UdpLoopbackIo.cpp                               */
/*    DATE: October 29, 2026                                */
/************************************************************/
#include "UdpLoopbackIo.h"
#include <arpa/inet.h>
#include <cerrno>
#include <fcntl.h>
#include <stdexcept>
#include <system_error>
#include <sys/socket.h>
#include <unistd.h>

namespace mavmoos::egress {
UdpLoopbackIo::UdpLoopbackIo(uint16_t peer_port, uint16_t bind_port)
  : m_peer_port(peer_port)
{
  if(peer_port == 0) throw std::invalid_argument("loopback peer port required");
  m_peer.sin_family = AF_INET;
  m_peer.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  m_peer.sin_port = htons(peer_port);
  m_socket = socket(AF_INET, SOCK_DGRAM, 0);
  if(m_socket < 0) throw std::system_error(errno, std::generic_category(), "UDP socket");
  sockaddr_in local{};
  local.sin_family = AF_INET;
  local.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  local.sin_port = htons(bind_port);
  const int flags = fcntl(m_socket, F_GETFL, 0);
  int buffer_bytes = 65536;
  if(flags < 0 || fcntl(m_socket, F_SETFL, flags | O_NONBLOCK) < 0 ||
     fcntl(m_socket, F_SETFD, FD_CLOEXEC) < 0 ||
     setsockopt(m_socket, SOL_SOCKET, SO_RCVBUF, &buffer_bytes, sizeof(buffer_bytes)) < 0 ||
     bind(m_socket, reinterpret_cast<sockaddr*>(&local), sizeof(local)) < 0) {
    const int error = errno;
    close();
    throw std::system_error(error, std::generic_category(), "UDP setup");
  }
  socklen_t length = sizeof(local);
  if(getsockname(m_socket, reinterpret_cast<sockaddr*>(&local), &length) < 0) {
    const int error = errno;
    close();
    throw std::system_error(error, std::generic_category(), "UDP getsockname");
  }
  m_port = ntohs(local.sin_port);
  if(m_port == peer_port) {
    close();
    throw std::invalid_argument("loopback local/peer ports must differ");
  }
}
UdpLoopbackIo::~UdpLoopbackIo() { close(); }
void UdpLoopbackIo::close() noexcept
{
  if(m_socket >= 0) ::close(m_socket);
  m_socket = -1;
}
WriteResult UdpLoopbackIo::tryWrite(const Frame& frame) noexcept
{
  if(m_socket < 0) return {WriteCode::Error, 0, EBADF};
  const auto size = sendto(m_socket, frame.bytes.data(), frame.size, 0,
                          reinterpret_cast<const sockaddr*>(&m_peer), sizeof(m_peer));
  if(size >= 0) return {WriteCode::Returned, static_cast<std::size_t>(size), 0};
  const int error = errno;
  return {error == EAGAIN || error == EWOULDBLOCK ? WriteCode::WouldBlock : WriteCode::Error,
          0, error};
}
ReadResult UdpLoopbackIo::tryRead() noexcept
{
  ReadResult result;
  if(m_socket < 0) { result.code = ReadCode::Closed; return result; }
  sockaddr_in from{};
  iovec io{result.bytes.data(), result.bytes.size()};
  msghdr header{};
  header.msg_name = &from; header.msg_namelen = sizeof(from);
  header.msg_iov = &io; header.msg_iovlen = 1;
  const auto size = recvmsg(m_socket, &header, 0);
  const int error = size < 0 ? errno : 0;
  if(size < 0) {
    result.code = error == EAGAIN || error == EWOULDBLOCK ? ReadCode::WouldBlock : ReadCode::Error;
    result.error = error;
  } else if(from.sin_family != AF_INET || from.sin_addr.s_addr != htonl(INADDR_LOOPBACK) ||
            ntohs(from.sin_port) != m_peer_port) {
    result.code = ReadCode::WrongPeer;
  } else if(header.msg_flags & MSG_TRUNC) {
    result.code = ReadCode::Truncated;
  } else {
    result.code = ReadCode::Data;
    result.size = static_cast<std::size_t>(size);
  }
  return result;
}
} // namespace mavmoos::egress
