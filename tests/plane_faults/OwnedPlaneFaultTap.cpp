// Native test only. Interpose on the app's one nominated IPv4 UDP tuple.
// No routes, sockets, peer identities or production guards are created here.
#include <ardupilotmega/mavlink.h>

#include <arpa/inet.h>
#include <array>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <fcntl.h>
#include <mutex>
#include <string_view>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/uio.h>
#include <time.h>
#include <unistd.h>

namespace {
constexpr std::size_t trace_limit = 16 * 1024 * 1024;
constexpr std::size_t datagram_limit = 4096;
constexpr std::size_t header_size = 19;
constexpr uint64_t hold_limit_ns = 60ULL * 1000000000;

enum class Mode { Invalid, Pass, DropRx, DropTx, DropLoiter, HoldPoint, ReleasePoint, DropModeAck, DropTakeoffAck, DropRtlAck, ExitBeforeTakeoff, HoldHover, DropHover };
enum class Command { Other, Loiter, Point, Takeoff };

uint16_t port(const char* key)
{
  const char* input = std::getenv(key);
  if(!input || !*input) return 0;
  char* end = nullptr;
  const unsigned long value = std::strtoul(input, &end, 10);
  return *end == '\0' && value > 0 && value <= 65535 ? static_cast<uint16_t>(value) : 0;
}

bool loopbackPort(const sockaddr* address, socklen_t length, uint16_t expected)
{
  if(!address || length < sizeof(sockaddr_in) || address->sa_family != AF_INET) return false;
  const auto* ipv4 = reinterpret_cast<const sockaddr_in*>(address);
  return ipv4->sin_addr.s_addr == htonl(INADDR_LOOPBACK) && ntohs(ipv4->sin_port) == expected;
}

uint64_t timestamp()
{
  timespec stamp{};
#ifdef __APPLE__
  constexpr clockid_t clock = CLOCK_MONOTONIC_RAW;
#else
  constexpr clockid_t clock = CLOCK_MONOTONIC;
#endif
  if(clock_gettime(clock, &stamp) != 0) return 0;
  return static_cast<uint64_t>(stamp.tv_sec) * 1000000000 + stamp.tv_nsec;
}

void encode(uint8_t* out, uint64_t value, unsigned length)
{
  for(unsigned i = 0; i < length; ++i) {
    out[i] = static_cast<uint8_t>(value);
    value >>= 8;
  }
}

Mode readMode(const char* path)
{
  if(!path || path[0] != '/') return Mode::Invalid;
  const int fd = open(path, O_RDONLY | O_CLOEXEC | O_NOFOLLOW | O_NONBLOCK);
  if(fd < 0) return Mode::Invalid;
  struct stat info{};
  char data[24]{};
  const bool private_file = fstat(fd, &info) == 0 && S_ISREG(info.st_mode) &&
    info.st_uid == geteuid() && (info.st_mode & 0777) == 0600;
  const auto length = private_file ? read(fd, data, sizeof(data)) : -1;
  close(fd);
  if(length <= 0 || length >= static_cast<ssize_t>(sizeof(data))) return Mode::Invalid;
  const std::string_view value(data, static_cast<std::size_t>(length));
  if(value == "pass\n") return Mode::Pass;
  if(value == "drop_rx\n") return Mode::DropRx;
  if(value == "drop_tx\n") return Mode::DropTx;
  if(value == "drop_loiter\n") return Mode::DropLoiter;
  if(value == "hold_point\n") return Mode::HoldPoint;
  if(value == "release_point\n") return Mode::ReleasePoint;
  if(value == "drop_mode_ack\n") return Mode::DropModeAck;
  if(value == "drop_takeoff_ack\n") return Mode::DropTakeoffAck;
  if(value == "drop_rtl_ack\n") return Mode::DropRtlAck;
  if(value == "exit_before_takeoff\n") return Mode::ExitBeforeTakeoff;
  if(value == "hold_hover\n") return Mode::HoldHover;
  if(value == "drop_hover\n") return Mode::DropHover;
  return Mode::Invalid;
}

bool decodeDatagram(const void* data, std::size_t size, mavlink_message_t& decoded)
{
  if(!data || size < MAVLINK_NUM_NON_PAYLOAD_BYTES || size > MAVLINK_MAX_PACKET_LEN) return false;
  mavlink_message_t buffer{};
  mavlink_status_t status{}, output{};
  const auto* bytes = static_cast<const uint8_t*>(data);
  for(std::size_t i = 0; i < size; ++i) {
    const auto framing = mavlink_frame_char_buffer(&buffer, &status, bytes[i], &decoded, &output);
    if(framing != (i + 1 == size ? MAVLINK_FRAMING_OK : MAVLINK_FRAMING_INCOMPLETE)) return false;
  }
  return true;
}

bool addressedAck(const void* data, std::size_t size, unsigned command)
{
  mavlink_message_t decoded{};
  if(!decodeDatagram(data, size, decoded) || decoded.sysid != 1 || decoded.compid != 1 ||
     decoded.msgid != MAVLINK_MSG_ID_COMMAND_ACK) return false;
  mavlink_command_ack_t ack{};
  mavlink_msg_command_ack_decode(&decoded, &ack);
  return ack.command == command && ack.target_system == 245 && ack.target_component == 191;
}

bool hoverPacket(const void* data, std::size_t size, bool from_controller)
{
  mavlink_message_t decoded{};
  if(!decodeDatagram(data, size, decoded) || decoded.msgid != MAVLINK_MSG_ID_TUNNEL ||
     decoded.sysid != (from_controller ? 1 : 245) || decoded.compid != (from_controller ? 1 : 191))
    return false;
  mavlink_tunnel_t packet{};
  mavlink_msg_tunnel_decode(&decoded, &packet);
  return packet.target_system == (from_controller ? 245 : 1) &&
    packet.target_component == (from_controller ? 191 : 1) &&
    packet.payload_type == 0 && packet.payload_length == (from_controller ? 10 : 24) &&
    std::memcmp(packet.payload, "MQH1", 4) == 0;
}

Command command(const void* data, std::size_t size)
{
  mavlink_message_t decoded{};
  if(!decodeDatagram(data, size, decoded) || decoded.sysid != 245 || decoded.compid != 191) return Command::Other;
  if(decoded.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
    mavlink_command_long_t request{};
    mavlink_msg_command_long_decode(&decoded, &request);
    if(request.target_system == 1 && request.target_component == 1 && request.command == MAV_CMD_NAV_TAKEOFF)
      return Command::Takeoff;
    if(request.target_system == 1 && request.target_component == 1 && request.command == MAV_CMD_DO_SET_MODE &&
       request.param1 == MAV_MODE_FLAG_CUSTOM_MODE_ENABLED && request.param2 == 12)
      return Command::Loiter;
  }
  if(decoded.msgid == MAVLINK_MSG_ID_COMMAND_INT) {
    mavlink_command_int_t request{};
    mavlink_msg_command_int_decode(&decoded, &request);
    if(request.target_system == 1 && request.target_component == 1 && request.command == MAV_CMD_DO_REPOSITION &&
       request.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT_INT && request.param2 == 0)
      return Command::Point;
  }
  return Command::Other;
}

using Send = ssize_t (*)(int, const void*, size_t, int, const sockaddr*, socklen_t);
using Receive = ssize_t (*)(int, msghdr*, int);

class Tap {
public:
  ~Tap() { if(m_output >= 0) close(m_output); }

  ssize_t send(Send actual, int fd, const void* data, size_t size, int flags,
               const sockaddr* remote, socklen_t length)
  {
    if(!matches(fd, remote, length)) return actual(fd, data, size, flags, remote, length);
    std::lock_guard<std::mutex> hold(m_lock);
    const Mode mode = currentMode();
    if(mode == Mode::Invalid || !room(size)) { errno = EIO; return -1; }
    const Command kind = command(data, size);
    if(mode == Mode::ExitBeforeTakeoff && kind == Command::Takeoff) {
      if(!record('h',data,size)) { errno = EIO; return -1; }
      _exit(88);
    }
    if(mode == Mode::DropTx || (mode == Mode::DropLoiter && kind == Command::Loiter))
      return discarded(mode == Mode::DropTx ? 'o' : 'l', data, size);
    if((mode == Mode::HoldPoint && kind == Command::Point) ||
       (mode == Mode::HoldHover && hoverPacket(data, size, false))) {
      if(m_held_size != 0) return discarded('q', data, size);
      if(!record('h', data, size)) { errno = EIO; return -1; }
      std::memcpy(m_held.data(), data, size);
      m_held_size = size;
      m_held_fd = fd;
      m_held_at = m_last_stamp;
      return static_cast<ssize_t>(size);
    }
    // A later actual app send supplies a newly checked socket and destination.
    // Deliver the single preserved packet before forwarding that call.
    if(mode == Mode::ReleasePoint && m_held_size != 0) {
      const auto now = timestamp();
      if(fd != m_held_fd || flags != 0 || !now || now < m_held_at || now - m_held_at >= hold_limit_ns ||
         !room(size + m_held_size + header_size, false)) { poison(); errno = EIO; return -1; }
      const auto released = actual(fd, m_held.data(), m_held_size, 0, remote, length);
      const bool delivered = released == static_cast<ssize_t>(m_held_size);
      const bool recorded = record(delivered ? 'd' : 'e', m_held.data(), m_held_size);
      m_held_size = 0;
      if(!delivered || !recorded) { errno = EIO; return -1; }
    }
    const auto result = actual(fd, data, size, flags, remote, length);
    const int saved = errno;
    if(!record(result == static_cast<ssize_t>(size) ? 'T' : 'e', data, size)) { errno = EIO; return -1; }
    errno = saved;
    return result;
  }

  ssize_t receive(Receive actual, int fd, msghdr* message, int flags)
  {
    const auto result = actual(fd, message, flags);
    const int saved = errno;
    if(result >= 0 && matches(fd, static_cast<const sockaddr*>(message->msg_name), message->msg_namelen)) {
      std::lock_guard<std::mutex> hold(m_lock);
      const Mode mode = currentMode();
      if(flags != 0 || message->msg_iovlen != 1 || (message->msg_flags & MSG_TRUNC) ||
         static_cast<std::size_t>(result) > message->msg_iov[0].iov_len) {
        poison();
        message->msg_flags |= MSG_TRUNC;
      } else {
        const auto* data = message->msg_iov[0].iov_base;
        const bool drop = mode == Mode::DropRx ||
          (mode == Mode::DropHover && hoverPacket(data, result, true)) ||
          (mode == Mode::DropModeAck && addressedAck(data, result, MAV_CMD_DO_SET_MODE)) ||
          (mode == Mode::DropTakeoffAck && addressedAck(data, result, MAV_CMD_NAV_TAKEOFF)) ||
          (mode == Mode::DropRtlAck && addressedAck(data, result, MAV_CMD_NAV_RETURN_TO_LAUNCH));
        // Consume exactly one kernel datagram. Do not invent EAGAIN and falsely
        // certify an empty socket queue before the receiver really drains it.
        if(mode == Mode::Invalid || !record(drop ? 'i' : 'R', data, result) || drop) message->msg_flags |= MSG_TRUNC;
      }
    }
    errno = saved;
    return result;
  }

private:
  bool matches(int fd, const sockaddr* remote, socklen_t length) const
  {
    if(!m_app || !m_peer || m_app == m_peer || !loopbackPort(remote, length, m_peer)) return false;
    sockaddr_storage local{};
    socklen_t local_length = sizeof(local);
    int type = 0;
    socklen_t type_length = sizeof(type);
    return getsockname(fd, reinterpret_cast<sockaddr*>(&local), &local_length) == 0 &&
      loopbackPort(reinterpret_cast<sockaddr*>(&local), local_length, m_app) &&
      getsockopt(fd, SOL_SOCKET, SO_TYPE, &type, &type_length) == 0 && type == SOCK_DGRAM;
  }

  Mode currentMode()
  {
    if(m_failed) return Mode::Invalid;
    if(m_output < 0 && m_trace && m_trace[0] == '/')
      m_output = open(m_trace, O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, 0600);
    const Mode mode = readMode(m_control);
    if(m_output < 0 || mode == Mode::Invalid) { poison(); return Mode::Invalid; }
    return mode;
  }

  void poison()
  {
    if(!m_failed && m_output >= 0) {
      const uint8_t invalid = 0;
      (void)write(m_output, &invalid, 1);
    }
    m_failed = true;
  }

  bool room(std::size_t size, bool single = true)
  {
    if((single && size > datagram_limit) || size > trace_limit - header_size ||
       m_written > trace_limit - header_size - size) { poison(); return false; }
    return true;
  }

  ssize_t discarded(char outcome, const void* data, std::size_t size)
  {
    // Simulate a datagram lost after local acceptance, including a lost Loiter.
    if(!record(outcome, data, size)) { errno = EIO; return -1; }
    return static_cast<ssize_t>(size);
  }

  bool record(char outcome, const void* data, std::size_t size)
  {
    if(m_failed || !room(size)) return false;
    const auto now = timestamp();
    if(!now || now < m_last_stamp) { poison(); return false; }
    m_last_stamp = now;
    std::array<uint8_t, header_size> header{'P', 'L', 'F', '1', static_cast<uint8_t>(outcome)};
    const bool incoming = outcome == 'R' || outcome == 'i';
    encode(header.data() + 5, size, 2);
    encode(header.data() + 7, incoming ? m_peer : m_app, 2);
    encode(header.data() + 9, incoming ? m_app : m_peer, 2);
    encode(header.data() + 11, now, 8);
    const iovec parts[]{{header.data(), header.size()}, {const_cast<void*>(data), size}};
    const auto total = header.size() + size;
    if(writev(m_output, parts, 2) != static_cast<ssize_t>(total)) { poison(); return false; }
    m_written += total;
    return true;
  }

  const uint16_t m_app = port("MAVMOOS_PLANE_TAP_APP");
  const uint16_t m_peer = port("MAVMOOS_PLANE_TAP_PEER");
  const char* m_trace = std::getenv("MAVMOOS_PLANE_TAP_PATH");
  const char* m_control = std::getenv("MAVMOOS_PLANE_TAP_CONTROL");
  std::mutex m_lock;
  std::array<uint8_t, datagram_limit> m_held{};
  std::size_t m_held_size = 0, m_written = 0;
  uint64_t m_held_at = 0, m_last_stamp = 0;
  int m_output = -1, m_held_fd = -1;
  bool m_failed = false;
};

Tap& tap()
{
  static Tap value;
  return value;
}
} // namespace

#ifdef __APPLE__
ssize_t tappedSend(int fd, const void* data, size_t size, int flags, const sockaddr* address, socklen_t length)
{
  return tap().send(sendto, fd, data, size, flags, address, length);
}

ssize_t tappedReceive(int fd, msghdr* message, int flags)
{
  return tap().receive(recvmsg, fd, message, flags);
}

struct Interpose { const void* replacement; const void* original; };
__attribute__((used, section("__DATA,__interpose")))
const Interpose replacements[]{
  {reinterpret_cast<const void*>(&tappedSend), reinterpret_cast<const void*>(&sendto)},
  {reinterpret_cast<const void*>(&tappedReceive), reinterpret_cast<const void*>(&recvmsg)}
};
#else
extern "C" ssize_t sendto(int fd, const void* data, size_t size, int flags,
                          const sockaddr* address, socklen_t length)
{
  static const auto actual = reinterpret_cast<Send>(dlsym(RTLD_NEXT, "sendto"));
  if(!actual) { errno = ENOSYS; return -1; }
  return tap().send(actual, fd, data, size, flags, address, length);
}

extern "C" ssize_t recvmsg(int fd, msghdr* message, int flags)
{
  static const auto actual = reinterpret_cast<Receive>(dlsym(RTLD_NEXT, "recvmsg"));
  if(!actual) { errno = ENOSYS; return -1; }
  return tap().receive(actual, fd, message, flags);
}
#endif
