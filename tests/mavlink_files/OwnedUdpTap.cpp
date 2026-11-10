// Native fixture only. Preserve the app's direct socket and record completed
// syscalls for one nominated loopback port pair in a fresh private file.
#include <arpa/inet.h>
#include <array>
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <dlfcn.h>
#include <fcntl.h>
#include <mutex>
#include <sys/socket.h>
#include <sys/uio.h>
#include <time.h>
#include <unistd.h>

namespace {
constexpr std::size_t limit = 4 * 1024 * 1024;
constexpr std::size_t datagram_limit = 4096;

uint16_t port(const char* key)
{
  const char* text = std::getenv(key);
  if(!text || !*text) return 0;
  char* end = nullptr;
  const auto value = std::strtoul(text, &end, 10);
  return *end == '\0' && value > 0 && value <= 65535 ? static_cast<uint16_t>(value) : 0;
}

bool loopbackPort(const sockaddr* address, socklen_t length, uint16_t expected)
{
  if(!address || length < sizeof(sockaddr_in) || address->sa_family != AF_INET) return false;
  const auto* ipv4 = reinterpret_cast<const sockaddr_in*>(address);
  return ipv4->sin_addr.s_addr == htonl(INADDR_LOOPBACK) && ntohs(ipv4->sin_port) == expected;
}

void encode(uint8_t* out, uint64_t value, unsigned length)
{
  for(unsigned i = 0; i < length; ++i) {
    out[i] = static_cast<uint8_t>(value);
    value >>= 8;
  }
}

struct Tap {
  const uint16_t app = port("MAVMOOS_FTP_TAP_APP");
  const uint16_t peer = port("MAVMOOS_FTP_TAP_PEER");
  const char* path = std::getenv("MAVMOOS_FTP_TAP_PATH");
  std::mutex lock;
  std::size_t written = 0;
  int output = -1;
  bool failed = false;

  ~Tap() { if(output >= 0) close(output); }

  void record(char direction, int fd, const void* data, ssize_t size, const sockaddr* remote, socklen_t length)
  {
    if(!path || !app || !peer || app == peer || size <= 0 || !loopbackPort(remote, length, peer)) return;
    sockaddr_storage local{};
    socklen_t local_length = sizeof(local);
    int type = 0;
    socklen_t type_length = sizeof(type);
    if(getsockname(fd, reinterpret_cast<sockaddr*>(&local), &local_length) != 0 ||
       !loopbackPort(reinterpret_cast<sockaddr*>(&local), local_length, app) ||
       getsockopt(fd, SOL_SOCKET, SO_TYPE, &type, &type_length) != 0 || type != SOCK_DGRAM) return;

    std::lock_guard<std::mutex> hold(lock);
    if(failed) return;
    if(output < 0) output = open(path, O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, 0600);
    if(output < 0) {
      failed = true;
      return;
    }
    std::array<uint8_t, 19> header{'F', 'T', 'P', '1', static_cast<uint8_t>(direction)};
    timespec stamp{};
    if(static_cast<std::size_t>(size) > datagram_limit || written + header.size() + size > limit ||
       clock_gettime(CLOCK_MONOTONIC, &stamp) != 0) {
      // Leave an invalid final byte so a truncated proof cannot pass.
      const uint8_t bad = 0;
      (void)write(output, &bad, 1);
      failed = true;
      return;
    }
    encode(header.data() + 5, static_cast<uint64_t>(size), 2);
    encode(header.data() + 7, direction == 'T' ? app : peer, 2);
    encode(header.data() + 9, direction == 'T' ? peer : app, 2);
    encode(header.data() + 11, static_cast<uint64_t>(stamp.tv_sec) * 1000000000 + stamp.tv_nsec, 8);
    const iovec parts[]{{header.data(), header.size()}, {const_cast<void*>(data), static_cast<std::size_t>(size)}};
    const auto total = static_cast<ssize_t>(header.size()) + size;
    failed = writev(output, parts, 2) != total;
    if(failed) {
      const uint8_t bad = 0;
      (void)write(output, &bad, 1);
    }
    written += static_cast<std::size_t>(total);
  }
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
  const auto result = sendto(fd, data, size, flags, address, length);
#else
extern "C" ssize_t sendto(int fd, const void* data, size_t size, int flags,
                         const sockaddr* address, socklen_t length)
{
  using Send = ssize_t (*)(int, const void*, size_t, int, const sockaddr*, socklen_t);
  static const auto actual = reinterpret_cast<Send>(dlsym(RTLD_NEXT, "sendto"));
  if(!actual) { errno = ENOSYS; return -1; }
  const auto result = actual(fd, data, size, flags, address, length);
#endif
  const int saved = errno;
  if(result >= 0) tap().record('T', fd, data, result, address, length);
  errno = saved;
  return result;
}

#ifdef __APPLE__
ssize_t tappedReceive(int fd, msghdr* message, int flags)
{
  const auto result = recvmsg(fd, message, flags);
#else
extern "C" ssize_t recvmsg(int fd, msghdr* message, int flags)
{
  using Receive = ssize_t (*)(int, msghdr*, int);
  static const auto actual = reinterpret_cast<Receive>(dlsym(RTLD_NEXT, "recvmsg"));
  if(!actual) { errno = ENOSYS; return -1; }
  const auto result = actual(fd, message, flags);
#endif
  const int saved = errno;
  if(result > 0 && message->msg_iovlen == 1 && !(message->msg_flags & MSG_TRUNC) &&
     static_cast<std::size_t>(result) <= message->msg_iov[0].iov_len)
    tap().record('R', fd, message->msg_iov[0].iov_base, result,
                 static_cast<sockaddr*>(message->msg_name), message->msg_namelen);
  errno = saved;
  return result;
}

#ifdef __APPLE__
struct Interpose { const void* replacement; const void* original; };
__attribute__((used, section("__DATA,__interpose")))
const Interpose replacements[]{
  {reinterpret_cast<const void*>(&tappedSend), reinterpret_cast<const void*>(&sendto)},
  {reinterpret_cast<const void*>(&tappedReceive), reinterpret_cast<const void*>(&recvmsg)}
};
#endif
