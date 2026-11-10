#include <ardupilotmega/mavlink.h>

#include <arpa/inet.h>
#include <array>
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <sys/socket.h>
#include <unistd.h>

namespace {
sockaddr_in local(unsigned port)
{
  sockaddr_in endpoint{};
  endpoint.sin_family = AF_INET;
  endpoint.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  endpoint.sin_port = htons(port);
  return endpoint;
}

int openSocket(unsigned port)
{
  const int fd = socket(AF_INET, SOCK_DGRAM, 0);
  if(fd < 0) std::exit(2);
  const auto endpoint = local(port);
  timeval timeout{1, 0};
  if(bind(fd, reinterpret_cast<const sockaddr*>(&endpoint), sizeof(endpoint)) != 0 ||
     setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) != 0) std::exit(2);
  return fd;
}

ssize_t transmit(int fd, unsigned port, const std::string& kind)
{
  const auto endpoint = local(port);
  std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> data{};
  std::size_t size = 1;
  if(kind == "plain") data[0] = 'x';
  else {
    mavlink_message_t message{};
    if(kind == "point" || kind == "otherpoint") {
      const int32_t latitude = kind == "point" ? 423585000 : 423586000;
      mavlink_msg_command_int_pack(245, 191, &message, 1, 1, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        MAV_CMD_DO_REPOSITION, 0, 0, -1, 0, 0, 0, latitude, -710875000, 65);
    } else {
      const float mode = kind == "rtl" ? 11 : 12;
      mavlink_msg_command_long_pack(245, 191, &message, 1, 1, MAV_CMD_DO_SET_MODE, 0,
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode, 0, 0, 0, 0, 0);
    }
    size = mavlink_msg_to_send_buffer(data.data(), &message);
    if(kind == "badloiter") data[size - 1] ^= 1;
  }
  return sendto(fd, data.data(), size, 0, reinterpret_cast<const sockaddr*>(&endpoint), sizeof(endpoint));
}

const char* receive(int fd)
{
  sockaddr_in source{};
  std::array<uint8_t, 128> data{};
  iovec bytes{data.data(), data.size()};
  msghdr message{};
  message.msg_name = &source;
  message.msg_namelen = sizeof(source);
  message.msg_iov = &bytes;
  message.msg_iovlen = 1;
  if(recvmsg(fd, &message, 0) < 0) return errno == EAGAIN || errno == EWOULDBLOCK ? "empty" : "error";
  return message.msg_flags & MSG_TRUNC ? "discarded" : "received";
}
} // namespace

int main(int argc, char** argv)
{
  if(argc != 2) return 2;
  const int socket = openSocket(std::stoul(argv[1]));
  const int other = openSocket(0);
  std::cout << "ready" << std::endl;
  std::string action;
  while(std::cin >> action) {
    if(action == "receive") std::cout << receive(socket) << std::endl;
    else if(action == "exit") break;
    else {
      unsigned port = 0;
      std::string kind;
      if((action != "send" && action != "other") || !(std::cin >> port >> kind)) return 2;
      std::cout << (transmit(action == "send" ? socket : other, port, kind) < 0 ? "error" : "accepted") << std::endl;
    }
  }
  close(other);
  close(socket);
}
