#define _POSIX_C_SOURCE 200809L
#include <arpa/inet.h>
#include <errno.h>
#include <inttypes.h>
#include <netinet/in.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

enum { GCS_PORT = 14550, ONBOARD_PORT = 14540, MAX_BYTES = 4194304, MAX_PACKETS = 12000 };
#ifdef PX4_GCS_ONLY
static const int local_ports[] = {GCS_PORT};
static const int sender_ports[] = {18570};
#else
static const int local_ports[] = {GCS_PORT, ONBOARD_PORT};
static const int sender_ports[] = {18570, 14580};
#endif
enum { LINK_COUNT = sizeof(local_ports) / sizeof(local_ports[0]) };

static void fail(const char* message)
{
  fprintf(stderr, "%s\n", message);
  exit(1);
}

static uint64_t nanos(void)
{
  struct timespec now;
  if(clock_gettime(CLOCK_MONOTONIC, &now)) fail("monotonic clock unavailable");
  return (uint64_t)now.tv_sec * UINT64_C(1000000000) + (uint64_t)now.tv_nsec;
}

static int listen_local(unsigned port)
{
  int fd = socket(AF_INET, SOCK_DGRAM, 0);
  struct sockaddr_in local = {
    .sin_family = AF_INET,
    .sin_port = htons(port),
    .sin_addr = {htonl(INADDR_LOOPBACK)}
  };
  if(fd < 0 || bind(fd, (const struct sockaddr*)&local, sizeof(local))) fail("cannot bind capture port");
  return fd;
}

int main(void)
{
  FILE* raw = fopen("px4-wire.mavlink", "wbx");
  FILE* index = fopen("px4-datagrams.tsv", "wx");
  if(!raw || !index) fail("cannot create exclusive evidence files");
  struct pollfd peers[LINK_COUNT];
  for(size_t i = 0; i < LINK_COUNT; ++i)
    peers[i] = (struct pollfd){.fd = listen_local(local_ports[i]), .events = POLLIN};
  FILE* ready = fopen("capture-ready", "wx");
  if(!ready || fclose(ready)) fail("cannot record readiness");
  const uint64_t start = nanos();
  const uint64_t duration = UINT64_C(16000000000);
  size_t offset = 0, packets = 0;
  while(nanos() - start < duration) {
    int count = poll(peers, LINK_COUNT, 100);
    if(count < 0 && errno == EINTR) continue;
    if(count < 0) fail("capture poll failed");
    for(size_t i = 0; i < LINK_COUNT; ++i) {
      if(peers[i].revents & (POLLERR | POLLHUP | POLLNVAL)) fail("capture socket fault");
      if(!(peers[i].revents & POLLIN)) continue;
      unsigned char data[65535];
      struct sockaddr_in sender = {0};
      socklen_t size = sizeof(sender);
      ssize_t length = recvfrom(peers[i].fd, data, sizeof(data), 0, (struct sockaddr*)&sender, &size);
      if(length <= 0 || sender.sin_family != AF_INET || sender.sin_addr.s_addr != htonl(INADDR_LOOPBACK) ||
         ntohs(sender.sin_port) != sender_ports[i]) fail("unexpected PX4 datagram/source");
      const uint64_t at = nanos() - start;
      if(++packets > MAX_PACKETS || offset + (size_t)length > MAX_BYTES)
        fail("capture evidence bound exceeded");
      if(fwrite(data, 1, (size_t)length, raw) != (size_t)length ||
         fprintf(index, "%" PRIu64 "\t%d\t%d\t%zu\t%zd\n", at, local_ports[i],
                 sender_ports[i], offset, length) < 0) fail("evidence write failed");
      offset += (size_t)length;
    }
  }
  for(size_t i = 0; i < LINK_COUNT; ++i) close(peers[i].fd);
  if(fclose(raw) || fclose(index) || !packets) fail("capture incomplete");
  return 0;
}
