/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: UdpReceiver.cpp                                 */
/*    DATE: October 29, 2026                                */
/************************************************************/
#include "UdpReceiver.h"
#include "Telemetry.h"
#include "SimulationSendGate.h"
#include <algorithm>
#include <arpa/inet.h>
#include <cerrno>
#include <cmath>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string>
#include <vector>
#ifdef __APPLE__
#include <libproc.h>
#include <sys/proc_info.h>
#elif defined(__linux__)
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <ifaddrs.h>
#include <net/if.h>
#include <set>
#include <sstream>
#include <sys/stat.h>
#endif

namespace mavmoos {
#if defined(__linux__)
namespace {
bool isolatedPx4Network(const std::string& proc)
{
  struct stat local{},remote{};
  if(stat("/proc/self/ns/net",&local)!=0 || stat((proc+"/ns/net").c_str(),&remote)!=0 ||
     local.st_dev!=remote.st_dev || local.st_ino!=remote.st_ino) return false;
  struct ifaddrs* interfaces=nullptr;
  if(getifaddrs(&interfaces)!=0) return false;
  bool ipv4_loopback=false,other_interface=false;
  for(const auto* i=interfaces;i;i=i->ifa_next) {
    if(!(i->ifa_flags&IFF_UP))continue;
    if(!(i->ifa_flags&IFF_LOOPBACK))other_interface=true;
    if(i->ifa_addr && i->ifa_addr->sa_family==AF_INET &&
       (i->ifa_flags&IFF_LOOPBACK) &&
       reinterpret_cast<const sockaddr_in*>(i->ifa_addr)->sin_addr.s_addr==htonl(INADDR_LOOPBACK))
      ipv4_loopback=true;
  }
  freeifaddrs(interfaces);
  std::ifstream routes("/proc/self/net/route");
  std::string header,interface;
  return ipv4_loopback && !other_interface && std::getline(routes,header) &&
    !(routes>>interface) && routes.eof();
}
}
#endif

void QueueEpoch::reset(double before_bind)
{
  m_floor = m_last_read = before_bind;
  m_draining = !std::isfinite(before_bind) || before_bind < 0;
}

bool QueueEpoch::ordered(double before_read, double after_read) const
{
  return std::isfinite(m_floor) && m_floor >= 0 &&
    std::isfinite(before_read) && std::isfinite(after_read) &&
    before_read >= m_floor && before_read >= m_last_read && after_read >= before_read;
}

QueueEpoch::Decision QueueEpoch::received(double before_read, double after_read, double max_age)
{
  if(!ordered(before_read, after_read) || !std::isfinite(max_age) || max_age <= 0) {
    m_draining = true;
    return Decision::ClockInvalid;
  }
  m_last_read = after_read;
  if(m_draining || after_read - m_floor > max_age) {
    m_draining = true;
    return Decision::Stale;
  }
  return Decision::Accept;
}

bool QueueEpoch::empty(double before_read, double after_read)
{
  if(!ordered(before_read, after_read)) {
    m_draining = true;
    return false;
  }
  // A packet can arrive after EAGAIN but before after_read. Only the sample
  // before the syscall is therefore a valid lower socket-admission boundary.
  m_floor = before_read;
  m_last_read = after_read;
  m_draining = false;
  return true;
}

bool UdpReceiver::isSitlProcess(int pid,const std::string& profile)
{
  if(pid<=0 || (profile!="arducopter" && profile!="ardurover" && profile!="arduplane" && profile!="px4" && profile!="ardusub")) return false;
  char path[4096]{};
#ifdef __APPLE__
  if(proc_pidpath(pid, path, sizeof(path)) <= 0) return false;
#elif defined(__linux__)
  const auto link=std::string("/proc/")+std::to_string(pid)+"/exe";
  const auto count=readlink(link.c_str(),path,sizeof(path)-1);
  if(count<=0) return false;
  path[count]=0;
#else
  return false;
#endif
  const std::string executable(path);
  if(profile=="px4") {
#ifdef __linux__
    return executable=="/opt/px4/bin/px4";
#else
    return false;
#endif
  }
  return executable.substr(executable.find_last_of('/')+1)==profile;
}

bool UdpReceiver::isSitlEndpoint(int pid, uint16_t peer_port,const std::string& profile)
{
  if(peer_port==0 || !isSitlProcess(pid,profile)) return false;
#ifdef __APPLE__
  // Inspect the nominated process's descriptors, never a shell command or the
  // system-wide port list. A different localhost gateway is not a SITL endpoint.
  constexpr std::size_t max_fds=4096;
  const int needed=proc_pidinfo(pid,PROC_PIDLISTFDS,0,nullptr,0);
  if(needed<=0 || static_cast<std::size_t>(needed)>max_fds*sizeof(proc_fdinfo)) return false;
  const std::size_t capacity=std::min(max_fds,static_cast<std::size_t>(needed)/sizeof(proc_fdinfo)+32);
  std::vector<proc_fdinfo> descriptors(capacity);
  const int count=proc_pidinfo(pid,PROC_PIDLISTFDS,0,descriptors.data(),static_cast<int>(descriptors.size()*sizeof(proc_fdinfo)));
  if(count<=0 || static_cast<std::size_t>(count)>descriptors.size()*sizeof(proc_fdinfo)) return false;
  for(std::size_t i=0;i<static_cast<std::size_t>(count)/sizeof(proc_fdinfo);++i) {
    if(descriptors[i].proc_fdtype!=PROX_FDTYPE_SOCKET) continue;
    socket_fdinfo socket{};
    if(proc_pidfdinfo(pid,descriptors[i].proc_fd,PROC_PIDFDSOCKETINFO,&socket,sizeof(socket))!=sizeof(socket)) continue;
    if(socket.psi.soi_family!=AF_INET || socket.psi.soi_type!=SOCK_DGRAM || socket.psi.soi_protocol!=IPPROTO_UDP) continue;
    const auto& endpoint=socket.psi.soi_proto.pri_in;
    if(ntohs(static_cast<uint16_t>(endpoint.insi_lport))==peer_port &&
       endpoint.insi_laddr.ina_46.i46a_addr4.s_addr==htonl(INADDR_LOOPBACK)) return true;
  }
  return false;
#elif defined(__linux__)
  try {
    const std::string root="/proc/"+std::to_string(pid);
    // The pinned PX4 onboard socket uses INADDR_ANY. Its only permitted
    // deployment for this feature is the same Linux network namespace with
    // no usable interface except loopback. Never relax the other profiles.
    const bool px4=profile=="px4";
    if(px4 && (peer_port!=14580 || !isolatedPx4Network(root))) return false;
    std::set<std::string> sockets;
    std::size_t count=0;
    for(const auto& entry:std::filesystem::directory_iterator(root+"/fd")) {
      if(++count>4096) return false;
      char link[256]{};
      const auto length=readlink(entry.path().c_str(),link,sizeof(link)-1);
      if(length<=0) continue;
      const std::string target(link,static_cast<std::size_t>(length));
      if(target.rfind("socket:[",0)==0 && target.back()==']') sockets.insert(target.substr(8,target.size()-9));
    }
    std::ifstream udp(root+"/net/udp");
    if(!udp) return false;
    char expected[32]{},wildcard[32]{};
    std::snprintf(expected,sizeof(expected),"%08X:%04X",htonl(INADDR_LOOPBACK),peer_port);
    if(px4)std::snprintf(wildcard,sizeof(wildcard),"00000000:%04X",peer_port);
    std::string line;
    std::getline(udp,line); // header
    count=0;
    while(std::getline(udp,line)) {
      if(++count>16384) return false;
      std::istringstream fields(line); std::string value,local,inode;
      for(unsigned column=0;column<10 && fields>>value;++column) {
        if(column==1) local=value;
        if(column==9) inode=value;
      }
      if((local==expected || (px4 && local==wildcard)) && sockets.count(inode)) return true;
    }
  } catch(const std::exception&) { return false; }
  return false;
#else
  return false;
#endif
}

bool UdpReceiver::sendSimulationFrame(const uint8_t* bytes, std::size_t size, int sitl_pid,
  double expires,const std::function<bool(double)>& eligible)
{
  // Only a live direct SITL executable and loopback route may receive effects.
  if(!isOpen() || m_config.bind_address!="127.0.0.1" ||
     m_config.peer_address!="127.0.0.1" || !bytes || size==0 || size>MAVLINK_MAX_PACKET_LEN)
    return false;
  sockaddr_in peer{};
  peer.sin_family=AF_INET; peer.sin_port=htons(m_config.peer_port);
  peer.sin_addr.s_addr=m_peer_ipv4;
  return detail::attemptSimulationSend(expires,
    [this,sitl_pid] { return isSitlEndpoint(sitl_pid,m_config.peer_port,m_config.simulator_profile); },
    [] { return monotonicSeconds(); },eligible,
    [this,bytes,size,&peer] {
      return sendto(m_socket,bytes,size,0,reinterpret_cast<sockaddr*>(&peer),sizeof(peer))==static_cast<ssize_t>(size);
    }).accepted();
}
UdpReceiver::~UdpReceiver() { close(); }

void UdpReceiver::close()
{
  if(m_socket >= 0)
    ::close(m_socket);
  m_socket = -1;
}

bool UdpReceiver::open(const RxConfig& config)
{
  close();
  m_stats = {};
  m_last_poll_at = -1;
  if(!validateConfig(config).empty()) {
    m_stats.last_errno = EINVAL;
    return false;
  }
  m_config = config;
  in_addr peer{};
  inet_pton(AF_INET, config.peer_address.c_str(), &peer);
  m_peer_ipv4 = peer.s_addr;
  m_socket = ::socket(AF_INET, SOCK_DGRAM, 0);
  if(m_socket < 0) {
    m_stats.last_errno = errno;
    return false;
  }
  const int flags = fcntl(m_socket, F_GETFL, 0);
  sockaddr_in local{};
  local.sin_family = AF_INET;
  local.sin_port = htons(config.bind_port);
  inet_pton(AF_INET, config.bind_address.c_str(), &local.sin_addr);
  int buffer_bytes = 65536;
  const double before_bind = monotonicSeconds();
  if(flags < 0 || fcntl(m_socket, F_SETFL, flags | O_NONBLOCK) < 0 ||
     fcntl(m_socket, F_SETFD, FD_CLOEXEC) < 0 ||
     setsockopt(m_socket, SOL_SOCKET, SO_RCVBUF, &buffer_bytes, sizeof(buffer_bytes)) < 0 ||
     ::bind(m_socket, reinterpret_cast<sockaddr*>(&local), sizeof(local)) < 0) {
    m_stats.last_errno = errno;
    close();
    return false;
  }
  m_epoch.reset(before_bind);
  m_stats.queue_epoch_at = m_epoch.floor();
  m_stats.draining = m_epoch.draining();
  return true;
}

void UdpReceiver::poll(Telemetry& telemetry)
{
  if(!isOpen())
    return;
  ++m_stats.polls;
  const double poll_at = monotonicSeconds();
  if(m_last_poll_at >= 0 && poll_at >= m_last_poll_at) {
    m_stats.last_poll_gap_s = poll_at - m_last_poll_at;
    m_stats.max_poll_gap_s = std::max(m_stats.max_poll_gap_s, m_stats.last_poll_gap_s);
  } else m_stats.last_poll_gap_s = -1;
  m_last_poll_at = poll_at;
  for(unsigned n = 0; n < m_config.max_datagrams_per_iter; ++n) {
    sockaddr_in sender{};
    iovec io{m_buffer.data(), m_buffer.size()};
    msghdr header{};
    header.msg_name = &sender;
    header.msg_namelen = sizeof(sender);
    header.msg_iov = &io;
    header.msg_iovlen = 1;
    const double before_read = monotonicSeconds();
    const auto size = recvmsg(m_socket, &header, 0);
    const int receive_errno = size < 0 ? errno : 0;
    const double read_at = monotonicSeconds();
    if(size < 0) {
      if(receive_errno == EAGAIN || receive_errno == EWOULDBLOCK) {
        if(m_epoch.empty(before_read, read_at)) ++m_stats.empty_events;
        m_stats.queue_epoch_at = m_epoch.floor();
        m_stats.draining = m_epoch.draining();
        return;
      }
      ++m_stats.errors;
      m_stats.last_errno = receive_errno;
      return;
    }
    ++m_stats.datagrams;
    m_stats.bytes += static_cast<uint64_t>(size);
    m_stats.last_read_at = read_at;
    const auto decision = m_epoch.received(before_read, read_at, m_config.max_queue_age);
    const double queue_age = read_at - m_epoch.floor();
    m_stats.last_queue_age_upper_s = decision != QueueEpoch::Decision::ClockInvalid &&
      std::isfinite(queue_age) && queue_age >= 0 ? queue_age : -1;
    if(m_stats.last_queue_age_upper_s >= 0)
      m_stats.max_queue_age_upper_s = std::max(m_stats.max_queue_age_upper_s, queue_age);
    m_stats.draining = m_epoch.draining();
    if(sender.sin_family != AF_INET || sender.sin_addr.s_addr != m_peer_ipv4 ||
       ntohs(sender.sin_port) != m_config.peer_port) {
      ++m_stats.wrong_peer;
    } else if(header.msg_flags & MSG_TRUNC) {
      ++m_stats.truncated;
    } else if(decision == QueueEpoch::Decision::ClockInvalid) {
      ++m_stats.queue_clock_dropped;
    } else if(decision == QueueEpoch::Decision::Stale) {
      ++m_stats.queue_stale_dropped;
    } else {
      m_stats.last_effective_at = m_epoch.floor();
      telemetry.ingest(m_buffer.data(), static_cast<std::size_t>(size),
                       m_epoch.floor(), m_stats.datagrams, read_at, "QUEUE_EMPTY_FLOOR");
    }
    if(n + 1 == m_config.max_datagrams_per_iter)
      ++m_stats.budget_hits;
  }
}
} // namespace mavmoos
