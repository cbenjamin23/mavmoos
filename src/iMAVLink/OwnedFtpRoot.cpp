#include "OwnedFtpRoot.h"
#include "../libmavmoos_ftp/FtpTransaction.h"
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <memory>
#include <sys/stat.h>
#include <unistd.h>
#include <utility>
#ifdef __APPLE__
#include <libproc.h>
#include <sys/proc_info.h>
#endif

namespace mavmoos {
namespace {

constexpr int directory_flags = O_RDONLY | O_DIRECTORY | O_CLOEXEC;
constexpr int named_directory_flags = directory_flags | O_NOFOLLOW;

void closeFd(int fd)
{
  if(fd >= 0) close(fd);
}

class Descriptor {
public:
  explicit Descriptor(int fd) : m_fd(fd) {}
  ~Descriptor() { closeFd(m_fd); }
  Descriptor(const Descriptor&) = delete;
  Descriptor& operator=(const Descriptor&) = delete;

  int get() const { return m_fd; }
  int release() { return std::exchange(m_fd, -1); }
  void reset(int fd) { closeFd(std::exchange(m_fd, fd)); }

private:
  int m_fd;
};

bool privateOwner(const struct stat& state)
{
  return state.st_uid == geteuid() && !(state.st_mode & (S_IRWXG | S_IRWXO));
}

bool privateDirectory(int fd, struct stat& state)
{
  return fd >= 0 && fstat(fd, &state) == 0 && S_ISDIR(state.st_mode) && privateOwner(state) &&
    (state.st_mode & (S_IRUSR | S_IXUSR)) == (S_IRUSR | S_IXUSR);
}

bool samePrivateDirectory(int captured, int current)
{
  struct stat original{}, named{};
  return privateDirectory(captured, original) && privateDirectory(current, named) &&
    original.st_dev == named.st_dev && original.st_ino == named.st_ino;
}

#ifdef __APPLE__
int openAbsoluteDirectory(std::string_view path)
{
  if(path.size() < 2 || path.front() != '/' || path.back() == '/') return -1;
  // Reading an ancestor (including macOS protected folders) can be denied
  // even when the owned descendant is accessible by path.
  constexpr int traverse = O_SEARCH | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW;
  path.remove_prefix(1);
  Descriptor fd(open("/", traverse));
  while(fd.get() >= 0 && !path.empty()) {
    const auto slash = path.find('/');
    const auto component = path.substr(0, slash);
    if(component.empty() || component == "." || component == "..") return -1;
    const std::string name(component);
    fd.reset(openat(fd.get(), name.c_str(), traverse));
    if(slash == std::string_view::npos) break;
    path.remove_prefix(slash + 1);
  }
  return fd.get() >= 0 ? openat(fd.get(), ".", named_directory_flags) : -1;
}
#endif

int openProcessCwd(int pid)
{
  if(pid <= 0) return -1;
#ifdef __APPLE__
  proc_bsdinfo process{};
  proc_vnodepathinfo paths{};
  if(proc_pidinfo(pid, PROC_PIDTBSDINFO, 0, &process, sizeof(process)) != sizeof(process) ||
     process.pbi_uid != geteuid() ||
     proc_pidinfo(pid, PROC_PIDVNODEPATHINFO, 0, &paths, sizeof(paths)) != sizeof(paths))
    return -1;
  const auto& cwd = paths.pvi_cdir;
  const auto length = strnlen(cwd.vip_path, sizeof(cwd.vip_path));
  if(length == sizeof(cwd.vip_path)) return -1;
  Descriptor fd(openAbsoluteDirectory(std::string_view(cwd.vip_path, length)));
  struct stat current{};
  // The pathname could have changed since the kernel reported the cwd vnode.
  if(!privateDirectory(fd.get(), current) || static_cast<std::uint32_t>(current.st_dev) != cwd.vip_vi.vi_stat.vst_dev ||
     current.st_ino != cwd.vip_vi.vi_stat.vst_ino)
    return -1;
  return fd.release();
#elif defined(__linux__)
  const std::string path = "/proc/" + std::to_string(pid);
  Descriptor process(open(path.c_str(), named_directory_flags));
  struct stat current{};
  if(process.get() < 0 || fstat(process.get(), &current) != 0 || current.st_uid != geteuid())
    return -1;
  // Follow the kernel's procfs cwd link to the actual process directory.
  return openat(process.get(), "cwd", directory_flags);
#else
  return -1;
#endif
}

int openFtpDirectory(int cwd)
{
  return openat(cwd, ftp::directory.data(), named_directory_flags);
}

bool allowedEntry(int root, const char* name, bool list_only)
{
  struct stat entry{};
  if(fstatat(root, name, &entry, AT_SYMLINK_NOFOLLOW) != 0 || !privateOwner(entry)) return false;
  if(list_only && S_ISDIR(entry.st_mode)) return true;
  return S_ISREG(entry.st_mode) && entry.st_nlink == 1 && (entry.st_mode & S_IRUSR) &&
    (list_only || (entry.st_size >= 0 && static_cast<std::uint64_t>(entry.st_size) <= ftp::max_file_size));
}

} // namespace

OwnedFtpRoot::OwnedFtpRoot(OwnedFtpRoot&& other) noexcept
  : m_pid(other.m_pid), m_cwd(std::exchange(other.m_cwd, -1)), m_root(std::exchange(other.m_root, -1))
{
}

OwnedFtpRoot& OwnedFtpRoot::operator=(OwnedFtpRoot&& other) noexcept
{
  if(this != &other) {
    closeFd(m_root);
    closeFd(m_cwd);
    m_pid = other.m_pid;
    m_cwd = std::exchange(other.m_cwd, -1);
    m_root = std::exchange(other.m_root, -1);
  }
  return *this;
}

OwnedFtpRoot::~OwnedFtpRoot()
{
  closeFd(m_root);
  closeFd(m_cwd);
}

std::optional<OwnedFtpRoot> OwnedFtpRoot::capture(int pid, std::string* error)
{
  if(error) error->clear();
  const auto reject = [error](const char* reason) -> std::optional<OwnedFtpRoot> {
    if(error) *error = reason;
    return std::nullopt;
  };

  Descriptor cwd(openProcessCwd(pid));
  struct stat cwd_state{}, root_state{};
  if(!privateDirectory(cwd.get(), cwd_state))
    return reject("MAVLink FTP requires the live SITL process to have an owner-private working directory");
  Descriptor root(openFtpDirectory(cwd.get()));
  if(!privateDirectory(root.get(), root_state))
    return reject("MAVLink FTP requires a real owner-private mavmoos-ftp directory inside the SITL working directory");
  OwnedFtpRoot proof(pid, cwd.release(), root.release());
  if(!proof.allowsList())
    return reject("MAVLink FTP remote directory changed or contains untrusted entries");
  return proof;
}

bool OwnedFtpRoot::stillBound() const
{
  if(m_cwd < 0 || m_root < 0) return false;
  Descriptor cwd(openProcessCwd(m_pid));
  if(!samePrivateDirectory(m_cwd, cwd.get())) return false;
  Descriptor root(openFtpDirectory(cwd.get()));
  return samePrivateDirectory(m_root, root.get());
}

bool OwnedFtpRoot::allowsList() const
{
  if(!stillBound()) return false;
  // Open a new file description so one scan never advances the captured FD.
  Descriptor scan(openat(m_root, ".", named_directory_flags));
  if(scan.get() < 0) return false;
  const std::unique_ptr<DIR, decltype(&closedir)> stream(fdopendir(scan.get()), closedir);
  if(!stream) return false;
  scan.release();
  std::size_t count = 0;
  for(;;) {
    errno = 0;
    const auto* entry = readdir(stream.get());
    if(!entry) return errno == 0 && stillBound();
    // ArduPilot counts the POSIX dot entries in the same wire listing budget.
    if(++count > ftp::max_list_offset) return false;
    const std::string_view name(entry->d_name);
    if(name != "." && name != ".." && !allowedEntry(m_root, entry->d_name, true))
      return false;
  }
}

bool OwnedFtpRoot::allowsOpen(std::string_view full_relative_path) const
{
  const auto prefix_length = ftp::directory.size();
  if(full_relative_path.size() <= prefix_length || full_relative_path.substr(0, prefix_length) != ftp::directory ||
     full_relative_path[prefix_length] != '/')
    return false;
  const auto basename = full_relative_path.substr(prefix_length + 1);
  if(!ftp::safeBasename(basename) || !stillBound()) return false;
  const std::string name(basename);
  return allowedEntry(m_root, name.c_str(), false);
}

} // namespace mavmoos
