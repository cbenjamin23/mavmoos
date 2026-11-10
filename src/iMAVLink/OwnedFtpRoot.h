#pragma once

#include <optional>
#include <string>
#include <string_view>

namespace mavmoos {

// Binds the relative FTP directory to the configured native process's actual
// working directory. The caller separately proves the exact SITL UDP endpoint.
// The private fixture owns the tree and keeps it stable across server syscalls.
class OwnedFtpRoot {
public:
  static std::optional<OwnedFtpRoot> capture(int pid, std::string* error = nullptr);

  OwnedFtpRoot(OwnedFtpRoot&&) noexcept;
  OwnedFtpRoot& operator=(OwnedFtpRoot&&) noexcept;
  ~OwnedFtpRoot();

  OwnedFtpRoot(const OwnedFtpRoot&) = delete;
  OwnedFtpRoot& operator=(const OwnedFtpRoot&) = delete;

  bool stillBound() const;
  bool allowsList() const;
  bool allowsOpen(std::string_view full_relative_path) const;

private:
  OwnedFtpRoot(int pid, int cwd, int root) : m_pid(pid), m_cwd(cwd), m_root(root) {}

  int m_pid;
  int m_cwd;
  int m_root;
};

} // namespace mavmoos
