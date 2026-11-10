#include "OwnedFtpRoot.h"
#include "FtpTransaction.h"
#include <cstdlib>
#include <fcntl.h>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <sys/stat.h>
#include <unistd.h>
#include <utility>

namespace {

namespace fs = std::filesystem;
using mavmoos::OwnedFtpRoot;

void check(bool okay, const char* reason)
{
  if(!okay) throw std::runtime_error(reason);
}

struct Fixture {
  const fs::path original = fs::current_path();
  fs::path base, cwd, remote;

  Fixture()
  {
    auto name = (fs::canonical(fs::temp_directory_path()) / "mavmoos-ftp-root-XXXXXX").string();
    check(mkdtemp(name.data()) != nullptr, "create private test root");
    base = name;
    cwd = base / "sitl";
    remote = cwd / "mavmoos-ftp";
    check(mkdir(cwd.c_str(), 0700) == 0 && chdir(cwd.c_str()) == 0, "enter private fixture cwd");
  }

  ~Fixture()
  {
    chdir(original.c_str());
    std::error_code error;
    fs::remove_all(base, error);
  }
};

void file(const fs::path& path, mode_t mode = 0600)
{
  const auto fd = open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, mode);
  check(fd >= 0 && close(fd) == 0, "create regular private fixture file");
}

void checkSearchOnlyAncestor(const Fixture& fixture)
{
  const auto ancestor = fixture.base / "search-only";
  const auto child = ancestor / "sitl";
  const auto root = child / "mavmoos-ftp";
  check(mkdir(ancestor.c_str(), 0700) == 0 && mkdir(child.c_str(), 0700) == 0 &&
        mkdir(root.c_str(), 0700) == 0, "create search-only ancestor fixture");
  file(root / "sample.bin");
  check(chmod(ancestor.c_str(), 0100) == 0, "make ancestor traversable only");
  const bool entered = chdir(child.c_str()) == 0;
  const auto proof = entered ? OwnedFtpRoot::capture(static_cast<int>(getpid())) : std::nullopt;
  const bool allowed = proof && proof->allowsList() && proof->allowsOpen("mavmoos-ftp/sample.bin");
  const bool restored_mode = chmod(ancestor.c_str(), 0700) == 0;
  const bool restored_cwd = chdir(fixture.cwd.c_str()) == 0;
  check(allowed && restored_mode && restored_cwd, "protected ancestor prevented owned descendant verification");
}

void run()
{
  Fixture fixture;
  checkSearchOnlyAncestor(fixture);
  std::string error;
  const auto pid = static_cast<int>(getpid());
  check(!OwnedFtpRoot::capture(-1, &error) && !error.empty(), "invalid PID admitted");
  check(!OwnedFtpRoot::capture(pid, &error) && error.find("mavmoos-ftp") != std::string::npos,
        "missing directory did not fail at startup");
  check(symlink(fixture.base.c_str(), fixture.remote.c_str()) == 0, "make symlink directory");
  check(!OwnedFtpRoot::capture(pid, &error), "symlink remote root admitted");
  check(unlink(fixture.remote.c_str()) == 0 && mkdir(fixture.remote.c_str(), 0700) == 0, "make real root");
  file(fixture.remote / "sample.bin");
  check(chmod(fixture.cwd.c_str(), 0755) == 0, "expose cwd");
  check(!OwnedFtpRoot::capture(pid, &error), "non-private process cwd admitted");
  check(chmod(fixture.cwd.c_str(), 0700) == 0, "restore private cwd");
  check(chmod(fixture.remote.c_str(), 0755) == 0, "expose root");
  check(!OwnedFtpRoot::capture(pid, &error), "non-private FTP root admitted");
  check(chmod(fixture.remote.c_str(), 0700) == 0, "restore private root");

  auto proof = OwnedFtpRoot::capture(pid, &error);
  check(proof && error.empty() && proof->stillBound() && proof->allowsList() && proof->allowsList() &&
        proof->allowsOpen("mavmoos-ftp/sample.bin"), "owned named cwd or repeat listing failed");
  for(const auto* name : {"mavmoos-ftp", "mavmoos-ftp/", "/mavmoos-ftp/sample.bin",
                         "mavmoos-ftp/../sample.bin", "mavmoos-ftp/sub/sample.bin",
                         "mavmoos-ftp2/sample.bin", "mavmoos-ftp/absent"}) {
    check(!proof->allowsOpen(name), "unbounded path or absent basename admitted");
  }
  auto nul = std::string("mavmoos-ftp/sample.bin") + '\0';
  check(!proof->allowsOpen(nul), "NUL alias admitted");

  const auto outside = fixture.base / "outside.bin";
  const auto alias = fixture.remote / "alias.bin";
  file(outside);
  check(symlink(outside.c_str(), alias.c_str()) == 0, "make symlink leaf");
  check(proof->stillBound() && !proof->allowsList() && !proof->allowsOpen("mavmoos-ftp/alias.bin"),
        "symlink leaf allowed in list or open");
  check(!OwnedFtpRoot::capture(pid, &error) && error.find("untrusted") != std::string::npos,
        "untrusted remote listing at startup accepted");
  check(unlink(alias.c_str()) == 0 && link(outside.c_str(), alias.c_str()) == 0, "make hardlink leaf");
  check(!proof->allowsList() && !proof->allowsOpen("mavmoos-ftp/alias.bin"),
        "multiple-link leaf allowed in list or open");
  check(unlink(alias.c_str()) == 0 && mkfifo(alias.c_str(), 0600) == 0, "make FIFO leaf");
  check(!proof->allowsList() && !proof->allowsOpen("mavmoos-ftp/alias.bin"), "FIFO allowed in list or open");
  check(unlink(alias.c_str()) == 0 && mkdir(alias.c_str(), 0700) == 0, "make ordinary child directory");
  check(proof->allowsList() && !proof->allowsOpen("mavmoos-ftp/alias.bin"), "child directory open allowed");
  check(rmdir(alias.c_str()) == 0, "remove child directory");
  const auto sample = fixture.remote / "sample.bin";
  check(chmod(sample.c_str(), 0640) == 0, "expose regular leaf");
  check(!proof->allowsList() && !proof->allowsOpen("mavmoos-ftp/sample.bin"), "non-private leaf admitted");
  check(chmod(sample.c_str(), 0600) == 0 && truncate(sample.c_str(), 65537) == 0, "make oversized sample");
  check(proof->allowsList() && !proof->allowsOpen("mavmoos-ftp/sample.bin"), "oversized leaf admitted for open");
  check(truncate(sample.c_str(), 65536) == 0 && proof->allowsOpen("mavmoos-ftp/sample.bin"), "maximum leaf denied");

  const auto alternate = fixture.base / "different-sitl";
  check(mkdir(alternate.c_str(), 0700) == 0 && chdir(alternate.c_str()) == 0, "change live PID cwd");
  check(!proof->stillBound() && !proof->allowsList() && !proof->allowsOpen("mavmoos-ftp/sample.bin"),
        "captured proof survived changed process cwd");
  check(chdir(fixture.cwd.c_str()) == 0, "restore PID cwd");
  const auto old_remote = fixture.cwd / "former-root";
  check(rename(fixture.remote.c_str(), old_remote.c_str()) == 0 && mkdir(fixture.remote.c_str(), 0700) == 0,
        "replace named remote directory");
  check(!proof->stillBound() && !proof->allowsList() && !proof->allowsOpen("mavmoos-ftp/sample.bin"),
        "captured proof survived replacement FTP root");
  check(rmdir(fixture.remote.c_str()) == 0 && rename(old_remote.c_str(), fixture.remote.c_str()) == 0,
        "restore original root inode");
  check(proof->allowsList(), "restored exact root failed check");

  // The regular sample plus two POSIX dot entries use three of the slots.
  for(std::size_t index = 3; index < mavmoos::ftp::max_list_offset; ++index)
    file(fixture.remote / ("small-" + std::to_string(index)));
  check(proof->allowsList(), "exact FTP directory entry budget denied");
  file(fixture.remote / "overflow");
  check(!proof->allowsList(), "directory above bounded FTP listing budget admitted");
  check(unlink((fixture.remote / "overflow").c_str()) == 0, "remove excess test file");

  auto moved = std::move(*proof);
  check(!proof->stillBound() && moved.allowsList(), "moving proof did not transfer descriptors");
}

} // namespace

int main()
{
  try {
    run();
    std::cout << "MAVLink FTP owned PID cwd, private root, bounded files and replacement rejection passed\n";
  } catch(const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
