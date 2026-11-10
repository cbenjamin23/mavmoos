#include "ControlReplayJournal.h"
#include <algorithm>
#include <cerrno>
#include <dirent.h>
#include <fcntl.h>
#include <filesystem>
#include <stdexcept>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

namespace mavmoos::runtime {
namespace {
constexpr char route_file[]=".route";
constexpr std::size_t max_id=120;

std::string recordName(const std::string& id)
{
  constexpr char hex[]="0123456789abcdef";
  std::string name;
  for(unsigned char c:id) {name+=hex[c>>4];name+=hex[c&15];}
  return name;
}

bool privateOwner(const struct stat& info)
{
  return info.st_uid==geteuid() && !(info.st_mode&(S_IRWXG|S_IRWXO));
}

bool recordValid(int directory,const std::string& name)
{
  struct stat info{};
  return name.size()>=2 && name.size()<=2*max_id && name.size()%2==0 &&
    std::all_of(name.begin(),name.end(),[](char c) {return (c>='0'&&c<='9')||(c>='a'&&c<='f');}) &&
    fstatat(directory,name.c_str(),&info,AT_SYMLINK_NOFOLLOW)==0 &&
    S_ISREG(info.st_mode) && privateOwner(info) && info.st_nlink==1 && info.st_size==0;
}

bool sync(int fd)
{
  int result;
  do {result=fsync(fd);} while(result<0&&errno==EINTR);
  return result==0;
}

bool bindRoute(int directory,const std::string& route)
{
  const int existing=openat(directory,route_file,O_RDONLY|O_NOFOLLOW|O_CLOEXEC|O_NONBLOCK);
  if(existing>=0) {
    struct stat info{};
    std::string contents(route.size(),'\0');
    const bool match=fstat(existing,&info)==0 && S_ISREG(info.st_mode) && privateOwner(info) &&
      info.st_nlink==1 && info.st_size==static_cast<off_t>(route.size()) &&
      read(existing,contents.data(),contents.size())==info.st_size && contents==route;
    close(existing);
    return match;
  }
  if(errno!=ENOENT) return false;
  const int fd=openat(directory,route_file,O_WRONLY|O_CREAT|O_EXCL|O_NOFOLLOW|O_CLOEXEC,0600);
  if(fd<0) return false;
  const bool written=write(fd,route.data(),route.size())==static_cast<ssize_t>(route.size()) && sync(fd);
  const bool closed=close(fd)==0;
  return written && closed && sync(directory);
}
}

ControlReplayJournal::ControlReplayJournal(const std::string& directory,const std::string& route,bool create):m_directory(directory)
{
  if(directory.empty() || directory.front()!='/' || directory.back()=='/' ||
     directory.size()>4096 || directory.find('\0')!=std::string::npos || route.size()>256)
    throw std::invalid_argument("native replay requires an absolute private directory");
  if(create) {
    if(mkdir(directory.c_str(),0700)!=0 && errno!=EEXIST)
      throw std::invalid_argument("native replay directory cannot be created");
    const auto parent=std::filesystem::path(directory).parent_path();
    const int parent_fd=open(parent.c_str(),O_RDONLY|O_DIRECTORY|O_CLOEXEC);
    const bool synced=parent_fd>=0 && sync(parent_fd);
    if(parent_fd>=0) close(parent_fd);
    if(!synced) throw std::invalid_argument("native replay parent cannot be synced");
  }
  m_fd=open(directory.c_str(),O_RDONLY|O_DIRECTORY|O_NOFOLLOW|O_CLOEXEC);
  try {
    if(m_fd<0 || !bound() || flock(m_fd,LOCK_EX|LOCK_NB)!=0 || !bound())
      throw std::invalid_argument("native replay directory must exist, be private and be exclusively lockable");
    if(!route.empty() && !bindRoute(m_fd,route))
      throw std::invalid_argument("native replay directory belongs to a different route or has an unsafe binding");
    const int copy=dup(m_fd);
    DIR* entries=copy<0?nullptr:fdopendir(copy);
    if(!entries) {
      if(copy>=0) close(copy);
      throw std::invalid_argument("native replay records cannot be read");
    }
    for(;;) {
      errno=0;
      const auto* entry=readdir(entries);
      if(!entry) {
        const int error=errno;
        closedir(entries);
        if(error) throw std::invalid_argument("native replay records cannot be read");
        break;
      }
      const std::string name=entry->d_name;
      if(name=="." || name==".." || (!route.empty() && name==route_file)) continue;
      if(m_records.size()>=capacity || !recordValid(m_fd,name)) {
        closedir(entries);
        throw std::invalid_argument("native replay directory contains invalid or excess records");
      }
      m_records.insert(name);
    }
  } catch(...) {
    if(m_fd>=0) close(m_fd);
    throw;
  }
}

ControlReplayJournal::~ControlReplayJournal()
{
  close(m_fd);
}

bool ControlReplayJournal::bound() const
{
  struct stat held{}, named{};
  return fstat(m_fd,&held)==0 && lstat(m_directory.c_str(),&named)==0 &&
    S_ISDIR(held.st_mode) && S_ISDIR(named.st_mode) && privateOwner(held) &&
    held.st_dev==named.st_dev && held.st_ino==named.st_ino;
}

bool ControlReplayJournal::seen(const std::string& id) const
{
  return m_records.count(recordName(id))!=0;
}

ControlReplayJournal::Result ControlReplayJournal::reserve(const std::string& id)
{
  if(id.empty() || id.size()>max_id) return Result::Unavailable;
  if(m_failed || !bound()) {m_failed=true;return Result::Unavailable;}
  const auto name=recordName(id);
  if(m_records.count(name)) return Result::Seen;
  if(m_records.size()>=capacity) return Result::Full;
  const int fd=openat(m_fd,name.c_str(),O_WRONLY|O_CREAT|O_EXCL|O_NOFOLLOW|O_CLOEXEC,0600);
  if(fd<0) {m_failed=true;return Result::Unavailable;}
  const bool file_synced=sync(fd);
  const bool closed=close(fd)==0;
  // Keep any created record after a failed write. Never send an unrecorded ID.
  m_records.insert(name);
  if(!file_synced || !closed || !sync(m_fd) || !bound()) {m_failed=true;return Result::Unavailable;}
  return Result::Reserved;
}
}
