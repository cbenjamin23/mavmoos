/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: ComponentMetadataPlugin.cpp                     */
/*    DATE: November 6, 2026                                */
/************************************************************/
#include "PassiveObserver.h"
#include <algorithm>
#include <array>
#include <iterator>
#include <cerrno>
#include <fcntl.h>
#include <filesystem>
#include <stdexcept>
#include <string_view>
#include <sys/stat.h>
#include <unistd.h>

namespace mavmoos::runtime {
namespace {
constexpr std::string_view general_uri="mftp:///component_metadata/general.json";
constexpr std::size_t document_limit=4096;
constexpr std::array<std::string_view,4> fields{
  "vendorName","modelName","firmwareVersion","hardwareVersion"};

class Fd {
public:
  explicit Fd(int fd=-1):m_fd(fd) {}
  ~Fd() { reset(); }
  Fd(const Fd&)=delete;
  Fd& operator=(const Fd&)=delete;
  int get() const {return m_fd;}
  int release() {const int fd=m_fd;m_fd=-1;return fd;}
  void reset(int fd=-1) {if(m_fd>=0)close(m_fd);m_fd=fd;}
private:
  int m_fd;
};

int openPrivateDirectory(const std::string& directory)
{
  if(directory.size()<2 || directory.size()>4096 || directory.find('\0')!=std::string::npos)
    return -1;
  const std::filesystem::path path(directory);
  if(!path.is_absolute() || path.lexically_normal().string()!=directory || !path.has_filename())
    return -1;
  constexpr int flags=O_RDONLY|O_DIRECTORY|O_NOFOLLOW|O_CLOEXEC;
  Fd result(open("/",flags));
  for(const auto& name:path.relative_path()) {
    if(result.get()<0 || name.empty() || name=="." || name=="..")return -1;
    result.reset(openat(result.get(),name.c_str(),flags));
  }
  struct stat info{};
  return result.get()>=0 && fstat(result.get(),&info)==0 && info.st_uid==geteuid() &&
    (info.st_mode&077)==0?result.release():-1;
}

uint32_t mavlinkCrc32(std::string_view bytes)
{
  uint32_t crc=0;
  for(unsigned char byte:bytes) {
    crc^=byte;
    for(unsigned bit=0;bit<8;++bit) crc=(crc>>1)^((crc&1)?0xedb88320u:0u);
  }
  return crc;
}

class JsonInput {
public:
  explicit JsonInput(std::string_view bytes):m_bytes(bytes) {}
  bool take(char value)
  {
    whitespace();
    if(m_bytes.empty() || m_bytes.front()!=value)return false;
    m_bytes.remove_prefix(1);
    return true;
  }
  bool atEnd() {whitespace();return m_bytes.empty();}
  std::optional<std::string> string()
  {
    if(!take('"'))return {};
    std::string out;
    while(!m_bytes.empty()) {
      unsigned char c=m_bytes.front();m_bytes.remove_prefix(1);
      if(c=='"')return out;
      if(c=='\\') {
        if(m_bytes.empty())return {};
        c=m_bytes.front();m_bytes.remove_prefix(1);
        if(c=='u') {
          if(m_bytes.size()<4)return {};
          unsigned code=0;
          const auto parsed=std::from_chars(m_bytes.data(),m_bytes.data()+4,code,16);
          if(parsed.ec!=std::errc{} || parsed.ptr!=m_bytes.data()+4 || code>0x7e)return {};
          c=static_cast<unsigned char>(code);
          m_bytes.remove_prefix(4);
        } else if(c!='"' && c!='\\' && c!='/')return {};
      }
      if(c<0x20 || c>0x7e || out.size()>=96)return {};
      out+=static_cast<char>(c);
    }
    return {};
  }
private:
  void whitespace()
  {
    const auto at=m_bytes.find_first_not_of(" \t\r\n");
    m_bytes.remove_prefix(at==std::string_view::npos?m_bytes.size():at);
  }
  std::string_view m_bytes;
};

struct Document {
  std::array<std::optional<std::string>,fields.size()> text;
};

bool versionString(std::string_view version)
{
  unsigned parts=0;
  while(!version.empty()) {
    const auto dot=version.find('.');
    const auto value=version.substr(0,dot);
    if(value.empty() || value.size()>9 || value.find_first_not_of("0123456789")!=std::string_view::npos)
      return false;
    ++parts;
    if(dot==std::string_view::npos)return parts==3 || parts==4;
    version.remove_prefix(dot+1);
  }
  return false;
}

std::optional<Document> parseGeneralDocument(std::string_view bytes)
{
  JsonInput input(bytes);
  if(!input.take('{'))return {};
  bool version=false,types=false;
  Document document;
  do {
    const auto key=input.string();
    if(!key || !input.take(':'))return {};
    if(*key=="version") {
      if(version || !input.take('1'))return {};
      version=true;
    } else if(*key=="metadataTypes") {
      if(types || !input.take('[') || !input.take(']'))return {};
      types=true;
    } else {
      const auto field=std::find(fields.begin(),fields.end(),*key);
      if(field==fields.end())return {};
      const auto index=static_cast<std::size_t>(field-fields.begin());
      auto value=input.string();
      if(document.text[index] || !value || (index>=2 && !versionString(*value)))return {};
      document.text[index]=std::move(*value);
    }
  } while(input.take(','));
  if(!version || !types || !input.take('}') || !input.atEnd())return {};
  return document;
}

std::string quoted(const std::string& value)
{
  std::string out="\"";
  for(char c:value) {if(c=='"' || c=='\\')out+='\\';out+=c;}
  return out+'"';
}

class ComponentMetadata final : public PassiveObserver {
public:
  explicit ComponentMetadata(Services& services):PassiveObserver(services,true,3)
  {setObservationComponent(0);}

  std::string name() const override {return "component_metadata";}

  PluginDescriptor descriptor() const override
  {
    PluginDescriptor result;
    result.profile="mavlink_component_metadata_local_mirror";
    result.capabilities={{"observations.component_metadata","","","observation"}};
    result.exclusive_resources={"observation.component_metadata"};
    return result;
  }

  void configure(const std::string& key,const std::string& value) override
  {
    if(key!="component_metadata_component")return;
    unsigned component=0;
    const auto parsed=std::from_chars(value.data(),value.data()+value.size(),component);
    if(parsed.ec!=std::errc{} || parsed.ptr!=value.data()+value.size() || component<1 || component>255)
      throw std::invalid_argument("component_metadata_component must be in [1,255]");
    setObservationComponent(component);
  }

  ModuleConfigResult configureModule(const std::vector<ModuleSetting>& settings) override
  {
    if(!observationComponent())
      return {ModuleConfigStatus::Invalid,"component_metadata_component","selected component required"};
    std::vector<ModuleSetting> observer;
    std::optional<std::string> directory;
    for(const auto& setting:settings) {
      if(setting.key!="mirror_directory") observer.push_back(setting);
      else if(directory) return {ModuleConfigStatus::Invalid,"mirror_directory","duplicate setting"};
      else directory=setting.value;
    }
    if(!directory)return {ModuleConfigStatus::Invalid,"mirror_directory","private mirror directory required"};
    Fd fd(openPrivateDirectory(*directory));
    if(fd.get()<0)
      return {ModuleConfigStatus::Invalid,"mirror_directory","expected absolute owner-private directory without symlinks"};
    const auto result=PassiveObserver::configureModule(observer);
    if(result.status!=ModuleConfigStatus::Accepted)return result;
    m_directory=std::move(*directory);
    m_dir.reset(fd.release());
    return {};
  }

private:
  bool observes(uint32_t id) const override {return id==MAVLINK_MSG_ID_COMPONENT_METADATA;}

  void observe(const mavlink_message_t& message,const MessageTime& time) override
  {
    if(message.magic!=MAVLINK_STX || message.len<=8 || !wireShape(message,
       MAVLINK_MSG_ID_COMPONENT_METADATA_MIN_LEN,MAVLINK_MSG_ID_COMPONENT_METADATA_LEN)) {
      rejectWire();return;
    }
    mavlink_component_metadata_t packet{};
    mavlink_msg_component_metadata_decode(&message,&packet);
    const auto uri_end=std::find(std::begin(packet.uri),std::end(packet.uri),'\0');
    const bool supported=uri_end!=std::end(packet.uri) &&
      std::string_view(packet.uri,static_cast<std::size_t>(uri_end-packet.uri))==general_uri;
    if(m_stamp.id) {
      const uint32_t delta=packet.time_boot_ms-m_boot;
      if(!delta && packet.file_crc==m_crc && supported==m_supported_uri)return;
      if(!delta || delta>=0x80000000u) {
        if(!update(m_last_report,time))return;
        m_document.reset();
        m_pending=false;
        m_state="declaration_conflict_or_boot_regression";
        return;
      }
    }
    if(!update(m_last_report,time))return;
    m_stamp=m_last_report;
    m_boot=packet.time_boot_ms;
    m_crc=packet.file_crc;
    m_document.reset();
    m_supported_uri=supported;
    m_pending=supported;
    m_state=supported?"pending":"unsupported_uri";
  }

  void clearSamples() override
  {
    m_stamp={};
    m_last_report={};
    m_document.reset();
    m_boot=m_crc=0;
    m_supported_uri=m_pending=false;
    m_state="waiting_declaration";
  }

  const char* verify()
  {
    Fd named(openPrivateDirectory(m_directory));
    struct stat original{},current{},file{};
    if(named.get()<0 || fstat(m_dir.get(),&original)!=0 || fstat(named.get(),&current)!=0 ||
       original.st_dev!=current.st_dev || original.st_ino!=current.st_ino)
      return "mirror_directory_changed";

    Fd input(openat(named.get(),"general.json",O_RDONLY|O_NOFOLLOW|O_CLOEXEC|O_NONBLOCK));
    if(input.get()<0 || fstat(input.get(),&file)!=0 || !S_ISREG(file.st_mode) ||
       file.st_uid!=geteuid() || file.st_nlink!=1 || (file.st_mode&077)!=0)
      return "mirror_file_unavailable";
    if(file.st_size<=0 || static_cast<uint64_t>(file.st_size)>document_limit)return "document_size_invalid";

    std::array<char,document_limit+1> bytes{};
    std::size_t size=0;
    while(size<bytes.size()) {
      const auto count=read(input.get(),bytes.data()+size,bytes.size()-size);
      if(count<0 && errno==EINTR)continue;
      if(count<0)return "mirror_read_failed";
      if(!count)break;
      size+=static_cast<std::size_t>(count);
    }
    if(size!=static_cast<std::size_t>(file.st_size))return "document_size_changed";
    const std::string_view data(bytes.data(),size);
    if(mavlinkCrc32(data)!=m_crc)return "checksum_mismatch";
    m_document=parseGeneralDocument(data);
    return m_document?"verified":"unsupported_document_schema";
  }

  void publishSamples() override
  {
    const bool fresh=m_stamp.id && services().now()-m_stamp.time.effective_at<=sampleTimeout();
    if(m_pending && fresh) {
      m_pending=false;
      m_state=verify();
    }
    if(!fresh) {
      m_pending=false;
      m_document.reset();
    }
    std::ostringstream out;
    out << "{\"schema\":1,\"verification\":\"" << (m_stamp.id&&!fresh?"stale_declaration":m_state)
        << "\",\"method\":\"owned_local_mirror\",\"metadata\":" << metadata(m_stamp)
        << ",\"declaration\":";
    if(m_stamp.id) {
      out << "{\"time_boot_ms\":" << m_boot << ",\"file_crc32\":" << m_crc << ",\"uri\":"
          << (m_supported_uri?quoted(std::string(general_uri)):"null") << '}';
    } else out << "null";
    out << ",\"document\":";
    if(fresh && m_document) {
      out << "{\"version\":1,\"metadataTypes\":[]";
      for(std::size_t i=0;i<fields.size();++i)
        if(m_document->text[i])out << ",\"" << fields[i] << "\":" << quoted(*m_document->text[i]);
      out << '}';
    } else out << "null";
    out << '}';
    services().publish("MAVMOOS_COMPONENT_METADATA",out.str());
  }

  Fd m_dir;
  std::string m_directory;
  Stamp m_stamp, m_last_report;
  std::optional<Document> m_document;
  uint32_t m_boot=0,m_crc=0;
  bool m_supported_uri=false,m_pending=false;
  const char* m_state="waiting_declaration";
};
}

extern "C" Plugin* mavmoos_create_plugin(unsigned abi,Services* services)
{
  return abi==plugin_abi && services?new ComponentMetadata(*services):nullptr;
}

extern "C" void mavmoos_destroy_plugin(Plugin* plugin) {delete plugin;}

} // namespace mavmoos::runtime
