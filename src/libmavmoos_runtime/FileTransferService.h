#pragma once
#include <cstdint>
#include <string>

namespace mavmoos::runtime {
// The file provider may ask only for this read-only subset. The host
// validates and serializes these requests; plugins never receive a socket.
enum class FileTransferAction : uint8_t { Terminate=1, List=3, OpenReadOnly=4, Read=5 };
struct FileTransferRequest {
  std::string id, operation_id, path;
  FileTransferAction action=FileTransferAction::List;
  uint16_t sequence=0;
  uint8_t session=0, count=0;
  uint32_t offset=0;
  double expires=0, operation_expires=0;
};
// Optional typed extension. A file provider requests it by cross-casting the
// trusted host. Unrelated startup modules keep the existing Plugin/Services ABI.
class FileTransferService {
public:
  virtual ~FileTransferService()=default;
  virtual bool fileTransferReady() const=0;
  virtual bool fileTransfer(const FileTransferRequest&)=0;
  virtual void retireFileTransferPacket(const std::string&,const std::string&)=0;
  virtual void cancelFileTransfer(const std::string&)=0;
  virtual void finishFileTransfer(const std::string&)=0;
};

class FileTransferListener {
public:
  virtual ~FileTransferListener()=default;
  virtual void onFileTransferSent(const FileTransferRequest&,double,bool)=0;
};
}
