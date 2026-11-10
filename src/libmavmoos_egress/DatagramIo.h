/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: DatagramIo.h                                    */
/*    DATE: October 29, 2026                                */
/************************************************************/
#pragma once
#include "Types.h"

namespace mavmoos::egress {
class ManagedEgress;
enum class WriteCode { Returned, WouldBlock, Error };
struct WriteResult {
  WriteCode code = WriteCode::Error;
  std::size_t bytes = 0;
  int error = 0;
};
enum class ReadCode { Data, WouldBlock, WrongPeer, Truncated, Error, Closed };
struct ReadResult {
  ReadCode code = ReadCode::WouldBlock;
  std::array<uint8_t, 4096> bytes{};
  std::size_t size = 0;
  Tick acquired = 0;
  int error = 0;
};
struct ReceiveBatch {
  std::array<ReadResult, 8> datagrams{};
  std::size_t size = 0;
};

// Only the serialized engine may initiate I/O. No public raw send or fd escape.
class DatagramIo {
public:
  virtual ~DatagramIo() = default;
private:
  friend class ManagedEgress;
  virtual WriteResult tryWrite(const Frame& frame) noexcept = 0;
  virtual ReadResult tryRead() noexcept = 0;
  virtual bool isOpen() const noexcept = 0;
  virtual uint16_t localPort() const noexcept = 0;
  virtual void close() noexcept = 0;
};
} // namespace mavmoos::egress
