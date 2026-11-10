#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace mavmoos::ftp {

inline constexpr std::string_view directory = "mavmoos-ftp";
inline constexpr std::uint8_t bridge_system = 245;
inline constexpr std::uint8_t bridge_component = 191;
inline constexpr std::uint8_t fc_system = 1;
inline constexpr std::uint8_t fc_component = 1;
inline constexpr std::uint8_t client_session = 1;
inline constexpr std::size_t header_size = 12;
inline constexpr std::size_t data_size = 239;
inline constexpr std::size_t max_file_size = 65536;
inline constexpr std::size_t max_basename = 80;
inline constexpr std::uint32_t max_list_offset = 127;
inline constexpr double max_operation_seconds = 60.0;
inline constexpr double cleanup_seconds = 2.0;
using Payload = std::array<std::uint8_t, header_size + data_size>;

enum class Opcode : std::uint8_t {
  TerminateSession = 1,
  ListDirectory = 3,
  OpenFileRO = 4,
  ReadFile = 5,
};

enum class Phase {
  Idle, Listing, Listed, Opening, Reading, VerifyingEof, Closing,
  Complete, Cancelled, Expired, Failed,
};

enum class RemoteSession { None, Possible, Open, ConfirmedClosed, Uncertain };
enum class EntryKind { File, Directory };

struct DirectoryEntry {
  std::string name;
  EntryKind kind;
  std::uint32_t size;
};

struct Intent {
  Opcode opcode;
  std::uint16_t sequence;
  std::uint8_t session;
  std::uint32_t offset;
  std::uint8_t size;
  std::string path;
  std::uint64_t token;
  double expires;
  double operation_expires;
  double cleanup_expires;
};

struct Reply {
  std::uint8_t source_system;
  std::uint8_t source_component;
  std::uint8_t target_network;
  std::uint8_t target_system;
  std::uint8_t target_component;
  Payload payload;
  double effective_at;
  double read_at;
};

struct Status {
  Phase phase = Phase::Idle;
  std::string_view reason = "idle";
  RemoteSession remote_session = RemoteSession::None;
  double operation_expires = 0;
  double cleanup_expires = 0;
  std::size_t listed_entries = 0;
  std::size_t received_bytes = 0;
  std::optional<std::uint32_t> announced_bytes;
  std::optional<std::uint8_t> remote_error;
  std::optional<std::uint8_t> remote_errno;
};

struct File {
  std::string name;
  std::vector<std::uint8_t> bytes;
};

bool safeBasename(std::string_view name);
bool active(Phase phase);
std::optional<Payload> encodePayload(const Intent& intent);

// The caller owns the monotonic clock, session/address authentication and final socket handoff.
// Check eligible just before handoff and report that attempt with sent, including local drops.
class Transaction {
public:
  explicit Transaction(std::uint16_t initial_sequence = 0);

  bool beginList(double now, double deadline);
  bool beginList(std::string_view relative_directory, double now, double deadline);
  bool beginGet(std::string_view listed_leaf, double now, double deadline);

  std::optional<Intent> take(double now);
  bool eligible(const Intent& intent, double now) const;
  bool sent(const Intent& intent, double now, bool local_accepted);
  bool receive(const Reply& reply, double now);
  void advance(double now);
  bool cancel(double now);
  void reset(double now);

  const Status& status() const { return m_status; }
  const std::vector<DirectoryEntry>& listing() const { return m_listing; }
  std::optional<File> takeFile();

private:
  struct Pending {
    Opcode opcode;
    std::uint16_t sequence;
    std::uint8_t session;
    std::uint32_t offset;
    std::uint8_t size;
    std::string path;
    double ready_at;
    std::uint8_t attempts = 0;
    std::optional<double> first_sent_at;
    std::optional<Intent> issued;
  };

  bool clock(double now);
  bool startAllowed(double now, double deadline) const;
  void queue(Opcode opcode, std::uint32_t offset, std::uint8_t size,
             std::string_view path, double now);
  void queueRead(double now);
  bool parseListPage(const Payload& payload, std::uint8_t size);
  void acknowledge(const Reply& reply, double now);
  void reject(const Reply& reply, double now);
  void abandon(Phase phase, std::string_view reason, double now);
  void close(Phase final_phase, std::string_view reason, double now);
  void finish(Phase phase, std::string_view reason);
  void retirePending();

  Status m_status;
  std::optional<Pending> m_pending;
  std::vector<DirectoryEntry> m_listing;
  std::vector<DirectoryEntry> m_listing_stage;
  std::vector<std::uint8_t> m_file_stage;
  std::optional<File> m_file;
  std::string m_name;
  std::uint32_t m_list_offset = 0;
  std::uint32_t m_listed_size = 0;
  std::uint16_t m_next_sequence;
  std::uint64_t m_next_token = 0;
  double m_next_send_at = 0;
  double m_recover_at = 0;
  Phase m_after_close = Phase::Complete;
  std::string_view m_after_close_reason;
  std::optional<double> m_high_water;
  bool m_clock_fault = false;
};

}  // namespace mavmoos::ftp
