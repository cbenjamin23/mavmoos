#pragma once
#include <set>
#include <string>

namespace mavmoos::runtime {
class ControlReplayJournal {
public:
  enum class Result { Reserved, Seen, Full, Unavailable };
  explicit ControlReplayJournal(const std::string& directory,const std::string& route={},bool create=false);
  ~ControlReplayJournal();
  ControlReplayJournal(const ControlReplayJournal&)=delete;
  ControlReplayJournal& operator=(const ControlReplayJournal&)=delete;
  bool seen(const std::string& id) const;
  Result reserve(const std::string& id);
  std::size_t size() const { return m_records.size(); }
  static constexpr std::size_t capacity=4096;
private:
  bool bound() const;
  std::string m_directory;
  std::set<std::string> m_records;
  int m_fd=-1;
  bool m_failed=false;
};
}
