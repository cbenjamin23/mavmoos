#pragma once
#include <cstddef>

// This is a test-only, inert inspection surface. It is not part of the SDK.
namespace configuration_fixture {
enum class Hook : unsigned { None, Factory, Checked, Legacy, Destroy };
enum class Effect : unsigned {
  None, Command, Acquire, Revoke, Navigation, Parameter, Mission, MissionFinished,
  RecordProof, InvalidateProof, BeginExecution, BeginAbort, FinishExecution,
  Suspend, Resume, CommandFinished, CancelCommand, FlightLog, FlightLogFinished, Count
};
struct Audit {
  unsigned constructed=0, destroyed=0, checked=0, legacy=0;
  unsigned mail=0, message=0, iterations=0, input_boundaries=0, resets=0;
  unsigned attempted=0, accepted=0, caught=0, query_grants=0;
  unsigned last_batch_size=0, committed=0, backend_replacements=0;
  std::size_t value_size=0;
  char value[128]{};
  char callbacks[64][97]{};
};
using Read=const Audit* (*)();
using Reset=void (*)();
using Control=void (*)(unsigned,unsigned);
using SetName=void (*)(const char*);
}
