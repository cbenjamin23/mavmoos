#pragma once
#include "ControlBackend.h"
#include <memory>

namespace mavmoos::runtime {
// Internal components, not separately loaded public capability plugins.
class ArduCopterControlBackend : public VehicleControlBackend {
public:
  virtual void bindFrameType(unsigned type)=0;
};
std::unique_ptr<ArduCopterControlBackend> makeArduCopterBackend();
std::unique_ptr<Plugin> makeCopterCommands(Services&);
void deliverCopterNativeRequest(Plugin&,const Mail&,const std::string& public_id);
std::unique_ptr<Plugin> makeHelmAdapter(Services&,const VehicleControlBackend&);
} // namespace mavmoos::runtime
