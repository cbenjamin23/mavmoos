/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: CopterOperations.h                              */
/*    DATE: November 4, 2026                                */
/************************************************************/
#pragma once
#include "ControlComponents.h"

namespace mavmoos::runtime {
// Private coordinator inside the one public ArduCopter provider.
class CopterOperations : public Plugin {
public:
  virtual void setFacets(bool native_actions,bool helm)=0;
  virtual std::vector<Capability> capabilities() const=0;
  virtual void observePublication(const std::string& key,const std::string& value)=0;
  virtual bool boundHelmTaskTarget(NavigationTarget&) const=0;
  virtual bool boundNativeCommand(CommandRequest&) const=0;
  virtual void nativeSubmitted(const CommandRequest&,bool accepted)=0;
  virtual bool nativeFinished(const std::string& id)=0;
  virtual void afterComponents()=0;
};
std::unique_ptr<CopterOperations> makeCopterOperations(Services&,Plugin& native,Plugin& helm,
                                                     const VehicleControlBackend&);
} // namespace mavmoos::runtime
