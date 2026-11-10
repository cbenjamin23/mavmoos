/************************************************************/
/*    NAME: Charles Benjamin                                */
/*    ORGN: MAVMOOS project                                 */
/*    FILE: IntervalRequestFixture.cpp                      */
/*    DATE: November 4, 2026                                */
/************************************************************/
// Test-only observer: prove min-interval selection with unequal demands.
#include "Plugin.h"
using namespace mavmoos::runtime;
namespace {
class IntervalFixture final:public Plugin {
public:
  std::string name()const override{return "observer_review_intervals";}
  PluginDescriptor descriptor()const override
  {
    PluginDescriptor d;d.profile="test_interval_requests";
    d.capabilities={{"test.observer_intervals","","","observation"}};
    d.observation_streams={{33,100000},{24,250000}};return d;
  }
  void iterate()override{}
};
}
extern "C" Plugin* mavmoos_create_plugin(unsigned abi,Services* services)
{return abi==plugin_abi&&services?new IntervalFixture():nullptr;}
extern "C" void mavmoos_destroy_plugin(Plugin* plugin){delete plugin;}
