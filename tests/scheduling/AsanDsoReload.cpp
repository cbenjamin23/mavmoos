// Diagnostic only: real packaged modules, isolated loader lifetime, no Runtime or I/O.
#include "Plugin.h"
#include <dlfcn.h>
#include <iostream>
#include <stdexcept>
#include <vector>
using namespace mavmoos::runtime;
struct Host final:Services {
  Vehicle state;
  double now()const override{return 100;}
  const Vehicle& vehicle()const override{return state;}
  void publish(const std::string&,const std::string&)override{}
  void publish(const std::string&,double)override{}
  bool sendCommand(const CommandRequest&)override{return false;}
  bool acquireNavigation(const std::string&)override{return false;}
  void revokeNavigation(const std::string&)override{}
  bool navigation(const NavigationTarget&)override{return false;}
};
int main(int argc,char** argv) {
  try {
    if(argc<4)throw std::runtime_error("ARGS: MODULE [MODULE...] TOUCH_ITERATION HOLD_REFERENCE");
    const int module_end=argc-2;
    const unsigned touch=static_cast<unsigned>(std::stoul(argv[argc-2]));
    std::vector<void*> retained;
    if(std::stoi(argv[argc-1]))for(int i=1;i<module_end;++i){void* h=dlopen(argv[i],RTLD_NOW|RTLD_LOCAL);if(!h)throw std::runtime_error(dlerror());retained.push_back(h);}
    for(unsigned round=0;round<=touch;++round) {
      Host host;std::vector<void*> handles;std::vector<Plugin*> modules;std::vector<DestroyPlugin> destroyers;
      for(int i=1;i<module_end;++i) {
        void* h=dlopen(argv[i],RTLD_NOW|RTLD_LOCAL);if(!h)throw std::runtime_error(dlerror());
        auto create=reinterpret_cast<CreatePlugin>(dlsym(h,"mavmoos_create_plugin"));
        auto destroy=reinterpret_cast<DestroyPlugin>(dlsym(h,"mavmoos_destroy_plugin"));
        if(!create||!destroy)throw std::runtime_error("factory exports");
        Plugin* p=create(plugin_abi,&host);if(!p)throw std::runtime_error("factory ABI/context");
        handles.push_back(h);modules.push_back(p);destroyers.push_back(destroy);
      }
      std::cout<<"round="<<round<<" touch="<<(round==touch)<<" retained="<<retained.size()<<'\n'<<std::flush;
      if(round==touch) {
        Mail m;m.key="MAVMOOS_COMMAND";m.text="id=touch,action=hold";m.published_at=m.received_at=100.1;
        modules.front()->onMail(m);
      }
      while(!modules.empty()) {
        destroyers.back()(modules.back());modules.pop_back();destroyers.pop_back();
        dlclose(handles.back());handles.pop_back();
      }
    }
    for(void* h:retained)dlclose(h);
    std::cout<<"PASS loader lifetime diagnostic\n";return 0;
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
