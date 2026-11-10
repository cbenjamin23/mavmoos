// Diagnostic-only dyld constructor for A/B tests of an unchanged packaged executable.
#include <dlfcn.h>
#include <cstdio>
#include <cstdlib>

__attribute__((constructor)) static void retain_diagnostic_modules()
{
  // Canonical package first; legacy variables remain for archived split-DSO A/B diagnostics.
  const char* names[]={"MAVMOOS_DIAG_CONTROL_MODULE","MAVMOOS_DIAG_PARAMETER_MODULE",
                       "MAVMOOS_DIAG_COMMAND_MODULE","MAVMOOS_DIAG_HELM_MODULE"};
  const char* option=std::getenv("MAVMOOS_DIAG_NODELETE");
  const bool nodelete=option&&option[0]=='1';
  for(const char* key:names) {
    const char* path=std::getenv(key);
    if(!path)continue;
    void* handle=dlopen(path,RTLD_NOW|RTLD_LOCAL|(nodelete?RTLD_NODELETE:0));
    std::fprintf(stderr,"DIAG dlopen key=%s nodelete=%d handle=%p\n",key,nodelete,handle);
    if(!handle){std::fprintf(stderr,"DIAG loader error: %s\n",dlerror());std::abort();}
    // Retain code through the loader policy, not an extra logical handle.
    if(dlclose(handle)){std::fprintf(stderr,"DIAG dlclose failed\n");std::abort();}
  }
}
