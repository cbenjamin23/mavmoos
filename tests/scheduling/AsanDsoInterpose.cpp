// Diagnostic only: compare ordinary vs NODELETE dlopen without changing packaged code.
#include <dlfcn.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>

static void* diagnostic_dlopen(const char* path,int flags)
{
  const char* option=std::getenv("MAVMOOS_DIAG_NODELETE");
  const bool pin=option&&option[0]=='1'&&path&&std::strstr(path,"mavmoos_");
  if(pin)flags|=RTLD_NODELETE;
  return dlopen(path,flags);
}
__attribute__((used,section("__DATA,__interpose")))
static const struct {const void* replacement;const void* original;} interposition = {
  reinterpret_cast<const void*>(&diagnostic_dlopen),reinterpret_cast<const void*>(&dlopen)
};
