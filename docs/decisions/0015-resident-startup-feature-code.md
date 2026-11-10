# 0015: Process-resident startup feature code

Date: November 3, 2026. Status: implemented loader policy. Startup-selected shared-library plugins remain the packaging model; live unloading/replacement is unsupported.

The packaged real-plugin scheduling test exposed an ASan global-redzone failure when one feature image was unloaded and another reused its address. The archive contains the failing trace, module/global address chronology and exact artifacts. Changing Mach-O BUNDLE to DYLIB did not resolve it. With the same executable and module bytes, a small `dlopen` interposer adding only `RTLD_NODELETE` changed the failing run to a pass. This supports image-residency policy as the relevant difference; it does not establish the exact internals of Apple's sanitizer runtime.

The host now loads trusted startup modules with `RTLD_NOW | RTLD_LOCAL | RTLD_NODELETE` in every build. Plugin objects are still destroyed through their exported destroy factory while code is available; the host still releases logical handles with `dlclose`. Module code/static storage remains mapped for process lifetime. This is ordinary loader behavior on the tested Darwin/Linux platforms, not an ASan suppression or test-only unpoisoning. Matching ABI 4 host/modules are required.

The cost is resident module mappings until process exit. That is acceptable for a fixed startup feature set in one bridge process. It also makes the intended lifecycle explicit: changing feature code requires a process restart and fresh authority, rather than implying support for hot replacement. `NODELETE` does not sandbox plugins, make their C++ ABI stable or replace per-instance cleanup. A future live-unload requirement would need separate lifecycle and platform validation.

See the sanitizer investigation, independent Linux reproduction and final verification ledger. The historical failure remains archived separately from later passing runs.
