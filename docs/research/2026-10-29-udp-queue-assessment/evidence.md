# Evidence and provenance

October 29, 2026. Static inspection and architecture reasoning. The manifest records **17 pinned source files** plus hashes of five preserved architecture inputs at MAVMOOS predecessor `602a2a1`. No patch, backend, build, transport experiment or firmware action occurred.

## Baselines

- **libmavconn:** thirteen files at MAVROS `2d3f4db1101c7950c8817a9d788f0d11106fb261`. The ros2 head was rechecked October 29 and still matched the earlier comparison. Eleven files reuse that study's exact-commit source/tree matches; MsgBuffer and IoContextRunner add two matching tree-blob checks.
- **Generated MAVLink:** two codec/type files at `mavlink/c_library_v2` `a2d0ecf011a4aaa7840480e43f4e38d8c9e79d29`, matched to the fetched blob IDs.
- **Asio API reference:** `asio-1-36-0`, commit `231cb29bab30f82712fcd54faaea42424cc6e710`; two exact-commit headers matched fetched blob IDs. This is a versioned API/documentation reference, **not** the selected or installed libmavconn I/O dependency.
- **Architecture basis:** dated Copter README/loss policy, earlier backend README/criteria and proposed decision 0005 remain byte-identical to their recorded predecessor revision. Their interpretation is refined separately here; no earlier profile or policy is silently amended.

The source bytes were retained in task scratch space for inspection. MAVMOOS archives this analysis and immutable provenance, without integrating third-party source or adding application implementation. Dependency, platform, firmware and actual first link remain unselected.

## Source map

| ID | Observed finding | Pinned primary source |
| --- | --- | --- |
| S-01 | Private socket/peer/FIFO/RX storage and write entry points | [udp.hpp 85–103](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/libmavconn/include/mavconn/udp.hpp#L85-L103) |
| S-02 | Admission copies work into FIFO; send-chain kick is coalesced | [udp.cpp 227–263](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/libmavconn/src/udp.cpp#L227-L263) |
| S-03 | Active async operation borrows front storage; completion advances/drains FIFO, with transient datagram drops | [udp.cpp 350–419](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/libmavconn/src/udp.cpp#L350-L419) |
| S-04 | Initial remote resolution is distinct from later receive-driven remote updates | [udp.cpp 96–108](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/libmavconn/src/udp.cpp#L96-L108), [309–346](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/libmavconn/src/udp.cpp#L309-L346) |
| S-05 | Frame-sized buffer utility; typed finalization versus already packed/raw buffers | [msgbuffer.hpp 32–95](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/libmavconn/include/mavconn/msgbuffer.hpp#L32-L95) |
| S-06 | Raw/framing callback has no datagram acquisition envelope | [interface.hpp 117–120](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/libmavconn/include/mavconn/interface.hpp#L117-L120), [interface.cpp 112–137](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/libmavconn/src/interface.cpp#L112-L137) |
| S-07 | Close cancels/closes socket; owned/shared I/O shutdown has separate execution/lifetime behavior | [udp.cpp 161–193](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/libmavconn/src/udp.cpp#L161-L193), [io_context_runner.hpp](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/libmavconn/include/mavconn/io_context_runner.hpp) |
| S-08 | Library owns mutable parser/TX/signing state; bypass must respect or explicitly replace its codec boundary | [interface.hpp 329–367](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/libmavconn/include/mavconn/interface.hpp#L329-L367), [interface.cpp 233–288](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/libmavconn/src/interface.cpp#L233-L288) |
| S-09 | Asio async-send buffer lifetime, deferred completion and per-operation cancellation reference | [basic_datagram_socket.hpp 644–696](https://github.com/chriskohlhoff/asio/blob/231cb29bab30f82712fcd54faaea42424cc6e710/asio/include/asio/basic_datagram_socket.hpp#L644-L696) |
| S-10 | Synchronous nonblocking mode differs from async operation behavior | [basic_socket.hpp 1228–1286](https://github.com/chriskohlhoff/asio/blob/231cb29bab30f82712fcd54faaea42424cc6e710/asio/include/asio/basic_socket.hpp#L1228-L1286) |
| S-11 | Generated codec exposes frame encoding and caller-owned parser state | [mavlink_helpers.h 525–585](https://github.com/mavlink/c_library_v2/blob/a2d0ecf011a4aaa7840480e43f4e38d8c9e79d29/mavlink_helpers.h#L525-L585), [697–725](https://github.com/mavlink/c_library_v2/blob/a2d0ecf011a4aaa7840480e43f4e38d8c9e79d29/mavlink_helpers.h#L697-L725) |
| S-12 | Build/package surface remains distinct from a selected standalone/platform dependency | [CMakeLists.txt 10–54](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/libmavconn/CMakeLists.txt#L10-L54), [package.xml](https://github.com/mavlink/mavros/blob/2d3f4db1101c7950c8817a9d788f0d11106fb261/libmavconn/package.xml) |

## API context and inference

Versioned Asio pages consulted: [async_send_to](https://think-async.com/Asio/asio-1.36.0/doc/asio/reference/basic_datagram_socket/async_send_to/overload1.html), [socket cancel](https://think-async.com/Asio/asio-1.36.0/doc/asio/reference/basic_socket/cancel/overload1.html), and [nonblocking mode](https://think-async.com/Asio/asio-1.36.0/doc/asio/reference/basic_datagram_socket/non_blocking/overload1.html). [sendto(3p)](https://man7.org/linux/man-pages/man3/sendto.3p.html) is a POSIX.1-2017 reference reprint, used for local acceptance/error/delivery distinctions, not evidence of a chosen OS's runtime behavior. All were consulted October 29; none establishes a timing or remote-recall guarantee.

L1/L2/D interfaces, fixed-peer first scope, local fence/receipt interpretation, queue capacities/classes and direct-UDP preference are **new design proposals inferred from the source constraints**. Source observations do not prove that a vendor extension is implemented, that stock discovery behavior is defective, or that owned code would be more reliable. A shared I/O context or smaller FIFO alone does not supply the missing metadata/cancellation results.

The study deliberately does not estimate lines of code/person-days, select a stable dependency version, or claim ABI/platform compatibility. No measured scheduling, race, backpressure, remote delivery, FC timeout/capture or physical-stop result exists. Later qualification must use the actual version/platform/link/firmware and the [review cases](direct-udp-and-review.md#5-paper-review-cases).

## Documentation checks completed

At creation, all 17 source records passed blob/SHA-256/length checks and the five recorded architecture inputs matched their predecessor bytes. Local links/headings and immutable source-span bounds passed; all 14 UDP case IDs and 12 source-map IDs were unique/complete. Twenty-seven checked historical files remained unchanged, including the dated Copter profile and previous backend comparison. The README remained within its medium-length target. These are provenance/documentation checks; none of the paper scenarios was executed against a transport or vehicle.
