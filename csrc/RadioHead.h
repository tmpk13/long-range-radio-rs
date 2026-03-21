// Minimal RadioHead platform header for bare-metal Rust FFI.
// This file shadows RadioHead/RadioHead.h via include path priority,
// providing only the platform glue the routing stack needs.

#ifndef RadioHead_h
#define RadioHead_h

#include <stdint.h>
#include <string.h>

// ---- Platform identity (must not match any built-in) ----
#define RH_PLATFORM              100
#define RH_PLATFORM_RUST         100

// ---- Features we do NOT provide ----
// (no RH_HAVE_SERIAL, no RH_HAVE_HARDWARE_SPI)

// ---- Constants ----
#define RH_BROADCAST_ADDRESS     0xff

// ---- Atomic blocks (routing is single-threaded) ----
#define ATOMIC_BLOCK_START       {
#define ATOMIC_BLOCK_END         }

// ---- Yield (no-op; user can override) ----
#ifndef YIELD
#define YIELD
#endif

// ---- Misc ----
#define RH_INTERRUPT_ATTR
#define PROGMEM
#define memcpy_P  memcpy

// ---- Arduino-style print base constants ----
#define DEC 10
#define HEX 16

// ---- Byte-order (RISC-V & Xtensa are little-endian) ----
#if !defined(htons)
#define htons(x) ( ((x)<<8) | (((x)>>8)&0xFF) )
#define ntohs(x) htons(x)
#define htonl(x) ( ((x)<<24 & 0xFF000000UL) | \
                   ((x)<< 8 & 0x00FF0000UL) | \
                   ((x)>> 8 & 0x0000FF00UL) | \
                   ((x)>>24 & 0x000000FFUL) )
#define ntohl(x) htonl(x)
#endif

// ---- Platform functions (implemented in Rust, linked at final binary) ----
#ifdef __cplusplus
extern "C" {
#endif
    unsigned long rh_millis(void);
    void          rh_delay(unsigned long ms);
    long          rh_random(long min_val, long max_val);
#ifdef __cplusplus
}
#endif

// ---- Arduino-compatible wrappers (C++ only) ----
#ifdef __cplusplus
inline unsigned long millis()                      { return rh_millis(); }
inline void          delay(unsigned long ms)       { rh_delay(ms); }
inline long          random(long lo, long hi)      { return rh_random(lo, hi); }
inline long          random(long hi)               { return rh_random(0, hi); }
#endif

// ---- Minimal Print stub (needed by RHRouter::printRoutingTable) ----
#ifdef __cplusplus
class Print {
public:
    void print(int, int = 10)   {}
    void print(const char*)     {}
    void println(int, int = 10) {}
    void println(const char*)   {}
    void println()              {}
};
#endif

#endif // RadioHead_h
