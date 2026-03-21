// C API for RadioHead routing stack, callable from Rust via FFI.
#ifndef RH_SHIM_H
#define RH_SHIM_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- Driver callback table (Rust → C++) ----
// The Rust RadioDriver trait is represented as a vtable + opaque context pointer.
// The C++ RustDriver class calls these to perform actual radio I/O.
typedef struct {
    // Poll for a received packet. Non-blocking.
    // Writes raw over-the-air bytes (4-byte RH header + payload) into buf.
    // Sets *len to the number of bytes written, *rssi to the RSSI.
    // Returns true if a packet was available.
    bool (*poll_recv)(void* ctx, uint8_t* buf, uint8_t* len, int16_t* rssi);

    // Send a raw over-the-air packet (4-byte RH header + payload).
    // Should block until transmission is complete.
    // Returns true on success.
    bool (*send)(void* ctx, const uint8_t* data, uint8_t len);

    // Maximum over-the-air packet size (including 4-byte RH header).
    uint8_t (*max_message_length)(void* ctx);
} RhDriverVtable;

// ---- Lifecycle ----
// Create the global RustDriver + RHMesh objects (static storage, one instance).
void rh_create(const RhDriverVtable* vtable, void* ctx, uint8_t this_address);

// Initialise the mesh stack.  Call after rh_create().
bool rh_init(void);

// ---- Mesh send / recv ----
// Returns RH_ROUTER_ERROR_* codes (0 = success).
uint8_t rh_mesh_send(uint8_t* buf, uint8_t len, uint8_t dest, uint8_t flags);

// Non-blocking receive.  Returns true if a message was delivered to buf.
bool rh_mesh_recv(uint8_t* buf, uint8_t* len,
                  uint8_t* source, uint8_t* dest,
                  uint8_t* id, uint8_t* flags);

// Blocking receive with timeout (ms).
bool rh_mesh_recv_timeout(uint8_t* buf, uint8_t* len, uint16_t timeout,
                          uint8_t* source, uint8_t* dest,
                          uint8_t* id, uint8_t* flags);

// ---- Reliable datagram (point-to-point, no routing) ----
bool rh_reliable_send(uint8_t* buf, uint8_t len, uint8_t address);
bool rh_reliable_recv(uint8_t* buf, uint8_t* len,
                      uint8_t* from, uint8_t* to,
                      uint8_t* id, uint8_t* flags);

// ---- Configuration ----
void     rh_set_this_address(uint8_t address);
void     rh_reliable_set_timeout(uint16_t timeout_ms);
void     rh_reliable_set_retries(uint8_t retries);
void     rh_router_add_route(uint8_t dest, uint8_t next_hop);
bool     rh_router_delete_route(uint8_t dest);
void     rh_router_clear_routes(void);
void     rh_router_set_max_hops(uint8_t max_hops);

// ---- Statistics ----
uint16_t rh_driver_rx_good(void);
uint16_t rh_driver_rx_bad(void);
uint16_t rh_driver_tx_good(void);
int16_t  rh_driver_last_rssi(void);
uint32_t rh_reliable_retransmissions(void);

#ifdef __cplusplus
}
#endif

#endif // RH_SHIM_H
