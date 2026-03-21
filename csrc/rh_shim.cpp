// RustDriver: RHGenericDriver subclass that delegates radio I/O to Rust
// via function pointers, plus C shim functions for the routing stack.

#include "rh_shim.h"
#include <RHGenericDriver.h>
#include <RHDatagram.h>
#include <RHReliableDatagram.h>
#include <RHRouter.h>
#include <RHMesh.h>

// Placement new for bare-metal (no <new> header available)
inline void* operator new(decltype(sizeof(0)), void* p) noexcept { return p; }

// ======================================================================
// RustDriver – bridges RadioHead's virtual interface to Rust callbacks
// ======================================================================
class RustDriver : public RHGenericDriver {
public:
    RustDriver() : _vtable(nullptr), _ctx(nullptr), _rxBufLen(0), _rxBufValid(false) {}

    void configure(const RhDriverVtable* vtable, void* ctx) {
        _vtable = vtable;
        _ctx    = ctx;
    }

    // -- Pure virtuals ------------------------------------------------

    bool available() override {
        if (_rxBufValid)
            return true;
        if (_mode == RHModeTx)
            return false;

        // Poll the Rust driver for a received packet
        uint8_t tmpLen = (uint8_t)(sizeof(_rxBuf) > 255 ? 255 : sizeof(_rxBuf));
        int16_t rssi   = 0;
        if (!_vtable->poll_recv(_ctx, _rxBuf, &tmpLen, &rssi) || tmpLen < 4)
            return false;

        _rxBufLen = tmpLen;
        _lastRssi = rssi;

        // Extract RadioHead 4-byte header
        _rxHeaderTo    = _rxBuf[0];
        _rxHeaderFrom  = _rxBuf[1];
        _rxHeaderId    = _rxBuf[2];
        _rxHeaderFlags = _rxBuf[3];

        // Address filter
        if (_promiscuous ||
            _rxHeaderTo == _thisAddress ||
            _rxHeaderTo == RH_BROADCAST_ADDRESS)
        {
            _rxBufValid = true;
            _rxGood++;
            _mode = RHModeIdle;
            return true;
        }

        _rxBad++;
        return false;
    }

    bool recv(uint8_t* buf, uint8_t* len) override {
        if (!_rxBufValid)
            return false;

        uint8_t payloadLen = _rxBufLen - 4;
        if (*len > payloadLen)
            *len = payloadLen;
        memcpy(buf, _rxBuf + 4, *len);

        _rxBufValid = false;
        return true;
    }

    bool send(const uint8_t* data, uint8_t len) override {
        if (len > maxMessageLength())
            return false;

        waitPacketSent();  // wait for any previous TX

        // Build over-the-air packet: [TO, FROM, ID, FLAGS, ...payload...]
        uint8_t txBuf[RH_MAX_MESSAGE_LEN + 4];
        txBuf[0] = _txHeaderTo;
        txBuf[1] = _txHeaderFrom;
        txBuf[2] = _txHeaderId;
        txBuf[3] = _txHeaderFlags;
        memcpy(txBuf + 4, data, len);

        _mode = RHModeTx;
        bool ok = _vtable->send(_ctx, txBuf, len + 4);
        _mode = RHModeIdle;

        if (ok)
            _txGood++;
        return ok;
    }

    uint8_t maxMessageLength() override {
        // Subtract the 4-byte RadioHead header from the raw radio capacity
        uint8_t raw = _vtable->max_message_length(_ctx);
        return (raw >= 4) ? (raw - 4) : 0;
    }

    // -- Optional overrides -------------------------------------------

    bool init() override {
        _mode = RHModeIdle;
        return true;
    }

private:
    const RhDriverVtable* _vtable;
    void*                 _ctx;
    uint8_t               _rxBuf[RH_MAX_MESSAGE_LEN + 4];
    uint8_t               _rxBufLen;
    bool                  _rxBufValid;
};

// ======================================================================
// Static storage for the singleton instances
// ======================================================================
static uint8_t _driver_storage[sizeof(RustDriver)]  __attribute__((aligned(8)));
static uint8_t _mesh_storage[sizeof(RHMesh)]         __attribute__((aligned(8)));

static RustDriver* g_driver = nullptr;
static RHMesh*     g_mesh   = nullptr;

// ======================================================================
// C shim functions
// ======================================================================

extern "C" {

// Bare-metal C++ runtime support
void __cxa_pure_virtual() { while (1); }

// ---- Lifecycle ------------------------------------------------------

void rh_create(const RhDriverVtable* vtable, void* ctx, uint8_t this_address) {
    // Construct in static storage (no heap needed)
    g_driver = new (_driver_storage) RustDriver();
    g_driver->configure(vtable, ctx);

    g_mesh = new (_mesh_storage) RHMesh(*g_driver, this_address);
}

bool rh_init(void) {
    if (!g_mesh) return false;
    return g_mesh->init();
}

// ---- Mesh -----------------------------------------------------------

uint8_t rh_mesh_send(uint8_t* buf, uint8_t len, uint8_t dest, uint8_t flags) {
    return g_mesh->sendtoWait(buf, len, dest, flags);
}

bool rh_mesh_recv(uint8_t* buf, uint8_t* len,
                  uint8_t* source, uint8_t* dest,
                  uint8_t* id, uint8_t* flags)
{
    return g_mesh->recvfromAck(buf, len, source, dest, id, flags);
}

bool rh_mesh_recv_timeout(uint8_t* buf, uint8_t* len, uint16_t timeout,
                          uint8_t* source, uint8_t* dest,
                          uint8_t* id, uint8_t* flags)
{
    return g_mesh->recvfromAckTimeout(buf, len, timeout, source, dest, id, flags);
}

// ---- Reliable datagram (bypasses mesh/routing, uses RHReliableDatagram) --

bool rh_reliable_send(uint8_t* buf, uint8_t len, uint8_t address) {
    // RHMesh inherits RHReliableDatagram::sendtoWait through RHRouter
    // We call RHReliableDatagram::sendtoWait directly (hop-to-hop only)
    return static_cast<RHReliableDatagram*>(g_mesh)->sendtoWait(buf, len, address);
}

bool rh_reliable_recv(uint8_t* buf, uint8_t* len,
                      uint8_t* from, uint8_t* to,
                      uint8_t* id, uint8_t* flags)
{
    return static_cast<RHReliableDatagram*>(g_mesh)->recvfromAck(buf, len, from, to, id, flags);
}

// ---- Configuration --------------------------------------------------

void rh_set_this_address(uint8_t address) {
    g_mesh->setThisAddress(address);
}

void rh_reliable_set_timeout(uint16_t timeout_ms) {
    g_mesh->setTimeout(timeout_ms);
}

void rh_reliable_set_retries(uint8_t retries) {
    g_mesh->setRetries(retries);
}

void rh_router_add_route(uint8_t dest, uint8_t next_hop) {
    g_mesh->addRouteTo(dest, next_hop);
}

bool rh_router_delete_route(uint8_t dest) {
    return g_mesh->deleteRouteTo(dest);
}

void rh_router_clear_routes(void) {
    g_mesh->clearRoutingTable();
}

void rh_router_set_max_hops(uint8_t max_hops) {
    g_mesh->setMaxHops(max_hops);
}

// ---- Statistics -----------------------------------------------------

uint16_t rh_driver_rx_good(void) { return g_driver->rxGood(); }
uint16_t rh_driver_rx_bad(void)  { return g_driver->rxBad(); }
uint16_t rh_driver_tx_good(void) { return g_driver->txGood(); }
int16_t  rh_driver_last_rssi(void) { return g_driver->lastRssi(); }

uint32_t rh_reliable_retransmissions(void) {
    return g_mesh->retransmissions();
}

} // extern "C"
