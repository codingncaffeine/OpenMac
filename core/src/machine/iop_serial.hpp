#pragma once

// The serial chip behind an I/O processor (Macintosh IIfx, Quadra 900 and
// 950): the SCC's two /W/REQ outputs are that processor's two byte-DMA
// request lines. A line is up while the firmware has its channel enabled and
// the SCC port the channel points at can take or give a byte, so a transmit
// burst moves one byte per free holding register and cannot overrun it.
// Clean-room: the processor's DMA control bits as its downloaded firmware
// uses them, the request outputs as the Z8530's data sheet describes them.

#include "openmac/types.hpp"

#include "iop.hpp"
#include "quadra/scc.hpp"

namespace openmac {

// Call when the SCC's readiness changes, when the firmware writes a DMA
// control register, and before the processor runs.
inline void updateSerialDmaRequests(IoProcessor& iop, const Scc8530& scc) {
    for (int channel = 0; channel < 2; ++channel) {
        const u8 control = iop.dmaControl(channel);
        const bool enabled = (control & 0x01u) != 0;
        const bool peripheralToRam = (control & 0x04u) != 0;
        const u8 peripheral = static_cast<u8>(control >> 4);
        iop.setDmaRequest(channel, enabled && scc.dmaRequest(peripheral, peripheralToRam));
    }
}

} // namespace openmac
