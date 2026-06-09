// TES3 engine-global accessors. The NI:: methods the plugin calls are now
// compiled canonically from the SharedSE sources (file-globbed into
// SHAREDSE_SOURCES in CMakeLists.txt), so no NI forwarders live here anymore.
// SharedSE has no TES3 layer, so the few TES3 globals the patch needs are
// defined here instead.

#include "stdafx.h"

#include "TES3Cell.h"
#include "TES3DataHandler.h"
#include "TES3Defines.h"
#include "TES3WorldController.h"


namespace TES3 {

    // Inlined rather than introducing getCellFlag — patch only calls this.
    bool Cell::getIsInterior() const {
        return (cellFlags & TES3::CellFlag::Interior) != 0;
    }

    DataHandler* DataHandler::get() {
        return *reinterpret_cast<TES3::DataHandler**>(0x7C67E0);
    }

    WorldController* WorldController::get() {
        return *reinterpret_cast<TES3::WorldController**>(0x7C67DC);
    }

}
