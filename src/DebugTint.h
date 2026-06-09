#pragma once

// Debug-only occlusion tint overlay, split out of PatchOcclusionCulling.cpp.
// Recolors classified NiTriBasedGeom leaves so the cull decision is visible
// in-engine: occluded (red), survived the query (green), rasterised as an
// occluder (yellow). Engine-coupled (NI material/property mutation), so it is
// NOT unit-tested - the in-game overlay is the verification.
//
// Lifetime: the first tint on a shape clones its effective MaterialProperty
// and attaches the clone (one allocation per shape, ever); later frames only
// overwrite the clone's emissive. resetFrameTints() restores source colors at
// frame end; clearClones() drops all clones (and their NI::Pointer pins) on
// cell change. See DebugTint.cpp for the refcount / vtable contract.

namespace NI { struct AVObject; }

namespace msoc::debugtint {

    // Tint a classified leaf. No-op on non-NiTriBasedGeom nodes - tinting a
    // NiNode would stain its whole inheriting subtree.
    void tintOccluded(NI::AVObject* shape);
    void tintTested(NI::AVObject* shape);
    void tintOccluder(NI::AVObject* shape);

    // End-of-frame: restore each tracked clone to its source material's colors
    // so an untinted frame renders normally. Call once per main-scene frame.
    void resetFrameTints();

    // Drop every clone (releases the NI::Pointer pins on shapes + sources).
    // Call on cell change, where the outgoing cell's shapes are torn down.
    void clearClones();

} // namespace msoc::debugtint
