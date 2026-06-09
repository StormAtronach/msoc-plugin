#include "DebugTint.h"

#include "MemoryUtil.h"
#include "NIAVObject.h"
#include "NIColor.h"
#include "NINode.h"
#include "NIObject.h"
#include "NIPointer.h"
#include "NIProperty.h"
#include "NIRTTI.h"

#include <cstdint>
#include <unordered_map>

namespace msoc::debugtint {
namespace {

// Tint hues. Persistent-clone lifetime: first tint on a shape clones the
// effective MaterialProperty, detaches any own material, attaches the clone.
// Subsequent frames only overwrite emissive + bump revisionID. Avoids the
// per-frame allocator churn that previously crashed the DX8 per-material state
// cache (~1.7k new/delete per frame; recycled addresses returned dangling D3D
// state).
//
// Morrowind shares NiMaterialProperty via inheritance (parent NiNode material
// covers descendants without their own) AND via NIF streamable dedup. Cloning
// avoids both stains. Only NiTriBasedGeom leaves tint - a NiNode tint would
// propagate to its whole subtree.
const NI::Color kTintOccluded(1.0f, 0.0f, 0.0f);
const NI::Color kTintTested(0.0f, 1.0f, 0.0f);
const NI::Color kTintOccluder(1.0f, 1.0f, 0.0f);

// NiMaterialProperty vtable. MWSE exposes no public ctor; we open-code the
// engine pattern (Property::Property sets base vtable; concrete subclasses
// overwrite).
constexpr uintptr_t kNiMaterialPropertyVTable = 0x75036C;

struct TintClone {
    // Refcounts the shape so map key + attached clone stay valid across cell
    // unload.
    NI::Pointer<NI::AVObject> shape;
    // Refcounts the source so end-of-frame reset can read its current
    // emissive. For own-material shapes we detached it; this Pointer is now
    // the sole owner.
    NI::Pointer<NI::MaterialProperty> source;
    // Owned by the shape's property list (attachProperty refs it). Safe
    // raw-ptr because `shape` keeps the list alive.
    NI::MaterialProperty* clone;
};
std::unordered_map<NI::AVObject*, TintClone> g_tintClones;

// Walk obj -> ancestors, return first MaterialProperty (engine's
// property-inheritance rule).
NI::MaterialProperty* findInheritedMaterial(NI::AVObject* obj) {
    for (NI::AVObject* cur = obj; cur; cur = cur->parentNode) {
        for (auto* node = &cur->propertyNode; node && node->data; node = node->next) {
            if (node->data->getType() == NI::PropertyType::Material) {
                return static_cast<NI::MaterialProperty*>(node->data);
            }
        }
    }
    return nullptr;
}

// Standalone NiMaterialProperty seeded from source (if any) with the tint
// forced into ambient + diffuse + emissive. All three channels: a sibling
// NiVertexColorProperty may replace one of them from baked per-vertex colors
// (SOURCE_IGNORE / SOURCE_EMISSIVE / SOURCE_AMBIENT_DIFFUSE). World statics use
// SOURCE_EMISSIVE; skinned actors use IGNORE/AMBIENT_DIFFUSE - tinting all
// three covers both.
NI::MaterialProperty* cloneMaterialProperty(NI::MaterialProperty* source, const NI::Color& tint) {
    // Returns refCount=1 (caller-owned, matching NiObject::Clone).
    // attachProperty's wrapper at 0x405840 has a Pointer cycle around AddHead
    // that net-zeros the input refcount; refCount=0 would 0->1->0 -> DELETE,
    // leaving the property list dangling.
    NI::MaterialProperty* mat;
    if (source) {
        // Engine's NiObject::Clone (0x6E9910): stream-based deep copy, returns
        // refCount=1. Preferred - picks up every inherited field, even those
        // we don't enumerate below.
        mat = static_cast<NI::MaterialProperty*>(source->createClone());
    }
    else {
        // Fresh allocation. NiObject::ctor sets refCount=0; bump to match the
        // createClone path.
        mat = new NI::MaterialProperty();
        mat->refCount = 1;
        mat->vTable.asProperty = reinterpret_cast<NI::Property_vTable*>(kNiMaterialPropertyVTable);
        mat->flags = 1;
        mat->index = 0;
        mat->specular = NI::Color(0.0f, 0.0f, 0.0f);
        mat->shininess = 10.0f;
        mat->alpha = 1.0f;
    }
    mat->ambient = tint;
    mat->diffuse = tint;
    mat->emissive = tint;
    mat->revisionID = 0;
    return mat;
}

void tintEmissive(NI::AVObject* shape, const NI::Color& color) {
    // Only leaves: tinting a NiNode would stain every inheriting child.
    if (!shape->isInstanceOfType(NI::RTTIStaticPtr::NiTriBasedGeom)) {
        return;
    }

    // Subsequent tint on an already-cloned shape: overwrite emissive only. No
    // allocations, no property-list churn.
    auto it = g_tintClones.find(shape);
    if (it != g_tintClones.end()) {
        it->second.clone->emissive = color;
        it->second.clone->incrementRevisionId();
        return;
    }

    // First tint: clone the effective material, detach any own material,
    // attach the clone. findInheritedMaterial walks from shape upward so it
    // returns the own material when present, falling back to an ancestor's.
    NI::MaterialProperty* source = findInheritedMaterial(shape);
    NI::MaterialProperty* clone = cloneMaterialProperty(source, color);

    // Detach returns null if the shape had no own material (pure inheritance).
    // Either way our clone becomes the leading (and only) Material property on
    // the shape.
    NI::Pointer<NI::Property> detachedOwn = shape->detachPropertyByType(NI::PropertyType::Material);
    shape->attachProperty(clone);
    // attachProperty mutates the raw list only; the renderer reads from a
    // per-Geometry effective-material cache that updateProperties() rebuilds.
    // Without this, draws keep using the cached material until something else
    // updates properties.
    shape->updateProperties();

    // Keep the source alive. If the own material was detached, detachedOwn
    // holds the sole ref now - promote it to our map entry. Otherwise source
    // points into an ancestor's property list; take a fresh Pointer.
    NI::Pointer<NI::MaterialProperty> sourcePtr;
    if (detachedOwn) {
        sourcePtr = static_cast<NI::MaterialProperty*>(detachedOwn.get());
    }
    else {
        sourcePtr = source;
    }

    g_tintClones.emplace(shape, TintClone{ shape, sourcePtr, clone });
}

} // namespace

void tintOccluded(NI::AVObject* shape) { tintEmissive(shape, kTintOccluded); }
void tintTested(NI::AVObject* shape) { tintEmissive(shape, kTintTested); }
void tintOccluder(NI::AVObject* shape) { tintEmissive(shape, kTintOccluder); }

// End-of-frame tint reset: restore each tracked source's ambient + diffuse +
// emissive into its clone (all three channels - tintEmissive sets all three to
// handle NiVertexColorProperty SOURCE_*). Pointer-stable; updateProperties() is
// required for the DX8 effective-material cache to pick up the new colors.
void resetFrameTints() {
    const NI::Color zero(0.0f, 0.0f, 0.0f);
    for (auto& [key, entry] : g_tintClones) {
        if (entry.source) {
            entry.clone->ambient = entry.source->ambient;
            entry.clone->diffuse = entry.source->diffuse;
            entry.clone->emissive = entry.source->emissive;
        }
        else {
            entry.clone->ambient = zero;
            entry.clone->diffuse = zero;
            entry.clone->emissive = zero;
        }
        entry.clone->incrementRevisionId();
        entry.shape->updateProperties();
    }
}

void clearClones() {
    g_tintClones.clear();
}

} // namespace msoc::debugtint
