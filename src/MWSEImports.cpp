// _Claude_ Engine-method shim.
//
// MWSE's NI*.h / TES3*.h declare a lot of methods whose bodies live in
// MWSE's own .cpp files. Most of those .cpp files drag in MWSE's Lua
// binding layer (sol bindings, getOrCreateLuaObject, LuaManager) — we
// can't compile them without restoring all the MWSE runtime we just
// tore out. The portable alternative: local definitions here that do
// what upstream does, minus the Lua tangle. Mirrors UI Expansion's
// plugin_source/MWSEImports.cpp.
//
// Scope: ONLY the methods PatchOcclusionCulling.cpp references. If a
// future addition hits "unresolved external symbol", add the forwarder
// here. Do not pre-emptively stub methods we don't call.
//
// Every reinterpret_cast<sig>(0xADDR) is copy-paste from the matching
// upstream .cpp in deps/mwse-upstream/MWSE/. Keep the address + sig in
// sync with upstream if bumping the submodule.

#include "stdafx.h"

#include "MemoryUtil.h"

#include "NIAVObject.h"
#include "NIGeometryData.h"
#include "NIObject.h"
#include "NIProperty.h"           // declares both Property and MaterialProperty
#include "NIRTTI.h"
#include "NiTriBasedGeometryData.h"
#include "NITriShape.h"
#include "NiTriShapeData.h"

#include "TES3Cell.h"
#include "TES3DataHandler.h"
#include "TES3Defines.h"
#include "TES3Vectors.h"
#include "TES3WorldController.h"


namespace NI {

    // Heap — engine global allocator, surfaced via MWSE's MemoryUtil wrappers.
    void* Object::operator new(size_t size) { return mwse::tes3::_new(size); }
    void Object::operator delete(void* address) { mwse::tes3::_delete(address); }

    // Base ctor/dtor — needed because Property::Property() in this TU
    // generates an implicit call to the Object base subobject. Upstream
    // addresses: 0x6E98A0 (ctor), 0x6E98F0 (dtor).
    Object::Object() {
        reinterpret_cast<Object*(__thiscall*)(Object*)>(0x6E98A0)(this);
    }
    Object::~Object() {
        reinterpret_cast<Object*(__thiscall*)(Object*)>(0x6E98F0)(this);
    }

    Object* Object::createClone() {
        return reinterpret_cast<Object*(__thiscall*)(Object*)>(0x6E9910)(this);
    }

    // RTTI chain walk — all pure member access through the vtable, no
    // external symbols. Upstream body copied verbatim from NIObject.cpp.
    bool Object::isInstanceOfType(const RTTI* type) const {
        for (const RTTI* rtti = vTable.asObject->getRTTI(this); rtti; rtti = rtti->baseRTTI) {
            if (rtti == type) {
                return true;
            }
        }
        return false;
    }

    Property::Property() {
        reinterpret_cast<Property*(__thiscall*)(Property*)>(0x405990)(this);
    }

    PropertyType Property::getType() const {
        return static_cast<PropertyType>(vTable.asProperty->getType(this));
    }

    void MaterialProperty::incrementRevisionId() {
        revisionID++;
    }

    bool AVObject::getAppCulled() const {
        return vTable.asAVObject->getAppCulled(this);
    }

    void AVObject::updateProperties() {
        reinterpret_cast<void(__thiscall*)(AVObject*)>(0x6EB0E0)(this);
    }

    void AVObject::attachProperty(Property* property) {
        // Upstream wraps this through NI_PropertyList_addHead at 0x405840,
        // taking the raw Property* as a Pointer<Property> (implicit ctor
        // increments refCount).
        reinterpret_cast<void(__thiscall*)(PropertyLinkedList*, Pointer<Property>)>(0x405840)(
            &propertyNode, Pointer<Property>(property));
    }

    Pointer<Property> AVObject::detachPropertyByType(PropertyType type) {
        Pointer<Property> prop;
        reinterpret_cast<Pointer<Property>*(__thiscall*)(AVObject*, Pointer<Property>*, PropertyType)>(0x6EAE20)(
            this, &prop, type);
        return prop;
    }

    unsigned short GeometryData::getActiveVertexCount() const {
        return vTable.asGeometryData->getActiveVertexCount(this);
    }

    Triangle* TriBasedGeometryData::getTriList() {
        return vTable.asTriBasedGeometryData->getTriList(this);
    }

    unsigned short TriBasedGeometryData::getActiveTriangleCount() const {
        return vTable.asTriBasedGeometryData->getActiveTriangleCount(this);
    }

    Pointer<TriShapeData> TriShape::getModelData() const {
        return static_cast<TriShapeData*>(modelData.get());
    }

}

namespace TES3 {

    // Single ctor overload — patch only constructs Vector3(x,y,z). Other
    // overloads (Color&, sol::table, sol::object) are not linked in.
    Vector3::Vector3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}

    // Inline getCellFlag's body here rather than defining getCellFlag
    // separately — patch only calls getIsInterior, and adding
    // getCellFlag would introduce another unresolved symbol to link.
    bool Cell::getIsInterior() const {
        return (cellFlags & TES3::CellFlag::Interior) != 0;
    }

    // Engine globals — singleton pointers the game writes at init.
    DataHandler* DataHandler::get() {
        return *reinterpret_cast<TES3::DataHandler**>(0x7C67E0);
    }

    WorldController* WorldController::get() {
        return *reinterpret_cast<TES3::WorldController**>(0x7C67DC);
    }

}
