// Engine-method shim. MWSE's NI*.h / TES3*.h declare methods whose bodies
// live in MWSE .cpp files that drag in the Lua binding layer. Local
// definitions here do what upstream does, minus the Lua tangle. Mirrors
// UI Expansion's plugin_source/MWSEImports.cpp.
//
// Scope: only methods PatchOcclusionCulling.cpp references. Every
// reinterpret_cast<sig>(0xADDR) is copy-paste from the matching upstream
// .cpp in deps/mwse-upstream/MWSE/. Keep address + sig in sync if bumping
// the submodule.

#include "stdafx.h"

#include "MemoryUtil.h"

#include "NIAVObject.h"
#include "NIGeometryData.h"
#include "NIObject.h"
#include "NIProperty.h"
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

    void* Object::operator new(size_t size) { return mwse::tes3::_new(size); }
    void Object::operator delete(void* address) { mwse::tes3::_delete(address); }

    // Property::Property() in this TU generates an implicit call to the
    // Object base ctor, so this needs to be linkable.
    Object::Object() {
        reinterpret_cast<Object*(__thiscall*)(Object*)>(0x6E98A0)(this);
    }
    Object::~Object() {
        reinterpret_cast<Object*(__thiscall*)(Object*)>(0x6E98F0)(this);
    }

    Object* Object::createClone() {
        return reinterpret_cast<Object*(__thiscall*)(Object*)>(0x6E9910)(this);
    }

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

    Property::~Property() {
        reinterpret_cast<void(__thiscall*)(Property*)>(0x405B40)(this);
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
        // Wraps NI_PropertyList_addHead — Pointer<Property> ctor increments refCount.
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

    Pointer<TriBasedGeometryData> TriBasedGeometry::getModelData() const {
        return static_cast<TriBasedGeometryData*>(modelData.get());
    }

}

namespace TES3 {

    // Patch only constructs Vector3(x,y,z); other overloads (Color&,
    // sol::table, sol::object) are intentionally not linked in.
    Vector3::Vector3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}

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
