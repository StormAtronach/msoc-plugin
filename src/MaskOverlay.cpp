// Debug occlusion-mask overlay + PFM dump. See MaskOverlay.h for the why.

#include "MaskOverlay.h"

#include "OcclusionInternal.h"
#include "Log.h"

#include "NIPixelData.h"
#include "NIPointer.h"
#include "NISourceTexture.h"

#include <cerrno>
#include <cfloat>
#include <cstdio>
#include <ostream>
#include <vector>

namespace msoc::occlusion {

namespace {

// Created on first maskOverlayTexture() call and held for the process. The
// NI::Pointer pins keep the engine from freeing either object underneath the
// UI element that samples it.
NI::Pointer<NI::PixelData> g_pixelData;
NI::Pointer<NI::SourceTexture> g_texture;

// Readback scratch, sized once. Reused by both consumers.
std::vector<float> g_depth;

struct ToneMapStats {
    int occluderPixels;
    float rawMin;
    float rawMax;
};

// Read the completed mask into g_depth and tone-map it in place to [0, 1].
// MOC writes -1.0 into tiles nothing rasterized into and a small positive 1/w
// for occluder depth, so a naive auto-stretch collapses both to black.
// Unwritten becomes 0; occluder depth spreads over [0.2, 1.0] with the nearest
// surface brightest. Raw extents come back in the stats so the log line can
// report real numbers.
//
// flipY picks scanline order. MOC writes top-to-bottom under USE_D3D, which is
// what a D3D texture wants (false); PFM is bottom-up (true).
ToneMapStats readAndToneMap(bool flipY) {
    const size_t count = static_cast<size_t>(kMsocWidth) * kMsocHeight;
    g_depth.assign(count, 0.0f);
    g_msoc->ComputePixelDepthBuffer(g_depth.data(), flipY);

    ToneMapStats s{0, FLT_MAX, -FLT_MAX};
    for (const float v : g_depth) {
        if (v > 0.0f) {
            ++s.occluderPixels;
            if (v < s.rawMin) s.rawMin = v;
            if (v > s.rawMax) s.rawMax = v;
        }
    }
    if (s.occluderPixels == 0) {
        s.rawMin = 0.0f;
        s.rawMax = 0.0f;
        return s;  // g_depth is already all-zero from the assign above
    }

    const float range = (s.rawMax > s.rawMin) ? (s.rawMax - s.rawMin) : 1.0f;
    for (float& v : g_depth) {
        v = (v <= 0.0f) ? 0.0f : 0.2f + 0.8f * ((v - s.rawMin) / range);
    }
    return s;
}

}  // namespace

void* maskOverlayTexture() {
    if (g_texture) {
        return g_texture.get();
    }
    // Resources are allocated lazily (EnableMSOC toggle) and the mask size is
    // latched at install; without them there is nothing to size the texture to.
    if (!g_msoc) {
        return nullptr;
    }

    g_pixelData = NI::PixelData::create(kMsocWidth, kMsocHeight);
    if (!g_pixelData) {
        log::getLog() << "MSOC overlay: NI::PixelData::create failed; overlay unavailable." << std::endl;
        return nullptr;
    }
    // create() asks for RGBA32. Anything else means the engine handed back a
    // format this writer does not understand, so refuse rather than scribble.
    if (g_pixelData->bytesPerPixel != 4) {
        log::getLog() << "MSOC overlay: unexpected bytesPerPixel="
                      << g_pixelData->bytesPerPixel << "; overlay unavailable." << std::endl;
        g_pixelData = nullptr;
        return nullptr;
    }

    // Explicit format preferences rather than createSourceTexture()'s defaults.
    // The engine's default prefs are PIX_DEFAULT / MIP_DEFAULT / ALPHA_DEFAULT,
    // which let the converter choose; a mask written as opaque RGBA32 wants a
    // plain 32-bit true-colour target with no mip chain and no alpha, so the UI
    // cannot end up alpha-blending the whole panel away.
    NI::Texture::FormatPrefs prefs;
    prefs.pixelLayout = NI::Texture::FormatPrefs::PixelLayout::TRUE_COLOR_32;
    prefs.mipMapped = NI::Texture::FormatPrefs::MipFlag::NO;
    prefs.alphaFormat = NI::Texture::FormatPrefs::AlphaFormat::NONE;
    g_texture = NI::SourceTexture::createFromPixelData(g_pixelData.get(), &prefs);
    if (!g_texture) {
        log::getLog() << "MSOC overlay: createSourceTexture failed; overlay unavailable." << std::endl;
        g_pixelData = nullptr;
        return nullptr;
    }

    // NiSourceTexture_static::CreateFromPixelData sets isStatic = 1, which tells
    // the renderer the pixels never change and lets it treat the upload as
    // one-shot. Every live-updating texture in the wild clears it: Weather
    // Adjuster (hrnchamd, who also wrote MGE) does exactly
    //   niPixelData.new(256, 4):createSourceTexture()  then  .isStatic = false
    // before mutating the pixel data each frame. Without this the panel renders
    // black no matter how correct the pixel data and the revision bump are.
    g_texture->isStatic = false;

    log::getLog() << "MSOC overlay: texture created at " << kMsocWidth << "x" << kMsocHeight
                  << " (isStatic=false)." << std::endl;
    return g_texture.get();
}

void updateMaskOverlay() {
    if (!g_texture || !g_pixelData || !g_msoc) {
        return;
    }

    readAndToneMap(/*flipY*/ false);

    // Grey, opaque. Channel order is irrelevant while R == G == B, so this
    // writer does not care whether the engine's RGBA32 is RGBA or BGRA.
    unsigned char* dst = g_pixelData->pixels + g_pixelData->offsets[0];
    const size_t count = g_depth.size();
    for (size_t i = 0; i < count; ++i) {
        const auto grey = static_cast<unsigned char>(g_depth[i] * 255.0f + 0.5f);
        dst[i * 4 + 0] = grey;
        dst[i * 4 + 1] = grey;
        dst[i * 4 + 2] = grey;
        dst[i * 4 + 3] = 255;
    }

    // NiDX8SourceTextureData::Update re-converts and re-uploads exactly when
    // pixelData->revisionID differs from the copy it cached, so bump it first.
    ++g_pixelData->revisionID;

    // ...but nothing in Morrowind polls that on a texture bind. Verified
    // against Morrowind.exe: the only reference to Update (0x6C0050) is its own
    // vtable slot, and no call site anywhere loads Texture::rendererData and
    // dispatches +0x2C. MWSE's niPixelData:setPixelsByte() bumps the revision
    // and stops there, which is why a texture mutated that way can sit stale on
    // screen. So drive the upload ourselves.
    //
    // rendererData stays null until the renderer first binds the texture; that
    // first bind uploads whatever the pixels hold at the time, which is already
    // this frame's mask, so skipping here costs nothing.
    if (auto* rendererData = g_texture->rendererData) {
        // Slot 11 (+0x2C) of vtbl_sg_NiDX8SourceTextureData (0x74F770).
        constexpr int kUpdateSlot = 11;
        using UpdateFn = void(__thiscall*)(void*);
        auto** vtable = *reinterpret_cast<void***>(rendererData);
        reinterpret_cast<UpdateFn>(vtable[kUpdateSlot])(rendererData);
    }
}

void maskResolution(int* outWidth, int* outHeight) {
    if (outWidth) *outWidth = static_cast<int>(kMsocWidth);
    if (outHeight) *outHeight = static_cast<int>(kMsocHeight);
}

bool dumpMaskToPfm(const char* path) {
    if (path == nullptr) {
        log::getLog() << "MSOC dump: null path" << std::endl;
        return false;
    }
    if (!g_msoc) {
        log::getLog() << "MSOC dump: mask resources not live" << std::endl;
        return false;
    }

    const ToneMapStats s = readAndToneMap(/*flipY*/ true);

    FILE* f = std::fopen(path, "wb");
    if (!f) {
        log::getLog() << "MSOC dump: fopen failed for '" << path
                      << "' errno=" << errno << std::endl;
        return false;
    }
    std::fprintf(f, "Pf\n%u %u\n-1.0\n", kMsocWidth, kMsocHeight);
    std::fwrite(g_depth.data(), sizeof(float), g_depth.size(), f);
    std::fclose(f);

    log::getLog() << "MSOC: occlusion mask dumped to " << path
                  << " (" << kMsocWidth << "x" << kMsocHeight
                  << ", occluderPx=" << s.occluderPixels
                  << ", rawRange=[" << s.rawMin << ".." << s.rawMax << "])" << std::endl;
    return true;
}

}  // namespace msoc::occlusion
