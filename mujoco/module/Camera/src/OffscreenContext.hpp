#ifndef K1SIM_MODULE_CAMERA_OFFSCREENCONTEXT_HPP
#define K1SIM_MODULE_CAMERA_OFFSCREENCONTEXT_HPP

#if defined(__APPLE__)
    #include <OpenGL/OpenGL.h>

namespace k1sim::module::camera {

    // Native offscreen context: no Cocoa window or main-thread dependency.
    // MuJoCo provides its own framebuffer, so no CGL pbuffer is necessary.
    class OffscreenContext {
    public:
        OffscreenContext(int /*width*/, int /*height*/) {
            const CGLPixelFormatAttribute attributes[] = {kCGLPFAAccelerated,
                                                          kCGLPFAOpenGLProfile,
                                                          static_cast<CGLPixelFormatAttribute>(kCGLOGLPVersion_Legacy),
                                                          static_cast<CGLPixelFormatAttribute>(0)};
            CGLPixelFormatObj format                   = nullptr;
            GLint count                                = 0;
            if (CGLChoosePixelFormat(attributes, &format, &count) != kCGLNoError || format == nullptr) {
                return;
            }
            const CGLError result = CGLCreateContext(format, nullptr, &context_);
            CGLDestroyPixelFormat(format);
            valid_ = result == kCGLNoError && CGLSetCurrentContext(context_) == kCGLNoError;
        }

        ~OffscreenContext() {
            if (context_ != nullptr) {
                CGLSetCurrentContext(nullptr);
                CGLDestroyContext(context_);
            }
        }

        bool valid() const {
            return valid_;
        }

        OffscreenContext(const OffscreenContext&)            = delete;
        OffscreenContext& operator=(const OffscreenContext&) = delete;

    private:
        CGLContextObj context_ = nullptr;
        bool valid_            = false;
    };

}  // namespace k1sim::module::camera
#else
    #include "module/Camera/src/EglContext.hpp"

namespace k1sim::module::camera {
    using OffscreenContext = EglContext;
}
#endif

#endif  // K1SIM_MODULE_CAMERA_OFFSCREENCONTEXT_HPP
