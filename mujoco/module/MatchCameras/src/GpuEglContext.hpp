#ifndef K1SIM_MODULE_MATCHCAMERAS_GPUEGLCONTEXT_HPP
#define K1SIM_MODULE_MATCHCAMERAS_GPUEGLCONTEXT_HPP

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <array>
#include <cstring>

namespace k1sim::module::matchcameras {

    // An offscreen OpenGL context on a GPU, found through EGL's device platform. EGL_DEFAULT_DISPLAY
    // goes through the display server when DISPLAY is set, which under Wayland means rendering on the
    // CPU through Xwayland: six cameras fell from 15 fps to 0.5 fps. Going to the device directly
    // doesn't depend on the desktop.
    class GpuEglContext {
    public:
        GpuEglContext(int width, int height) {
            const auto query_devices =
                reinterpret_cast<PFNEGLQUERYDEVICESEXTPROC>(eglGetProcAddress("eglQueryDevicesEXT"));
            const auto query_device_string =
                reinterpret_cast<PFNEGLQUERYDEVICESTRINGEXTPROC>(eglGetProcAddress("eglQueryDeviceStringEXT"));
            const auto platform_display =
                reinterpret_cast<PFNEGLGETPLATFORMDISPLAYEXTPROC>(eglGetProcAddress("eglGetPlatformDisplayEXT"));
            if (query_devices == nullptr || query_device_string == nullptr || platform_display == nullptr) {
                return;
            }

            std::array<EGLDeviceEXT, 8> devices{};
            EGLint count = 0;
            if (query_devices(static_cast<EGLint>(devices.size()), devices.data(), &count) == EGL_FALSE) {
                return;
            }
            for (EGLint i = 0; i < count && !valid_; ++i) {
                const char* extensions = query_device_string(devices[i], EGL_EXTENSIONS);
                if (extensions != nullptr && std::strstr(extensions, "EGL_MESA_device_software") != nullptr) {
                    continue;
                }
                display_ = platform_display(EGL_PLATFORM_DEVICE_EXT, devices[i], nullptr);
                if (display_ != EGL_NO_DISPLAY) {
                    valid_ = create(width, height);
                }
            }
        }

        ~GpuEglContext() {
            if (display_ == EGL_NO_DISPLAY) {
                return;
            }
            eglMakeCurrent(display_, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
            if (context_ != EGL_NO_CONTEXT) {
                eglDestroyContext(display_, context_);
            }
            if (surface_ != EGL_NO_SURFACE) {
                eglDestroySurface(display_, surface_);
            }
            eglTerminate(display_);
        }

        GpuEglContext(const GpuEglContext&)            = delete;
        GpuEglContext& operator=(const GpuEglContext&) = delete;

        bool valid() const {
            return valid_;
        }

    private:
        bool create(int width, int height) {
            EGLint major = 0;
            EGLint minor = 0;
            if (eglInitialize(display_, &major, &minor) == EGL_FALSE) {
                return false;
            }
            const EGLint config_attributes[] = {EGL_SURFACE_TYPE,
                                                EGL_PBUFFER_BIT,
                                                EGL_RED_SIZE,
                                                8,
                                                EGL_GREEN_SIZE,
                                                8,
                                                EGL_BLUE_SIZE,
                                                8,
                                                EGL_DEPTH_SIZE,
                                                24,
                                                EGL_RENDERABLE_TYPE,
                                                EGL_OPENGL_BIT,
                                                EGL_NONE};
            EGLConfig config                 = nullptr;
            EGLint configs                   = 0;
            if (eglChooseConfig(display_, config_attributes, &config, 1, &configs) == EGL_FALSE || configs < 1) {
                return false;
            }
            const EGLint surface_attributes[] = {EGL_WIDTH, width, EGL_HEIGHT, height, EGL_NONE};
            surface_                          = eglCreatePbufferSurface(display_, config, surface_attributes);
            if (surface_ == EGL_NO_SURFACE || eglBindAPI(EGL_OPENGL_API) == EGL_FALSE) {
                return false;
            }
            context_ = eglCreateContext(display_, config, EGL_NO_CONTEXT, nullptr);
            return context_ != EGL_NO_CONTEXT && eglMakeCurrent(display_, surface_, surface_, context_) == EGL_TRUE;
        }

        EGLDisplay display_ = EGL_NO_DISPLAY;
        EGLSurface surface_ = EGL_NO_SURFACE;
        EGLContext context_ = EGL_NO_CONTEXT;
        bool valid_         = false;
    };

}  // namespace k1sim::module::matchcameras

#endif  // K1SIM_MODULE_MATCHCAMERAS_GPUEGLCONTEXT_HPP
