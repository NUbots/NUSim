#ifndef K1SIM_MODULE_CAMERA_EGLCONTEXT_HPP
#define K1SIM_MODULE_CAMERA_EGLCONTEXT_HPP

#include <EGL/egl.h>

namespace k1sim::module::camera {

// RAII offscreen OpenGL context via EGL (pbuffer). Camera uses this instead of a
// hidden GLFW window so it shares NO global GLFW/Xlib state with module::Viewer's
// on-screen GLFW window — the two ran on different threads and concurrent GLFW/
// Xlib calls corrupted the heap ("malloc(): invalid size"). EGL also works with
// no display at all, so the camera can render headless. MuJoCo's mjr renders into
// whatever GL context is current, so making this current before mjr_makeContext is
// all that's needed (the MUJOCO_GL=egl pattern from MuJoCo's own examples).
class EglContext {
public:
    EglContext(int width, int height) {
        display_ = eglGetDisplay(EGL_DEFAULT_DISPLAY);
        if (display_ == EGL_NO_DISPLAY) {
            return;
        }
        EGLint major = 0;
        EGLint minor = 0;
        if (eglInitialize(display_, &major, &minor) == EGL_FALSE) {
            display_ = EGL_NO_DISPLAY;
            return;
        }
        const EGLint config_attr[] = {EGL_SURFACE_TYPE,    EGL_PBUFFER_BIT,
                                      EGL_RED_SIZE,        8,
                                      EGL_GREEN_SIZE,      8,
                                      EGL_BLUE_SIZE,       8,
                                      EGL_DEPTH_SIZE,      24,
                                      EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT,
                                      EGL_NONE};
        EGLConfig config = nullptr;
        EGLint num_config = 0;
        if (eglChooseConfig(display_, config_attr, &config, 1, &num_config) == EGL_FALSE || num_config < 1) {
            return;
        }
        const EGLint pbuffer_attr[] = {EGL_WIDTH, width, EGL_HEIGHT, height, EGL_NONE};
        surface_                    = eglCreatePbufferSurface(display_, config, pbuffer_attr);
        if (surface_ == EGL_NO_SURFACE) {
            return;
        }
        if (eglBindAPI(EGL_OPENGL_API) == EGL_FALSE) {
            return;
        }
        context_ = eglCreateContext(display_, config, EGL_NO_CONTEXT, nullptr);
        if (context_ == EGL_NO_CONTEXT) {
            return;
        }
        if (eglMakeCurrent(display_, surface_, surface_, context_) == EGL_FALSE) {
            return;
        }
        valid_ = true;
    }

    ~EglContext() {
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

    bool valid() const {
        return valid_;
    }

    EglContext(const EglContext&)            = delete;
    EglContext& operator=(const EglContext&) = delete;

private:
    EGLDisplay display_ = EGL_NO_DISPLAY;
    EGLContext context_ = EGL_NO_CONTEXT;
    EGLSurface surface_ = EGL_NO_SURFACE;
    bool valid_         = false;
};

}  // namespace k1sim::module::camera

#endif  // K1SIM_MODULE_CAMERA_EGLCONTEXT_HPP
