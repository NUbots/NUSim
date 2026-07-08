#ifndef K1SIM_SHARED_GL_XTHREADS_HPP
#define K1SIM_SHARED_GL_XTHREADS_HPP

// Must be called ONCE, before any X11/GLFW/GL use, in any process that renders GL
// from more than one thread (here: module::Viewer on MainThread + module::Camera
// on its own render thread). Without it, concurrent Xlib calls corrupt the heap
// ("malloc(): invalid size"). XInitThreads() turns on Xlib's internal locking.
#if defined(__linux__) && defined(__has_include)
#if __has_include(<X11/Xlib.h>)
#include <X11/Xlib.h>
namespace k1sim {
inline void init_x_threads() {
    XInitThreads();
}
}  // namespace k1sim
#define K1SIM_HAVE_X11 1
#endif
#endif

#ifndef K1SIM_HAVE_X11
namespace k1sim {
inline void init_x_threads() {}
}  // namespace k1sim
#endif

#endif  // K1SIM_SHARED_GL_XTHREADS_HPP
