// The Azure Kinect's depth engine (libdepthengine.so.2.0, closed source) gets its OpenGL 4.4 context through Xlib and
// GLX: XOpenDisplay, an invisible window, glXCreateContextAttribsARB, glXMakeCurrent. Without an X server it fails
// (K4A_DEPTH_ENGINE_RESULT_FATAL_ERROR_GPU_OPENGL_CONTEXT, error 204). It draws into framebuffer objects only, so the
// window is not needed, only the context. This library defines the 15 Xlib/GLX functions the depth engine imports
// (nm -D --undefined-only) and creates the context with EGL on the NVIDIA GPU instead (EGL_EXT_platform_device, a
// pbuffer surface). The executable links this library, so the dynamic linker binds the depth engine's imports here
// before libX11 and libGL, which the depth engine still loads. Its GL functions come from libGL's glXGetProcAddressARB
// (glad); with GLVND these dispatch to whichever context is current, here the EGL one.
//
// Only the depth engine calls these functions in this process; they serve exactly its use and nothing else.
#include "egl_glx.hpp"

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <vector>

namespace
{
    // GLX types and constants (no GL/glx.h needed)
    struct FbConfig
    {
        EGLConfig config;
    };
    struct Context
    {
        EGLContext context;
        EGLSurface surface;
    };
    constexpr int GLX_CONTEXT_MAJOR_VERSION_ARB = 0x2091;
    constexpr int GLX_CONTEXT_MINOR_VERSION_ARB = 0x2092;
    constexpr int GLX_CONTEXT_FLAGS_ARB = 0x2094;
    constexpr int GLX_CONTEXT_PROFILE_MASK_ARB = 0x9126;
    constexpr int GLX_RENDER_TYPE = 0x8011;
    constexpr int GLX_RGBA_TYPE = 0x8014;

    std::mutex mutex;
    int cuda_device = -1;
    EGLDisplay egl_display = EGL_NO_DISPLAY;

    // the one fake X display, screen and visual (Xlib's macros read their fields directly)
    Visual visual{};
    Screen screen{};
    std::remove_pointer_t<_XPrivDisplay> display{};

    void fail(const char* what)
    {
        std::fprintf(stderr, "xblick-azure-kinect: depth engine context: %s (EGL error 0x%x)\n", what,
                     unsigned(eglGetError()));
    }

    template <typename F>
    F egl_function(const char* name)
    {
        return reinterpret_cast<F>(eglGetProcAddress(name));
    }

    // The EGL display of the CUDA device `cuda_device`, initialised; EGL_NO_DISPLAY after printing why not.
    EGLDisplay open_egl_display()
    {
        const auto query_devices = egl_function<PFNEGLQUERYDEVICESEXTPROC>("eglQueryDevicesEXT");
        const auto query_device = egl_function<PFNEGLQUERYDEVICEATTRIBEXTPROC>("eglQueryDeviceAttribEXT");
        const auto platform_display = egl_function<PFNEGLGETPLATFORMDISPLAYEXTPROC>("eglGetPlatformDisplayEXT");
        if (not query_devices or not query_device or not platform_display)
        {
            fail("EGL has no device platform (EGL_EXT_device_enumeration, EGL_EXT_platform_device)");
            return EGL_NO_DISPLAY;
        }
        EGLint n = 0;
        query_devices(0, nullptr, &n);
        std::vector<EGLDeviceEXT> devices(std::size_t(std::max(n, 0)));
        query_devices(n, devices.data(), &n);
        for (EGLDeviceEXT device : devices)
        {
            EGLAttrib ordinal = -1;
            // EGL_NV_device_cuda: only NVIDIA's devices have it; Mesa's software device does not answer
            if (query_device(device, EGL_CUDA_DEVICE_NV, &ordinal) and ordinal == cuda_device)
            {
                EGLDisplay d = platform_display(EGL_PLATFORM_DEVICE_EXT, device, nullptr);
                EGLint major = 0, minor = 0;
                if (d == EGL_NO_DISPLAY or not eglInitialize(d, &major, &minor))
                {
                    fail("eglInitialize failed on the CUDA device's EGL device");
                    return EGL_NO_DISPLAY;
                }
                return d;
            }
        }
        std::fprintf(stderr, "xblick-azure-kinect: depth engine context: no EGL device is CUDA device %d (%d EGL "
                     "devices)\n", cuda_device, int(n));
        return EGL_NO_DISPLAY;
    }
}  // namespace

void xblick::k4a_node::use_egl_for_depth_engine(int device)
{
    std::lock_guard lock(mutex);
    cuda_device = device;
}

extern "C"
{
    __attribute__((visibility("default"))) Display* XOpenDisplay(const char*)
    {
        std::lock_guard lock(mutex);
        if (cuda_device < 0)
        {
            std::fprintf(stderr, "xblick-azure-kinect: depth engine context: no GPU chosen "
                                 "(use_egl_for_depth_engine was not called)\n");
            return nullptr;
        }
        if (egl_display == EGL_NO_DISPLAY)
        {
            egl_display = open_egl_display();
            if (egl_display == EGL_NO_DISPLAY)
            {
                return nullptr;
            }
            visual.visualid = 1;
            visual.c_class = TrueColor;
            visual.bits_per_rgb = 8;
            visual.map_entries = 256;
            visual.red_mask = 0xff0000;
            visual.green_mask = 0x00ff00;
            visual.blue_mask = 0x0000ff;
            screen.display = reinterpret_cast<Display*>(&display);
            screen.root = 1;
            screen.root_visual = &visual;
            screen.root_depth = 24;
            screen.width = screen.height = 1;
            display.screens = &screen;
            display.nscreens = 1;
            display.default_screen = 0;
        }
        return reinterpret_cast<Display*>(&display);
    }

    __attribute__((visibility("default"))) int XCloseDisplay(Display*)
    {
        return 0;  // the EGL display stays initialised for the next context
    }

    __attribute__((visibility("default"))) XErrorHandler XSetErrorHandler(XErrorHandler)
    {
        return nullptr;
    }

    __attribute__((visibility("default"))) Colormap XCreateColormap(Display*, Window, Visual*, int)
    {
        return 1;
    }

    __attribute__((visibility("default"))) int XFreeColormap(Display*, Colormap)
    {
        return 0;
    }

    __attribute__((visibility("default"))) Window XCreateWindow(Display*, Window, int, int, unsigned, unsigned,
                                                                 unsigned, int, unsigned, Visual*, unsigned long,
                                                                 XSetWindowAttributes*)
    {
        return 2;  // never shown: the depth engine draws into framebuffer objects
    }

    __attribute__((visibility("default"))) int XDestroyWindow(Display*, Window)
    {
        return 0;
    }

    // frees what glXChooseFBConfig and glXGetVisualFromFBConfig returned (malloc'd here)
    __attribute__((visibility("default"))) int XFree(void* data)
    {
        std::free(data);
        return 1;
    }

    __attribute__((visibility("default"))) FbConfig** glXChooseFBConfig(Display*, int, const int*, int* count)
    {
        // the requested attributes describe the window's framebuffer, which is never drawn to: any RGBA8 config
        // with a pbuffer (the surface the context is made current with) serves
        const EGLint attributes[] = {EGL_SURFACE_TYPE, EGL_PBUFFER_BIT, EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT,
                                     EGL_RED_SIZE,     8,               EGL_GREEN_SIZE,      8,
                                     EGL_BLUE_SIZE,    8,               EGL_ALPHA_SIZE,      8,
                                     EGL_NONE};
        EGLConfig config;
        EGLint n = 0;
        if (not eglChooseConfig(egl_display, attributes, &config, 1, &n) or n != 1)
        {
            fail("no EGL config with OpenGL and a pbuffer");
            *count = 0;
            return nullptr;
        }
        auto** list = static_cast<FbConfig**>(std::malloc(sizeof(FbConfig*)));
        static FbConfig chosen;
        chosen.config = config;
        list[0] = &chosen;
        *count = 1;
        return list;
    }

    __attribute__((visibility("default"))) XVisualInfo* glXGetVisualFromFBConfig(Display*, FbConfig*)
    {
        auto* info = static_cast<XVisualInfo*>(std::calloc(1, sizeof(XVisualInfo)));
        info->visual = &visual;
        info->visualid = visual.visualid;
        info->depth = 24;
        info->c_class = TrueColor;
        info->red_mask = visual.red_mask;
        info->green_mask = visual.green_mask;
        info->blue_mask = visual.blue_mask;
        info->bits_per_rgb = 8;
        return info;
    }

    __attribute__((visibility("default"))) Context* glXCreateContextAttribsARB(Display*, FbConfig* config,
                                                                                Context* share, int, const int* glx)
    {
        // GLX_ARB_create_context and EGL 1.5 share the meaning and the bit values of version, flags and profile
        std::vector<EGLint> egl;
        int major = 0, minor = 0;
        for (const int* a = glx; a and a[0] != 0; a += 2)
        {
            switch (a[0])
            {
                case GLX_CONTEXT_MAJOR_VERSION_ARB: egl.insert(egl.end(), {EGL_CONTEXT_MAJOR_VERSION, a[1]}); major = a[1]; break;
                case GLX_CONTEXT_MINOR_VERSION_ARB: egl.insert(egl.end(), {EGL_CONTEXT_MINOR_VERSION, a[1]}); minor = a[1]; break;
                case GLX_CONTEXT_FLAGS_ARB: egl.insert(egl.end(), {EGL_CONTEXT_FLAGS_KHR, a[1]}); break;
                case GLX_CONTEXT_PROFILE_MASK_ARB: egl.insert(egl.end(), {EGL_CONTEXT_OPENGL_PROFILE_MASK, a[1]}); break;
                case GLX_RENDER_TYPE:
                    if (a[1] != GLX_RGBA_TYPE)
                    {
                        std::fprintf(stderr, "xblick-azure-kinect: depth engine context: render type 0x%x\n", a[1]);
                        return nullptr;
                    }
                    break;
                default:
                    std::fprintf(stderr, "xblick-azure-kinect: depth engine context: unknown GLX context "
                                         "attribute 0x%x\n", a[0]);
                    return nullptr;
            }
        }
        egl.push_back(EGL_NONE);
        if (not eglBindAPI(EGL_OPENGL_API))
        {
            fail("eglBindAPI(EGL_OPENGL_API) failed");
            return nullptr;
        }
        auto* context = new Context{};
        context->context = eglCreateContext(egl_display, config->config, share ? share->context : EGL_NO_CONTEXT,
                                            egl.data());
        const EGLint size[] = {EGL_WIDTH, 1, EGL_HEIGHT, 1, EGL_NONE};
        context->surface = eglCreatePbufferSurface(egl_display, config->config, size);
        if (context->context == EGL_NO_CONTEXT or context->surface == EGL_NO_SURFACE)
        {
            fail("eglCreateContext or eglCreatePbufferSurface failed");
            delete context;
            return nullptr;
        }
        std::fprintf(stderr, "xblick-azure-kinect: depth engine: OpenGL %d.%d context on CUDA device %d (EGL, no X)\n",
                     major, minor, cuda_device);
        return context;
    }

    __attribute__((visibility("default"))) int glXMakeCurrent(Display*, unsigned long, Context* context)
    {
        eglBindAPI(EGL_OPENGL_API);  // per thread
        const bool ok = context ? eglMakeCurrent(egl_display, context->surface, context->surface, context->context)
                                : eglMakeCurrent(egl_display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
        if (not ok)
        {
            fail("eglMakeCurrent failed");
        }
        return ok;
    }

    __attribute__((visibility("default"))) void glXDestroyContext(Display*, Context* context)
    {
        if (context)
        {
            eglDestroySurface(egl_display, context->surface);
            eglDestroyContext(egl_display, context->context);
            delete context;
        }
    }

    __attribute__((visibility("default"))) void (*glXGetProcAddressARB(const unsigned char* name))()
    {
        if (std::strcmp(reinterpret_cast<const char*>(name), "glXCreateContextAttribsARB") == 0)
        {
            return reinterpret_cast<void (*)()>(&glXCreateContextAttribsARB);
        }
        return eglGetProcAddress(reinterpret_cast<const char*>(name));
    }
}
