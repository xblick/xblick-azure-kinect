// The depth engine's OpenGL context through EGL on an NVIDIA GPU, without an X server (egl_glx.cpp).
#pragma once

namespace xblick::k4a_node
{
    // The GPU (CUDA device ordinal) on which the depth engine's OpenGL context is created. Must be called before a
    // device is opened or a transformation created; without it the depth engine finds no display and fails.
    void use_egl_for_depth_engine(int cuda_device);
}  // namespace xblick::k4a_node
