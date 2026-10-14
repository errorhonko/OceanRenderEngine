// Compile the vendored implementation exactly once per executable.
// Other translation units include the header without defining this macro.
// The pinned embedded fast_float fails with MSVC C3615 under /std:c++20.
// Use upstream's supported fallback without modifying the vendored header.
#define TINYOBJLOADER_DISABLE_FAST_FLOAT
#define TINYOBJLOADER_IMPLEMENTATION
#include "third_party/tinyobjloader/tiny_obj_loader.h"
