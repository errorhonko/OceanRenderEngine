#include "core/spectrum/SampledSpectrum.h"

// A second translation unit deliberately includes the legacy header, checking
// that reusable interpolation/CIE functions do not cause duplicate definitions.
float SpectrumInterpolationOtherTranslationUnit()
{
    float x, y, z;
    CIE_XYZ(550, x, y, z);
    return Interpolate(500, 400, 600, 2, 4) + y;
}
