#include "GpuCompute.h"
#include "TriangleQueryAcceptance.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string_view>

int main(int argc, char** argv) {
    bool warp = false;
    if (argc == 2 && std::string_view(argv[1]) == "--warp") warp = true;
    else if (argc != 1) {
        std::cerr << "Usage: OceanRenderEngine.GpuTests.exe [--warp]\n";
        return 1;
    }
    try {
        ocean::gpu::GpuCompute gpu(warp);
        const auto& info = gpu.Info();
        std::wcout << L"Adapter: " << info.adapterName << L'\n';
        std::cout << "Dedicated VRAM: " << info.dedicatedVideoMemoryBytes / (1024 * 1024)
                  << " MiB\nBackend: " << (info.softwareAdapter ? "WARP SOFTWARE" : "HARDWARE")
                  << "\nDXR tier: " << info.raytracingTier
                  << " (0=unsupported, 10=1.0, 11=1.1)\nDebug layer: "
                  << (info.debugLayerEnabled ? "enabled" : "disabled/unavailable") << '\n';
        if (warp) std::cout << "Software verification only; this is not GPU performance evidence.\n";
        std::size_t checked = 0;
        for (std::size_t size : {0u, 1u, 63u, 64u, 65u, 257u, 4097u}) {
            std::vector<float> input(size);
            for (std::size_t i = 0; i < size; ++i)
                input[i] = static_cast<float>(static_cast<int>(i % 127) - 63) * 0.25f;
            for (const auto [scale, bias] : {std::pair{2.0f, -0.5f}, std::pair{-0.5f, 3.0f}}) {
                const auto output = gpu.Transform(input, scale, bias);
                if (output.size() != input.size()) throw std::runtime_error("Output size mismatch");
                for (std::size_t i = 0; i < size; ++i) {
                    const float reference = input[i] * scale + bias;
                    if (!std::isfinite(output[i]) || std::abs(output[i] - reference) > 1e-6f)
                        throw std::runtime_error("CPU/GPU mismatch at size " + std::to_string(size)
                            + ", index " + std::to_string(i));
                    ++checked;
                }
            }
            std::cout << "PASS size=" << size << '\n';
        }
        bool rejected = false;
        try { gpu.Transform({}, std::numeric_limits<float>::quiet_NaN(), 0); }
        catch (const std::invalid_argument&) { rejected = true; }
        if (!rejected) throw std::runtime_error("Nonfinite parameter was accepted");
        const float invalid[]{std::numeric_limits<float>::infinity()};
        rejected = false;
        try { gpu.Transform(invalid, 1, 0); }
        catch (const std::invalid_argument&) { rejected = true; }
        if (!rejected) throw std::runtime_error("Nonfinite input was accepted");
        std::cout << "PASS invalid inputs\nPASS " << checked << " element comparisons\n";
        RunTriangleQueryAcceptance(gpu);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAILED: " << error.what() << '\n';
        return 1;
    }
}
