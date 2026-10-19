#include "core/spectrum/SpectralReflectanceCurve.h"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <string>

static std::string Utf8(const std::filesystem::path& path) {
    auto text = path.generic_u8string(); return {text.begin(), text.end()};
}
static std::string Csv(std::string text) {
    std::string result = "\"";
    for (char c : text) { if (c == '"') result += '"'; result += c; }
    return result + '"';
}
int wmain(int argc, wchar_t** argv) {
    if (argc != 3) return 2;
    const auto root = std::filesystem::path(argv[1]);
    std::ofstream out(std::filesystem::path(argv[2]), std::ios::out);
    out << "path,status,name,sample_number,measurement,count,min_nm,max_nm,rho_905,rho_1550,detail\n";
    std::size_t total = 0, accepted = 0, at905 = 0, at1550 = 0;
    std::map<std::string, std::size_t> reasons;
    for (const auto& entry : std::filesystem::recursive_directory_iterator(root)) {
        if (!entry.is_regular_file()) continue;
        ++total;
        out << Csv(Utf8(std::filesystem::relative(entry.path(), root))) << ',';
        try {
            auto curve = SpectralReflectanceCurve::FromAsterFile(entry.path());
            ++accepted;
            out << "accepted," << Csv(curve.Metadata().name) << ',' << Csv(curve.Metadata().sampleNumber)
                << ',' << Csv(curve.Metadata().measurement) << ',' << curve.SampleCount() << ','
                << curve.MinimumWavelengthNm() << ',' << curve.MaximumWavelengthNm() << ',';
            if (curve.Contains(905)) { ++at905; out << curve.Evaluate(905); }
            out << ',';
            if (curve.Contains(1550)) { ++at1550; out << curve.Evaluate(1550); }
            out << ",\n";
        } catch (const std::exception& e) {
            std::string detail = e.what();
            auto split = detail.find(": ");
            std::string reason = split == std::string::npos ? detail : detail.substr(split + 2);
            ++reasons[reason];
            out << "rejected,,,,,,,,," << Csv(detail) << '\n';
        }
    }
    std::cout << "Files=" << total << " accepted=" << accepted << " rejected=" << (total-accepted)
        << " supports905=" << at905 << " supports1550=" << at1550 << '\n';
    for (const auto& [reason, count] : reasons) std::cout << count << " : " << reason << '\n';
    return !out.good();
}
