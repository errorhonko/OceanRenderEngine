#pragma once

#include "core/spectrum/SpectralOpticalConstants.h"
#include <charconv>
#include <filesystem>
#include <fstream>
#include <istream>
#include <locale>
#include <sstream>
#include <string_view>
#include <system_error>

enum class OpticalConstantsWavelengthUnit { Nanometers, Micrometers };

// Deliberately NOT a general YAML/CSV/ASTER parser: whitespace-separated
// wavelength/n/k rows, '#' comments, explicit wavelength unit, dimensionless n/k.
// File parsing stays separate from the interpolation container and BRDF adapters.
class OpticalConstantsTableReader
{
public:
    static SpectralOpticalConstants FromStream(std::istream& input,
        OpticalConstantsWavelengthUnit unit, SpectralOpticalConstantsMetadata metadata = {},
        std::string sourceName = "<stream>")
    {
        const double scale = WavelengthScale(unit);
        if (!input.good()) throw std::runtime_error(sourceName + ": input stream is not readable.");
        if (metadata.source.empty()) metadata.source = sourceName;
        std::vector<OpticalConstantsSample> samples;
        std::string line;
        std::size_t lineNumber = 0;
        while (std::getline(input, line))
        {
            ++lineNumber;
            if (lineNumber == 1 && line.starts_with("\xEF\xBB\xBF")) line.erase(0, 3);
            if (const auto comment = line.find('#'); comment != std::string::npos) line.resize(comment);
            std::istringstream row(line);
            row.imbue(std::locale::classic());
            std::string wavelength, n, k, extra;
            if (!(row >> wavelength)) continue;
            if (!(row >> n >> k) || (row >> extra))
                throw Error(sourceName, lineNumber, "expected exactly three columns: wavelength n k.");
            const double lambdaNm = Number(wavelength, sourceName, lineNumber) * scale;
            if (!std::isfinite(lambdaNm))
                throw Error(sourceName, lineNumber, "wavelength unit conversion overflow.");
            samples.push_back({lambdaNm, Number(n, sourceName, lineNumber), Number(k, sourceName, lineNumber)});
        }
        if (input.bad() || (input.fail() && !input.eof()))
            throw std::runtime_error(sourceName + ": error reading optical-constant stream.");
        // Reuse the container's physical validation; never sort, clamp or guess.
        try { return SpectralOpticalConstants(std::move(samples), std::move(metadata)); }
        catch (const std::invalid_argument& error)
        { throw std::invalid_argument(sourceName + ": invalid optical-constant table: " + error.what()); }
    }

    static SpectralOpticalConstants FromFile(const std::filesystem::path& path,
        OpticalConstantsWavelengthUnit unit, SpectralOpticalConstantsMetadata metadata = {})
    {
        if (path.empty()) throw std::invalid_argument("Optical-constant file path must not be empty.");
        const auto utf8 = path.generic_u8string();
        const std::string source(utf8.begin(), utf8.end());
        std::ifstream input(path, std::ios::binary); // Native path, including Unicode filenames.
        if (!input) throw std::runtime_error("Cannot open optical-constant file: " + source);
        return FromStream(input, unit, std::move(metadata), source);
    }

private:
    static double WavelengthScale(OpticalConstantsWavelengthUnit unit)
    {
        switch (unit)
        {
        case OpticalConstantsWavelengthUnit::Nanometers: return 1.0;
        case OpticalConstantsWavelengthUnit::Micrometers: return 1000.0;
        default: throw std::invalid_argument("Unsupported optical-constant wavelength unit.");
        }
    }
    static std::invalid_argument Error(const std::string& source, std::size_t line, const std::string& detail)
    { return std::invalid_argument(source + ":" + std::to_string(line) + ": " + detail); }
    static double Number(std::string_view text, const std::string& source, std::size_t line)
    {
        if (!text.empty() && text.front() == '+')
        {
            text.remove_prefix(1);
            if (text.empty() || text.front() == '+' || text.front() == '-')
                throw Error(source, line, "invalid numeric sign.");
        }
        double value = 0.0;
        const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value, std::chars_format::general);
        if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() || !std::isfinite(value))
            throw Error(source, line, "invalid or nonfinite numeric value.");
        return value;
    }
};
