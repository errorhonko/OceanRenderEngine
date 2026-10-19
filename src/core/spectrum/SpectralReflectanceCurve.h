#pragma once
#include "core/spectrum/SpectrumInterpolation.h"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <istream>
#include <limits>
#include <locale>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

struct SpectralReflectanceSample
{
    double wavelengthNm = 0.0;
    double reflectance = 0.0; // Dimensionless rho, NOT percent or BRDF sr^-1.
};

struct SpectralReflectanceMetadata
{
    std::string name;
    std::string sampleNumber;
    std::string measurement; // Preserve acquisition geometry; not a BRDF definition.
    std::string source;
};

// PBRT v4 PiecewiseLinearSpectrum concept: tabulated wavelength/value pairs.
// Deliberate differences: double precision, rho in [0,1], no visible-only range,
// no extrapolation/implicit zero outside support, no RGB/CIE normalization.
// Header-only, independent of renderer, materials, BRDF and LiDAR state.
class SpectralReflectanceCurve
{
public:
    explicit SpectralReflectanceCurve(std::vector<SpectralReflectanceSample> samples)
        : samples(std::move(samples))
    {
        Validate();
    }

    double Evaluate(double wavelengthNm) const
    {
        RequireSamples();
        if (!std::isfinite(wavelengthNm) || wavelengthNm <= 0.0)
            throw std::invalid_argument("Reflectance query wavelength must be finite positive nm.");
        if (!Contains(wavelengthNm))
            throw std::out_of_range("Reflectance query lies outside the measured wavelength range.");
        const auto interval = SpectrumInterpolation::FindInterval(std::span(samples), wavelengthNm,
            [](const SpectralReflectanceSample& sample) { return sample.wavelengthNm; });
        return interval.Evaluate(samples[interval.lower].reflectance, samples[interval.upper].reflectance);
    }

    bool Contains(double wavelengthNm) const noexcept
    {
        return samples.size() >= 2 && std::isfinite(wavelengthNm) && wavelengthNm > 0.0 &&
            wavelengthNm >= samples.front().wavelengthNm && wavelengthNm <= samples.back().wavelengthNm;
    }
    std::size_t SampleCount() const noexcept { return samples.size(); }
    double MinimumWavelengthNm() const { RequireSamples(); return samples.front().wavelengthNm; }
    double MaximumWavelengthNm() const { RequireSamples(); return samples.back().wavelengthNm; }
    const std::vector<SpectralReflectanceSample>& Samples() const noexcept { return samples; }
    const SpectralReflectanceMetadata& Metadata() const noexcept { return metadata; }

    // ASTER/ECOSTRESS two-column text with declared X/Y units. Headers are NOT
    // assumed to have 26 lines. Unknown descriptive headers are skipped before
    // the first numeric row; afterwards every nonblank/noncomment row must be
    // exactly two numbers. Never guess missing units, clamp measured values,
    // reorder samples, or average duplicate wavelengths.
    static SpectralReflectanceCurve FromAsterStream(std::istream& input,
        std::string sourceName = "<stream>")
    {
        if (!input.good()) throw std::runtime_error(sourceName + ": input stream is not readable.");
        std::map<std::string, std::string> headers;
        std::vector<SpectralReflectanceSample> data;
        std::string line;
        std::size_t lineNumber = 0;
        bool started = false;
        double wavelengthScale = 0.0, reflectanceScale = 0.0;
        while (std::getline(input, line))
        {
            ++lineNumber;
            if (lineNumber == 1 && line.starts_with("\xEF\xBB\xBF")) line.erase(0, 3);
            std::string content = Trim(line);
            if (content.empty() || content.front() == '#') continue;
            if (!started && !NumericStart(content))
            {
                const auto colon = content.find(':');
                if (colon == std::string::npos) continue; // Descriptive continuation.
                const auto key = Normalize(content.substr(0, colon));
                if (RecognizedHeader(key))
                {
                    if (!headers.emplace(key, Trim(content.substr(colon + 1))).second)
                        throw Error(sourceName, lineNumber, "duplicate header: " + key);
                }
                continue;
            }
            const auto comment = content.find('#');
            if (comment != std::string::npos) content = Trim(content.substr(0, comment));
            if (!started)
            {
                wavelengthScale = WavelengthScale(headers, sourceName, lineNumber);
                reflectanceScale = ReflectanceScale(headers, sourceName, lineNumber);
                CheckColumn(headers, "firstcolumn", "x", sourceName, lineNumber);
                CheckColumn(headers, "secondcolumn", "y", sourceName, lineNumber);
                started = true;
            }
            std::istringstream row(content);
            row.imbue(std::locale::classic());
            std::string x, y, extra;
            if (!(row >> x >> y) || (row >> extra))
                throw Error(sourceName, lineNumber, "expected exactly two numeric columns.");
            const SpectralReflectanceSample sample{
                Number(x, sourceName, lineNumber) * wavelengthScale,
                Number(y, sourceName, lineNumber) * reflectanceScale};
            ValidateSample(sample, sourceName, lineNumber);
            if (!data.empty() && sample.wavelengthNm <= data.back().wavelengthNm)
                throw Error(sourceName, lineNumber, "wavelengths must be strictly increasing.");
            data.push_back(sample);
        }
        if (input.bad() || (input.fail() && !input.eof()))
            throw std::runtime_error(sourceName + ": error reading spectral stream.");
        if (data.size() < 2)
            throw Error(sourceName, lineNumber, "a curve requires at least two data rows.");
        const auto count = headers.find("numberofxvalues");
        if (count != headers.end())
        {
            const std::string value = Trim(count->second);
            std::size_t declared = 0;
            const auto parsed = std::from_chars(value.data(), value.data() + value.size(), declared);
            if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size() || declared != data.size())
                throw Error(sourceName, lineNumber, "declared sample count does not match data rows.");
        }
        CheckEndpoint(headers, "firstxvalue", data.front().wavelengthNm, wavelengthScale, sourceName, lineNumber);
        CheckEndpoint(headers, "lastxvalue", data.back().wavelengthNm, wavelengthScale, sourceName, lineNumber);
        SpectralReflectanceCurve curve(std::move(data));
        curve.metadata = {HeaderValue(headers, "name"), HeaderValue(headers, "sampleno."),
            HeaderValue(headers, "measurement"), std::move(sourceName)};
        return curve;
    }

    static SpectralReflectanceCurve FromAsterFile(const std::filesystem::path& path)
    {
        if (path.empty()) throw std::invalid_argument("Spectral file path must not be empty.");
        const auto utf8 = path.generic_u8string();
        const std::string source(utf8.begin(), utf8.end());
        std::ifstream input(path, std::ios::binary);
        if (!input) throw std::runtime_error("Cannot open spectral file: " + source);
        return FromAsterStream(input, source);
    }

private:
    std::vector<SpectralReflectanceSample> samples;
    SpectralReflectanceMetadata metadata;

    static std::invalid_argument Error(const std::string& source, std::size_t line, const std::string& detail)
    {
        return std::invalid_argument(source + ":" + std::to_string(line) + ": " + detail);
    }
    static bool Space(char c) noexcept
    {
        return c == ' ' || c == '\t' || c == '\r' || c == '\n' || c == '\f' || c == '\v';
    }
    static std::string Trim(const std::string& text)
    {
        std::size_t begin = 0, end = text.size();
        while (begin < end && Space(text[begin])) ++begin;
        while (end > begin && Space(text[end - 1])) --end;
        return text.substr(begin, end - begin);
    }
    static std::string Normalize(const std::string& text)
    {
        std::string result;
        for (char c : text) if (!Space(c)) result.push_back(c >= 'A' && c <= 'Z' ? char(c - 'A' + 'a') : c);
        return result;
    }
    static bool NumericStart(const std::string& text)
    {
        const char c = text.front();
        if ((c >= '0' && c <= '9') || c == '+' || c == '-' || c == '.') return true;
        const auto end = text.find_first_of(" \t\r\n\f\v");
        const auto token = Normalize(text.substr(0, end));
        return token == "nan" || token.starts_with("nan(") || token == "inf" || token == "infinity";
    }
    static bool RecognizedHeader(const std::string& key)
    {
        return key == "name" || key == "sampleno." || key == "measurement" || key == "xunits" ||
            key == "yunits" || key == "firstcolumn" || key == "secondcolumn" || key == "firstxvalue" ||
            key == "lastxvalue" || key == "numberofxvalues";
    }
    static double Number(std::string_view text, const std::string& source, std::size_t line)
    {
        if (!text.empty() && text.front() == '+')
        {
            text.remove_prefix(1);
            if (text.empty() || text.front() == '+' || text.front() == '-')
                throw Error(source, line, "invalid numeric sign.");
        }
        double result = 0.0;
        const auto parsed = std::from_chars(text.data(), text.data() + text.size(), result, std::chars_format::general);
        if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() || !std::isfinite(result))
            throw Error(source, line, "invalid or nonfinite numeric value.");
        return result;
    }
    static std::string HeaderValue(const std::map<std::string, std::string>& headers, const std::string& key)
    {
        const auto found = headers.find(key);
        return found == headers.end() ? std::string{} : found->second;
    }
    static double WavelengthScale(const std::map<std::string, std::string>& headers,
        const std::string& source, std::size_t line)
    {
        const auto unit = Normalize(HeaderValue(headers, "xunits"));
        if (unit == "wavelength(micrometers)" || unit == "wavelength(micrometres)" ||
            unit == "wavelength(um)" || unit == "wavelength(\xC2\xB5m)" || unit == "wavelength(\xCE\xBCm)") return 1000.0;
        if (unit == "wavelength(nanometers)" || unit == "wavelength(nanometres)" || unit == "wavelength(nm)") return 1.0;
        throw Error(source, line, "X Units must declare wavelength in micrometers or nanometers.");
    }
    static double ReflectanceScale(const std::map<std::string, std::string>& headers,
        const std::string& source, std::size_t line)
    {
        const auto unit = Normalize(HeaderValue(headers, "yunits"));
        if (unit == "reflectance(percent)" || unit == "reflectance(%)") return 0.01;
        if (unit == "reflectance(fraction)" || unit == "reflectance(dimensionless)" || unit == "reflectance(unitless)") return 1.0;
        throw Error(source, line, "Y Units must declare reflectance in percent or dimensionless fraction.");
    }
    static void CheckColumn(const std::map<std::string, std::string>& headers, const std::string& key,
        const std::string& expected, const std::string& source, std::size_t line)
    {
        const auto found = headers.find(key);
        if (found != headers.end() && Normalize(found->second) != expected)
            throw Error(source, line, "unsupported column declaration: " + key);
    }
    static void CheckEndpoint(const std::map<std::string, std::string>& headers, const std::string& key,
        double actual, double scale, const std::string& source, std::size_t line)
    {
        const auto found = headers.find(key);
        if (found == headers.end()) return;
        const double declared = Number(found->second, source, line) * scale;
        const double tolerance = 32 * std::numeric_limits<double>::epsilon() * std::max(std::abs(actual), std::abs(declared));
        if (!std::isfinite(declared) || declared <= 0 || std::abs(declared - actual) > tolerance)
            throw Error(source, line, "declared wavelength endpoint does not match data: " + key);
    }
    static void ValidateSample(const SpectralReflectanceSample& sample, const std::string& source, std::size_t line)
    {
        if (!std::isfinite(sample.wavelengthNm) || sample.wavelengthNm <= 0)
            throw Error(source, line, "sample wavelength must be finite positive nm.");
        if (!std::isfinite(sample.reflectance) || sample.reflectance < 0 || sample.reflectance > 1)
            throw Error(source, line, "sample reflectance must lie in [0,1]; no implicit clamping.");
    }
    void Validate() const
    {
        if (samples.size() < 2) throw std::invalid_argument("Reflectance curve requires at least two samples.");
        for (std::size_t i = 0; i < samples.size(); ++i)
        {
            ValidateSample(samples[i], "<samples>", i + 1);
            if (i && samples[i].wavelengthNm <= samples[i - 1].wavelengthNm)
                throw std::invalid_argument("Reflectance wavelengths must be strictly increasing, without duplicates.");
        }
    }
    void RequireSamples() const
    {
        if (samples.size() < 2) throw std::logic_error("Reflectance curve has no valid samples (moved-from object).");
    }
};
