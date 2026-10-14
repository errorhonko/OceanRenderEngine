#include "core/spectrum/SpectralReflectanceCurve.h" // Must be self-contained.
#include <iostream>
#include <limits>
#include <sstream>
#include <type_traits>

namespace {
void Check(const char* name, bool condition)
{
    if (!condition) { std::cerr << "[FAIL] " << name << '\n'; throw std::runtime_error(name); }
    std::cout << "[PASS] " << name << '\n';
}
bool Near(double a, double b, double tolerance = 1e-12) { return std::abs(a-b) <= tolerance; }
template<class Exception, class Function> bool Rejects(Function function)
{
    try { function(); } catch (const Exception&) { return true; } return false;
}
SpectralReflectanceCurve Parse(const std::string& body)
{
    std::istringstream input(body);
    return SpectralReflectanceCurve::FromAsterStream(input, "synthetic-stream");
}
const std::string Units = "X Units: Wavelength (micrometers)\nY Units: Reflectance (percent)\n";
const std::string Rows = "0.4 20\n0.6 60\n1.55 80\n";
std::filesystem::path Fixture()
{
    return std::filesystem::path(__FILE__).parent_path()/"fixtures"/"spectra"/"synthetic_reflectance.txt";
}
}
void RunSpectralReflectanceCurveAcceptanceTests()
{
    static_assert(!std::is_default_constructible_v<SpectralReflectanceCurve>);
    static_assert(std::is_same_v<decltype(std::declval<const SpectralReflectanceCurve&>().Evaluate(905.0)), double>);
    const SpectralReflectanceCurve curve({{400,0.2},{600,0.6},{1550,0.8}});
    Check("spectral curve owns three double precision sample pairs", curve.SampleCount()==3 && curve.Samples()[1].wavelengthNm==600);
    Check("spectral curve retains its measured wavelength domain", curve.MinimumWavelengthNm()==400 && curve.MaximumWavelengthNm()==1550);
    Check("spectral curve evaluates both closed endpoints exactly", curve.Evaluate(400)==0.2 && curve.Evaluate(1550)==0.8);
    Check("spectral curve evaluates internal knots exactly", curve.Evaluate(600)==0.6);
    Check("spectral curve linearly interpolates an interior wavelength", Near(curve.Evaluate(500),0.4));
    Check("spectral curve supports near infrared instead of RGB visible limits", Near(curve.Evaluate(905),0.6+0.2*305/950));
    Check("spectral curve reports inclusive support without extrapolation", curve.Contains(400)&&curve.Contains(1550)&&!curve.Contains(399)&&!curve.Contains(1551));
    Check("spectral curve rejects wavelength below measured support", Rejects<std::out_of_range>([&]{ (void)curve.Evaluate(399); }));
    Check("spectral curve rejects wavelength above measured support", Rejects<std::out_of_range>([&]{ (void)curve.Evaluate(1551); }));
    for (const double invalid : {0.0,-1.0,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {
        Check("spectral query rejects nonpositive or nonfinite wavelength", Rejects<std::invalid_argument>([&]{ (void)curve.Evaluate(invalid); }));
        Check("spectral support probe safely excludes invalid wavelengths", !curve.Contains(invalid));
    }
    const SpectralReflectanceCurve declining({{400,1},{800,0}}), constant({{100,0.3},{20000,0.3}});
    Check("spectral curve supports decreasing reflectance", Near(declining.Evaluate(600),0.5));
    Check("spectral curve retains reflectance zero and one without normalization", declining.Evaluate(400)==1 && declining.Evaluate(800)==0);
    Check("spectral constant curve remains constant on its declared support", constant.Evaluate(905)==0.3 && constant.Evaluate(1550)==0.3);
    bool bounded = true;
    for (int i=0;i<=1150;++i) { const double value=curve.Evaluate(400+i); bounded &= std::isfinite(value)&&value>=0.2&&value<=0.8; }
    Check("spectral dense interpolation stays finite inside the sample convex hull", bounded);
    std::vector<SpectralReflectanceSample> source{{400,0.2},{800,0.6}};
    const SpectralReflectanceCurve independent(source); source[0]={1,1};
    Check("spectral curve does not alias caller owned sample storage", independent.Evaluate(400)==0.2);
    Check("spectral curve refuses an empty sample table", Rejects<std::invalid_argument>([]{ SpectralReflectanceCurve({}); }));
    Check("spectral curve refuses a one point table", Rejects<std::invalid_argument>([]{ SpectralReflectanceCurve({{500,0.2}}); }));
    Check("spectral curve refuses unsorted wavelengths instead of rearranging data", Rejects<std::invalid_argument>([]{ SpectralReflectanceCurve({{600,0.2},{400,0.3}}); }));
    Check("spectral curve refuses duplicate wavelengths", Rejects<std::invalid_argument>([]{ SpectralReflectanceCurve({{500,0.2},{500,0.3}}); }));
    for (const double invalid : {0.0,-1.0,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()})
        Check("spectral curve validates every sample wavelength", Rejects<std::invalid_argument>([&]{ SpectralReflectanceCurve({{400,0.2},{invalid,0.3}}); }));
    for (const double invalid : {-0.01,1.01,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()})
        Check("spectral curve rejects invalid reflectance without clamping", Rejects<std::invalid_argument>([&]{ SpectralReflectanceCurve({{400,0.2},{600,invalid}}); }));
    const double next = std::nextafter(500.0,std::numeric_limits<double>::infinity());
    const SpectralReflectanceCurve close({{500,0.1},{next,0.9}});
    Check("spectral curve retains distinct adjacent representable wavelengths", close.Evaluate(500)==0.1 && close.Evaluate(next)==0.9);

    const auto parsed=Parse("Name: Test material\nSample No.: SAMPLE-1\nMeasurement: Measured geometry text\n"+Units+Rows);
    Check("ASTER converts micrometers to nanometers", parsed.MinimumWavelengthNm()==400 && parsed.MaximumWavelengthNm()==1550);
    Check("ASTER converts percentages to fractional reflectance", Near(parsed.Evaluate(400),0.2) && Near(parsed.Evaluate(600),0.6));
    Check("ASTER converted curve interpolates in wavelength space", Near(parsed.Evaluate(500),0.4));
    Check("ASTER preserves sample identity and measurement metadata", parsed.Metadata().name=="Test material" && parsed.Metadata().sampleNumber=="SAMPLE-1" && parsed.Metadata().measurement=="Measured geometry text");
    Check("ASTER records caller supplied provenance", parsed.Metadata().source=="synthetic-stream");
    Check("ASTER preserves hash characters inside descriptive sample metadata", Parse("Name: Sample #7\n"+Units+Rows).Metadata().name=="Sample #7");
    const auto varied=Parse("# comment\nInformation: descriptive header\ncontinued descriptive text\n\n"+Units+"\n0.4\t20\n# comment between rows\n0.6    60 # trailing comment\n1.55 80");
    Check("ASTER reads variable header length comments whitespace and missing final newline", varied.SampleCount()==3 && Near(varied.Evaluate(1550),0.8));
    const auto crlf=Parse("\xEF\xBB\xBFName: BOM material\r\nX Units: Wavelength (micrometers)\r\nY Units: Reflectance (percent)\r\n0.4 20\r\n0.6 60\r\n");
    Check("ASTER accepts UTF8 BOM and Windows CRLF", crlf.Metadata().name=="BOM material" && crlf.SampleCount()==2);
    const auto nm=Parse("X Units: Wavelength (nanometers)\nY Units: Reflectance (fraction)\n400 0.2\n600 0.6\n");
    Check("ASTER honors explicitly declared nanometers and fractional units", Near(nm.Evaluate(500),0.4));
    const auto variants=Parse(" x UNITS : wavelength ( um )\n y units : reflectance ( % )\n+.4 2e1\n+6e-1 6E1\n");
    Check("ASTER accepts declared unit spelling case and scientific numeric notation", Near(variants.Evaluate(500),0.4));
    const auto micro=Parse("X Units: Wavelength (\xC2\xB5m)\nY Units: Reflectance (percent)\n"+Rows);
    Check("ASTER accepts the UTF8 micro symbol when declared", Near(micro.Evaluate(500),0.4));
    Check("ASTER verifies optional count endpoints and column metadata", Parse("First Column: X\nSecond Column: Y\nNumber of X Values: 3\nFirst X Value: .4\nLast X Value: 1.55\n"+Units+Rows).SampleCount()==3);
    Check("ASTER refuses unitless raw columns instead of guessing units", Rejects<std::invalid_argument>([]{ (void)Parse(Rows); }));
    Check("ASTER refuses a missing reflectance unit declaration", Rejects<std::invalid_argument>([]{ (void)Parse("X Units: Wavelength (micrometers)\n"+Rows); }));
    Check("ASTER refuses wavenumbers instead of treating them as wavelengths", Rejects<std::invalid_argument>([]{ (void)Parse("X Units: Wavenumber (cm-1)\nY Units: Reflectance (percent)\n"+Rows); }));
    Check("ASTER refuses emissivity instead of treating it as reflectance", Rejects<std::invalid_argument>([]{ (void)Parse("X Units: Wavelength (micrometers)\nY Units: Emissivity (percent)\n"+Rows); }));
    Check("ASTER refuses unsupported wavelength units", Rejects<std::invalid_argument>([]{ (void)Parse("X Units: Wavelength (meters)\nY Units: Reflectance (percent)\n"+Rows); }));
    Check("ASTER refuses ambiguous duplicate unit declarations", Rejects<std::invalid_argument>([]{ (void)Parse(Units+Units+Rows); }));
    Check("ASTER refuses duplicate wavelengths", Rejects<std::invalid_argument>([]{ (void)Parse(Units+"0.4 20\n0.4 30\n"); }));
    Check("ASTER refuses descending tables", Rejects<std::invalid_argument>([]{ (void)Parse(Units+"0.6 20\n0.4 30\n"); }));
    Check("ASTER refuses a negative sample wavelength", Rejects<std::invalid_argument>([]{ (void)Parse(Units+"-0.4 20\n0.6 30\n"); }));
    Check("ASTER refuses a negative measured percentage", Rejects<std::invalid_argument>([]{ (void)Parse(Units+"0.4 -1\n0.6 30\n"); }));
    Check("ASTER refuses measured percentages above one hundred", Rejects<std::invalid_argument>([]{ (void)Parse(Units+"0.4 101\n0.6 30\n"); }));
    Check("ASTER refuses NaN sample values", Rejects<std::invalid_argument>([]{ (void)Parse(Units+"0.4 nan\n0.6 30\n"); }));
    Check("ASTER refuses infinite wavelengths", Rejects<std::invalid_argument>([]{ (void)Parse(Units+"inf 20\n0.6 30\n"); }));
    Check("ASTER refuses malformed numeric suffixes", Rejects<std::invalid_argument>([]{ (void)Parse(Units+"0.4 20oops\n0.6 30\n"); }));
    Check("ASTER refuses a malformed double sign even on zero reflectance", Rejects<std::invalid_argument>([]{ (void)Parse(Units+"0.4 +-0\n0.6 30\n"); }));
    Check("ASTER refuses an extra numeric column", Rejects<std::invalid_argument>([]{ (void)Parse(Units+"0.4 20 99\n0.6 30\n"); }));
    Check("ASTER refuses incomplete numeric rows", Rejects<std::invalid_argument>([]{ (void)Parse(Units+"0.4\n0.6 30\n"); }));
    Check("ASTER refuses nonnumeric corruption after the table starts", Rejects<std::invalid_argument>([]{ (void)Parse(Units+"0.4 20\ncorrupt data\n0.6 30\n"); }));
    Check("ASTER refuses header mutations inside the data table", Rejects<std::invalid_argument>([]{ (void)Parse(Units+"0.4 20\nY Units: Reflectance (fraction)\n0.6 30\n"); }));
    Check("ASTER refuses data overflow during unit conversion", Rejects<std::invalid_argument>([]{ (void)Parse(Units+"1e307 20\n1e308 30\n"); }));
    Check("ASTER refuses header only files", Rejects<std::invalid_argument>([]{ (void)Parse(Units); }));
    Check("ASTER refuses single row files", Rejects<std::invalid_argument>([]{ (void)Parse(Units+"0.4 20\n"); }));
    Check("ASTER refuses inconsistent declared sample count", Rejects<std::invalid_argument>([]{ (void)Parse("Number of X Values: 4\n"+Units+Rows); }));
    Check("ASTER refuses negative declared sample count", Rejects<std::invalid_argument>([]{ (void)Parse("Number of X Values: -3\n"+Units+Rows); }));
    Check("ASTER refuses malformed declared sample count", Rejects<std::invalid_argument>([]{ (void)Parse("Number of X Values: 3oops\n"+Units+Rows); }));
    Check("ASTER refuses inconsistent first wavelength metadata", Rejects<std::invalid_argument>([]{ (void)Parse("First X Value: .3\n"+Units+Rows); }));
    Check("ASTER refuses inconsistent last wavelength metadata", Rejects<std::invalid_argument>([]{ (void)Parse("Last X Value: 2\n"+Units+Rows); }));
    Check("ASTER refuses swapped column declarations", Rejects<std::invalid_argument>([]{ (void)Parse("First Column: Y\nSecond Column: X\n"+Units+Rows); }));
    bool diagnosed=false;
    try { (void)Parse(Units+"0.4 20\n0.6 broken\n"); }
    catch(const std::invalid_argument& error) { diagnosed=std::string(error.what()).find("synthetic-stream:4:")!=std::string::npos; }
    Check("ASTER parse errors include provenance and actual line number", diagnosed);
    Check("ASTER detects unreadable stream state", Rejects<std::runtime_error>([]{ std::istringstream input(Units+Rows);input.setstate(std::ios::badbit);(void)SpectralReflectanceCurve::FromAsterStream(input); }));
    Check("ASTER file reader rejects an empty path", Rejects<std::invalid_argument>([]{ (void)SpectralReflectanceCurve::FromAsterFile({}); }));
    Check("ASTER file reader diagnoses a missing path", Rejects<std::runtime_error>([]{ (void)SpectralReflectanceCurve::FromAsterFile(Fixture().parent_path()/"intentionally_missing.txt"); }));
    const auto file=SpectralReflectanceCurve::FromAsterFile(Fixture());
    Check("ASTER actual file path resolves and converts the synthetic fixture", file.SampleCount()==3 && Near(file.Evaluate(532),0.4));
    Check("ASTER file reader retains measurement name and source path", file.Metadata().name=="Synthetic test curve" && file.Metadata().source.find("synthetic_reflectance.txt")!=std::string::npos);
    auto copied=file;
    Check("spectral curve copy retains sample and provenance metadata", Near(copied.Evaluate(905),file.Evaluate(905)) && copied.Metadata().source==file.Metadata().source);
    auto moved=std::move(copied);
    Check("spectral curve move retains destination data", Near(moved.Evaluate(532),0.4));
    if(copied.SampleCount()<2)Check("spectral moved from query fails safely",Rejects<std::logic_error>([&]{(void)copied.Evaluate(532);}));
}
