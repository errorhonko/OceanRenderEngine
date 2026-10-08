#include "IdealUniformAdc.h"
#include "LidarVoltageWaveform.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
void Check(const char* name, bool ok) {
    if (!ok) throw std::runtime_error(std::string("[FAIL] ") + name);
    std::cout << "[PASS] " << name << '\n';
}
template<class E, class F> bool Throws(F f) {
    try { f(); } catch (const E&) { return true; }
    return false;
}
}

void RunIdealUniformAdcAcceptanceTests() {
    const IdealUniformAdc adc(12, 0, 1);
    const double lsb = 1.0 / 4096;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    const double maximum = std::numeric_limits<double>::max();
    Check("ADC stores bit count", adc.BitCount() == 12);
    Check("ADC maximum code is 4095", adc.MaximumCode() == 4095);
    Check("ADC LSB uses full span divided by 4096", adc.LsbVoltageV() == lsb);
    const auto lo = adc.Quantize(0), hi = adc.Quantize(1);
    Check("ADC lower endpoint code zero without underrange", lo.code == 0 && !lo.belowRange && !lo.aboveRange);
    Check("ADC upper endpoint maximum without overrange", hi.code == 4095 && !hi.belowRange && !hi.aboveRange);
    const auto under = adc.Quantize(-.1), over = adc.Quantize(1.2);
    Check("ADC strict underrange saturates and flags", under.code == 0 && under.belowRange && !under.aboveRange);
    Check("ADC strict overrange saturates and flags", over.code == 4095 && !over.belowRange && over.aboveRange);
    Check("ADC quarter volt gives code 1024", adc.Quantize(.25).code == 1024);
    Check("ADC midpoint gives code 2048", adc.Quantize(.5).code == 2048);
    Check("ADC top in-range code is not overrange", adc.Quantize(1 - .25 * lsb).code == 4095 && !adc.Quantize(1 - .25 * lsb).aboveRange);
    Check("ADC code zero reconstructs first interval center", adc.CodeCenterVoltageV(0) == .5 * lsb);
    Check("ADC maximum reconstructs last interval center", adc.CodeCenterVoltageV(4095) == 1 - .5 * lsb);
    Check("ADC quarter volt midpoint reconstruction", adc.CodeCenterVoltageV(1024) == .2501220703125);
    // Binary-exact range: exercise every interior threshold and its adjacent doubles.
    bool thresholds = true, centers = true;
    for (std::uint32_t c = 0; c <= adc.MaximumCode(); ++c) {
        const auto sample = adc.Quantize(adc.CodeCenterVoltageV(c));
        centers = centers && sample.code == c && !sample.belowRange && !sample.aboveRange;
        if (c == 0) continue;
        const double edge = c * lsb;
        thresholds = thresholds && adc.Quantize(edge).code == c
            && adc.Quantize(std::nextafter(edge, -inf)).code == c - 1
            && adc.Quantize(std::nextafter(edge, inf)).code == c;
    }
    Check("ADC all 4096 code centers round trip", centers);
    Check("ADC all 4095 interior thresholds and neighbors", thresholds);
    Check("ADC immediately below minimum flags", adc.Quantize(std::nextafter(0.0, -inf)).belowRange);
    Check("ADC immediately above maximum flags", adc.Quantize(std::nextafter(1.0, inf)).aboveRange);
    const IdealUniformAdc bipolar(12, -1, 1);
    Check("ADC bipolar negative input supported", bipolar.Quantize(-.5).code == 1024);
    Check("ADC bipolar zero is middle code", bipolar.Quantize(0).code == 2048);
    bool monotonic = true, errorBound = true;
    std::uint32_t previous = 0;
    for (int i = 0; i <= 65536; ++i) {
        const double v = -1 + 2.0 * i / 65536;
        const auto sample = bipolar.Quantize(v);
        monotonic = monotonic && sample.code >= previous && sample.code <= bipolar.MaximumCode();
        previous = sample.code;
        errorBound = errorBound && !sample.belowRange && !sample.aboveRange
            && std::abs(bipolar.CodeCenterVoltageV(sample.code) - v) <= .5 * bipolar.LsbVoltageV();
    }
    Check("ADC dense bipolar sweep is monotonic", monotonic);
    Check("ADC dense in-range sweep error at most half LSB", errorBound);
    Check("ADC clipped input is not covered by half LSB bound", std::abs(adc.CodeCenterVoltageV(over.code) - 1.2) > .5 * lsb);
    const IdealUniformAdc oneBit(1, 0, 1), twentyFourBit(24, 0, 1);
    Check("ADC one bit endpoints midpoint and centers", oneBit.MaximumCode() == 1 && oneBit.Quantize(.5).code == 1 && oneBit.CodeCenterVoltageV(0) == .25 && oneBit.CodeCenterVoltageV(1) == .75);
    Check("ADC 24 bit code range and last center", twentyFourBit.MaximumCode() == 16777215 && twentyFourBit.Quantize(twentyFourBit.CodeCenterVoltageV(16777215)).code == 16777215);
    Check("ADC rejects bits below one", Throws<std::invalid_argument>([]{ IdealUniformAdc bad(0, 0, 1); }));
    Check("ADC rejects bits above 24", Throws<std::invalid_argument>([]{ IdealUniformAdc bad(25, 0, 1); }));
    Check("ADC rejects equal voltage endpoints", Throws<std::invalid_argument>([]{ IdealUniformAdc bad(12, 1, 1); }));
    Check("ADC rejects reversed voltage endpoints", Throws<std::invalid_argument>([]{ IdealUniformAdc bad(12, 1, 0); }));
    Check("ADC rejects NaN range endpoint", Throws<std::invalid_argument>([&]{ IdealUniformAdc bad(12, nan, 1); }));
    Check("ADC rejects infinite range endpoint", Throws<std::invalid_argument>([&]{ IdealUniformAdc bad(12, 0, inf); }));
    Check("ADC rejects overflowing span", Throws<std::invalid_argument>([&]{ IdealUniformAdc bad(12, -maximum, maximum); }));
    Check("ADC rejects underflowing LSB", Throws<std::invalid_argument>([]{ IdealUniformAdc bad(24, 0, std::numeric_limits<double>::denorm_min()); }));
    Check("ADC rejects NaN input", Throws<std::invalid_argument>([&]{ adc.Quantize(nan); }));
    Check("ADC rejects positive infinity input", Throws<std::invalid_argument>([&]{ adc.Quantize(inf); }));
    Check("ADC rejects negative infinity input", Throws<std::invalid_argument>([&]{ adc.Quantize(-inf); }));
    Check("ADC rejects out of range reconstruction code", Throws<std::out_of_range>([&]{ adc.CodeCenterVoltageV(4096); }));
    // Existing current -> voltage -> code chain; these are bin-average values,
    // not an independent instantaneous sampling clock or aperture model.
    const LidarAnalogWaveform current({10e-9, 2e-9, 5}, {0, 25e-6, 50e-6, 100e-6, 120e-6});
    const auto voltage = LidarVoltageWaveform::FromCurrentWaveform(current, IdealTransimpedanceAmplifier(10e3));
    const auto before = voltage.AverageVoltageBinsV();
    const std::uint32_t expected[] = {0, 1024, 2048, 4095, 4095};
    bool chain = true;
    for (std::size_t i = 0; i < 5; ++i) {
        const auto sample = adc.Quantize(before[i]);
        chain = chain && sample.code == expected[i] && !sample.belowRange && sample.aboveRange == (i == 4);
    }
    Check("ADC quantizes existing voltage waveform and flags clipping", chain);
    Check("ADC quantization leaves voltage waveform unchanged", voltage.AverageVoltageBinsV() == before);
}
