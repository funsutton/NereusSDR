// no-port-check: NereusSDR-original test.
//
// The receive leveler rides station loudness after AGC. Off is a hard
// bypass. Auto lifts a quiet tone toward the speech preset and stops at
// the cap. A sudden peak is held to full scale. A pause keeps the gain,
// then lets it go. CW and DIGU are not voice modes.

#include <QtTest/QtTest>

#include "core/dsp/RxLeveler.h"

#include <cmath>
#include <vector>

using namespace NereusSDR;

namespace {

RxLevelerRequest automatic()
{
    RxLevelerRequest request;
    request.mode = RxLevelerMode::Auto;
    return request;
}

void fill(std::vector<float>& left, std::vector<float>& right, int frames, float amplitude)
{
    left.assign(static_cast<std::size_t>(frames), amplitude);
    right.assign(static_cast<std::size_t>(frames), amplitude);
}

bool pump(RxLeveler& leveler, float amplitude, int frames, const RxLevelerRequest& request)
{
    std::vector<float> left;
    std::vector<float> right;
    fill(left, right, frames, amplitude);
    return leveler.process(left.data(), right.data(), frames, 48000.0, request);
}

}  // namespace

class TestRxLeveler : public QObject {
    Q_OBJECT

private slots:
    void offLeavesTheBlockUntouched();
    void voiceModesOnly();
    void quietToneRisesToTheCap();
    void loudToneIsNotCut();
    void aPeakIsCaught();
    void aPauseHoldsThenLetsGo();
    void silenceDoesNotLift();
};

void TestRxLeveler::offLeavesTheBlockUntouched()
{
    RxLeveler leveler;
    std::vector<float> left{0.2f, -0.4f, 1.5f, 0.0f};
    std::vector<float> right{0.2f, -0.4f, 1.5f, 0.0f};
    const auto originalL = left;
    const auto originalR = right;
    RxLevelerRequest request;
    request.mode = RxLevelerMode::Off;
    QVERIFY(!leveler.process(left.data(), right.data(), 4, 48000.0, request));
    QCOMPARE(left, originalL);
    QCOMPARE(right, originalR);
    QCOMPARE(leveler.boostDb(), 0.0);
}

void TestRxLeveler::voiceModesOnly()
{
    QVERIFY(RxLeveler::modeAllows(DSPMode::USB));
    QVERIFY(RxLeveler::modeAllows(DSPMode::LSB));
    QVERIFY(RxLeveler::modeAllows(DSPMode::AM));
    QVERIFY(RxLeveler::modeAllows(DSPMode::SAM));
    QVERIFY(RxLeveler::modeAllows(DSPMode::DSB));
    QVERIFY(!RxLeveler::modeAllows(DSPMode::DIGU));
    QVERIFY(!RxLeveler::modeAllows(DSPMode::DIGL));
    QVERIFY(!RxLeveler::modeAllows(DSPMode::CWL));
    QVERIFY(!RxLeveler::modeAllows(DSPMode::CWU));
    QVERIFY(!RxLeveler::modeAllows(DSPMode::FM));
    QVERIFY(!RxLeveler::modeAllows(DSPMode::DRM));
    QVERIFY(!RxLeveler::modeAllows(DSPMode::SPEC));
    QVERIFY(!RxLeveler::modeAllows(DSPMode::RADE_U));
}

void TestRxLeveler::quietToneRisesToTheCap()
{
    RxLeveler leveler;
    const RxLevelerRequest request = automatic();
    QVERIFY(pump(leveler, 0.01f, 48000 * 3, request));
    QVERIFY(leveler.boostDb() > 11.0);
    QVERIFY(leveler.boostDb() <= RxLeveler::kAutoMaxBoostDb + 0.05);

    std::vector<float> left;
    std::vector<float> right;
    fill(left, right, 480, 0.01f);
    QVERIFY(leveler.process(left.data(), right.data(), 480, 48000.0, request));
    QVERIFY(left.back() > 0.03f);
    QVERIFY(left.back() < 0.05f);
    QVERIFY(left.back() <= 1.0f);
}

void TestRxLeveler::loudToneIsNotCut()
{
    RxLeveler leveler;
    QVERIFY(pump(leveler, 0.5f, 48000 * 4, automatic()));
    QVERIFY(leveler.boostDb() < 1.0);

    std::vector<float> left;
    std::vector<float> right;
    fill(left, right, 480, 0.5f);
    QVERIFY(leveler.process(left.data(), right.data(), 480, 48000.0, automatic()));
    QVERIFY(std::abs(left.back() - 0.5f) < 0.03f);
}

void TestRxLeveler::aPeakIsCaught()
{
    RxLeveler leveler;
    RxLevelerRequest request;
    request.mode = RxLevelerMode::Custom;
    request.targetDb = -6.0;
    request.maxBoostDb = 24.0;
    request.attackMs = 20.0;
    request.releaseMs = 1500.0;
    request.pauseHoldMs = 600.0;
    QVERIFY(pump(leveler, 0.01f, 48000, request));
    QVERIFY(leveler.boostDb() > 23.0);

    std::vector<float> left(480, 0.2f);
    std::vector<float> right(480, 0.2f);
    QVERIFY(leveler.process(left.data(), right.data(), 480, 48000.0, request));
    for (float sample : left) {
        QVERIFY(sample <= 1.0f + 1e-4f);
    }
    QVERIFY(left.front() > 0.9f);
}

void TestRxLeveler::aPauseHoldsThenLetsGo()
{
    RxLeveler leveler;
    const RxLevelerRequest request = automatic();
    QVERIFY(pump(leveler, 0.01f, 48000 * 2, request));
    const double held = leveler.boostDb();
    QVERIFY(held > 11.0);

    QVERIFY(pump(leveler, 0.0f, 48000 / 5, request));
    QVERIFY(leveler.boostDb() > held - 0.5);

    QVERIFY(pump(leveler, 0.0f, 48000 * 5, request));
    QVERIFY(leveler.boostDb() < 1.0);
}

void TestRxLeveler::silenceDoesNotLift()
{
    RxLeveler leveler;
    QVERIFY(pump(leveler, 1.0e-4f, 48000 * 2, automatic()));
    QVERIFY(leveler.boostDb() < 0.5);
}

QTEST_APPLESS_MAIN(TestRxLeveler)
#include "tst_rx_leveler.moc"
