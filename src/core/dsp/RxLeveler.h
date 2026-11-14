#pragma once

// =================================================================
// src/core/dsp/RxLeveler.h  (NereusSDR)
// =================================================================
//
// Receive loudness rider. It sits after AGC, on the demodulated audio,
// and lifts quiet stations toward one listening level. It does not
// replace AGC and it does not share the transmit leveler.
//
// Off leaves the samples untouched. Auto is the speech preset. Custom
// uses the operator's target, boost cap, attack, release and pause hold.
// A fast ceiling after the slow gain stops that boost from clipping.
//
// NereusSDR-original. Brian Sutton (W4KZO), AI-assisted via xAI Grok.
// =================================================================

#include "core/WdspTypes.h"

#include <atomic>

namespace NereusSDR {

// The numbers the DSP thread reads once per block. Off adds no gain.
// Auto rides speech with the preset below. Custom uses the operator's
// knobs, still capped at kMaxBoostDbMax.
struct RxLevelerRequest {
    RxLevelerMode mode{RxLevelerMode::Off};
    double targetDb{-18.0};
    double maxBoostDb{12.0};
    double attackMs{400.0};
    double releaseMs{1500.0};
    double pauseHoldMs{600.0};
};

// Upward speech rider plus a fast output ceiling.
//
// process() is called on the receive DSP thread only. boostDb() is safe
// to read from the UI thread.
class RxLeveler {
public:
    // Auto preset: a moderate cap, a slow ride, and a pause long enough
    // to cover a gap between words. The custom ceiling is 24 dB.
    static constexpr double kAutoTargetDb     = -18.0;
    static constexpr double kAutoMaxBoostDb   =  12.0;
    static constexpr double kAutoAttackMs     = 400.0;
    static constexpr double kAutoReleaseMs    = 1500.0;
    static constexpr double kAutoPauseHoldMs  = 600.0;

    static constexpr double kTargetDbMin      = -40.0;
    static constexpr double kTargetDbMax      =  -6.0;
    static constexpr double kMaxBoostDbMin    =   0.0;
    static constexpr double kMaxBoostDbMax    =  24.0;
    static constexpr double kAttackMsMin      =  20.0;
    static constexpr double kAttackMsMax      = 2000.0;
    static constexpr double kReleaseMsMin     =  50.0;
    static constexpr double kReleaseMsMax     = 5000.0;
    static constexpr double kPauseHoldMsMin   =   0.0;
    static constexpr double kPauseHoldMsMax   = 2000.0;

    // Voice modes only. CW, FM and the digital modes are left alone:
    // a speech pause-hold is the wrong shape for them, and DIGU/DIGL
    // feed decoders through the same audio tap.
    static bool modeAllows(DSPMode mode);

    void reset();

    // Returns false when the block was left untouched (Off, or no samples).
    bool process(float* left, float* right, int frames, double sampleRate,
                 const RxLevelerRequest& request);

    double boostDb() const;

private:
    enum class Phase { Riding, Holding, Releasing };

    double m_power{0.0};
    double m_speechPeakDb{-120.0};
    double m_gainDb{0.0};
    double m_limiter{1.0};
    double m_holdLeft{0.0};
    Phase  m_phase{Phase::Releasing};
    std::atomic<float> m_publishedBoost{0.0f};
};

}  // namespace NereusSDR
