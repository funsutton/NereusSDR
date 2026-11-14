// =================================================================
// src/core/dsp/RxLeveler.cpp  (NereusSDR)
// =================================================================
//
// Receive loudness rider. See RxLeveler.h.
//
// NereusSDR-original. Brian Sutton (W4KZO), AI-assisted via xAI Grok.
// =================================================================

#include "RxLeveler.h"

#include <algorithm>
#include <cmath>

namespace NereusSDR {

namespace {

constexpr double kMeasureMs = 30.0;
constexpr double kPauseDropDb = 12.0;
constexpr double kSpeechFloorDb = -55.0;
constexpr double kSpeechPeakReleaseMs = 2000.0;
// AGC already aims peaks at full scale. The ceiling stops the rider's
// extra gain from going past that. It does not trim a peak the AGC
// already left at or under full scale.
constexpr double kSafetyCeiling = 1.0;
constexpr double kLimiterReleaseMs = 80.0;
constexpr double kPowerFloor = 1e-12;

double onePole(double tauMs, double sampleRate)
{
    const double samples = std::max(1.0, tauMs * 0.001 * sampleRate);
    return 1.0 - std::exp(-1.0 / samples);
}

double clampDb(double value, double lo, double hi)
{
    return std::clamp(value, lo, hi);
}

bool finitePair(float left, float right)
{
    return std::isfinite(left) && std::isfinite(right);
}

}  // namespace

bool RxLeveler::modeAllows(DSPMode mode)
{
    switch (mode) {
    case DSPMode::LSB:
    case DSPMode::USB:
    case DSPMode::DSB:
    case DSPMode::AM:
    case DSPMode::SAM:
        return true;
    default:
        return false;
    }
}

void RxLeveler::reset()
{
    m_power = 0.0;
    m_speechPeakDb = -120.0;
    m_gainDb = 0.0;
    m_limiter = 1.0;
    m_holdLeft = 0.0;
    m_phase = Phase::Releasing;
    m_publishedBoost.store(0.0f, std::memory_order_relaxed);
}

double RxLeveler::boostDb() const
{
    return static_cast<double>(m_publishedBoost.load(std::memory_order_relaxed));
}

bool RxLeveler::process(float* left, float* right, int frames, double sampleRate,
                        const RxLevelerRequest& request)
{
    if (left == nullptr || right == nullptr || frames <= 0 || sampleRate <= 0.0
        || request.mode == RxLevelerMode::Off) {
        if (request.mode == RxLevelerMode::Off) {
            reset();
        }
        return false;
    }

    const bool automatic = request.mode == RxLevelerMode::Auto;
    const double targetDb = automatic
        ? kAutoTargetDb
        : clampDb(request.targetDb, kTargetDbMin, kTargetDbMax);
    const double maxBoostDb = automatic
        ? kAutoMaxBoostDb
        : clampDb(request.maxBoostDb, kMaxBoostDbMin, kMaxBoostDbMax);
    const double attackMs = automatic
        ? kAutoAttackMs
        : clampDb(request.attackMs, kAttackMsMin, kAttackMsMax);
    const double releaseMs = automatic
        ? kAutoReleaseMs
        : clampDb(request.releaseMs, kReleaseMsMin, kReleaseMsMax);
    const double pauseHoldMs = automatic
        ? kAutoPauseHoldMs
        : clampDb(request.pauseHoldMs, kPauseHoldMsMin, kPauseHoldMsMax);

    const double measureCoeff = onePole(kMeasureMs, sampleRate);
    const double peakCoeff = onePole(kSpeechPeakReleaseMs, sampleRate);
    const double attackCoeff = onePole(attackMs, sampleRate);
    const double releaseCoeff = onePole(releaseMs, sampleRate);
    const double limiterCoeff = onePole(kLimiterReleaseMs, sampleRate);
    const double holdSamples = std::max(0.0, pauseHoldMs * 0.001 * sampleRate);

    for (int i = 0; i < frames; ++i) {
        float sampleL = left[i];
        float sampleR = right[i];
        if (!finitePair(sampleL, sampleR)) {
            sampleL = 0.0f;
            sampleR = 0.0f;
        }

        const double power = 0.5 * (static_cast<double>(sampleL) * sampleL
                                   + static_cast<double>(sampleR) * sampleR);
        m_power += measureCoeff * (power - m_power);
        if (m_power < kPowerFloor) {
            m_power = kPowerFloor;
        }
        const double levelDb = 10.0 * std::log10(m_power);

        const bool speech = levelDb > kSpeechFloorDb
            && levelDb > m_speechPeakDb - kPauseDropDb;
        if (speech) {
            m_phase = Phase::Riding;
            m_holdLeft = 0.0;
        } else if (m_phase == Phase::Riding) {
            m_phase = Phase::Holding;
            m_holdLeft = holdSamples;
        }
        if (m_phase == Phase::Holding) {
            if (m_holdLeft > 0.0) {
                m_holdLeft -= 1.0;
            } else {
                m_phase = Phase::Releasing;
            }
        }

        if (levelDb > m_speechPeakDb) {
            m_speechPeakDb = levelDb;
        } else {
            m_speechPeakDb += peakCoeff * (levelDb - m_speechPeakDb);
        }

        if (m_phase != Phase::Holding) {
            double desired = 0.0;
            double coeff = releaseCoeff;
            if (m_phase == Phase::Riding) {
                desired = clampDb(targetDb - levelDb, 0.0, maxBoostDb);
                coeff = desired > m_gainDb ? attackCoeff : releaseCoeff;
            }
            m_gainDb += coeff * (desired - m_gainDb);
            if (m_gainDb < 0.0) {
                m_gainDb = 0.0;
            } else if (m_gainDb > maxBoostDb) {
                m_gainDb = maxBoostDb;
            }
        }

        const double boosted = std::pow(10.0, m_gainDb / 20.0);
        double outL = static_cast<double>(sampleL) * boosted;
        double outR = static_cast<double>(sampleR) * boosted;
        const double peak = std::max(std::abs(outL), std::abs(outR));
        const double room = peak > kPowerFloor ? (kSafetyCeiling / peak) : 1.0;
        const double capped = room < 1.0 ? room : 1.0;
        if (capped < m_limiter) {
            m_limiter = capped;
        } else {
            m_limiter += limiterCoeff * (1.0 - m_limiter);
            if (m_limiter > capped) {
                m_limiter = capped;
            }
        }
        if (m_limiter < 0.0) {
            m_limiter = 0.0;
        }
        outL *= m_limiter;
        outR *= m_limiter;
        left[i] = static_cast<float>(outL);
        right[i] = static_cast<float>(outR);
    }

    m_publishedBoost.store(static_cast<float>(m_gainDb), std::memory_order_relaxed);
    return true;
}

}  // namespace NereusSDR
