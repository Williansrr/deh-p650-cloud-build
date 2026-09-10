
#pragma once

#include <stddef.h>
#include <stdint.h>

enum BDKAudioProfile : uint8_t {
    BDK_AUDIO_PROFILE_BOOMBOX3 = 1,
    BDK_AUDIO_PROFILE_XTREME4  = 2,
    BDK_AUDIO_PROFILE_HIFI     = 3
};

struct BDKBassDSPStats {
    uint32_t framesProcessed;
    uint32_t limiterSamples;

    // Harmonic bass
    uint32_t virtualBassSamples;
    uint32_t realBassSamples;

    // Detail enhancer
    uint32_t maskReductionSamples;
    uint32_t midTransientSamples;
    uint32_t presenceTransientSamples;
    uint32_t airTransientSamples;

    float peakInput;
    float peakOutput;
    float peakVirtual;
    float peakRealBass;
    float peakDetail;
};

class BDKBassDSP {
public:
    BDKBassDSP();

    void init(uint32_t sampleRate);
    void setSampleRate(uint32_t sampleRate);
    void reset();

    void setEnabled(bool enabled) { m_enabled = enabled; }
    bool isEnabled() const { return m_enabled; }

    void setHarmonicBassEnabled(bool enabled) { m_harmonicBassEnabled = enabled; }
    bool harmonicBassEnabled() const { return m_harmonicBassEnabled; }

    void setDetailEnabled(bool enabled) { m_detailEnabled = enabled; }
    bool detailEnabled() const { return m_detailEnabled; }

    void setProfile(BDKAudioProfile profile);
    BDKAudioProfile profile() const { return m_profile; }
    const char* profileName() const;

    uint32_t sampleRate() const { return m_sampleRate; }

    void processBlock(int32_t* interleavedStereo, size_t frames);
    BDKBassDSPStats stats() const { return m_stats; }

private:
    struct Biquad {
        float b0 = 1.0f, b1 = 0.0f, b2 = 0.0f;
        float a1 = 0.0f, a2 = 0.0f;
        float z1 = 0.0f, z2 = 0.0f;

        inline float process(float x) {
            float y = b0 * x + z1;
            z1 = b1 * x - a1 * y + z2;
            z2 = b2 * x - a2 * y;
            return y;
        }

        void clear() { z1 = z2 = 0.0f; }
        void lowpass(float fs, float freq, float q);
        void highpass(float fs, float freq, float q);
        void lowshelf(float fs, float freq, float gainDb, float slope);
        void peaking(float fs, float freq, float gainDb, float q);
    };

    static float q31ToFloat(int32_t v);
    static int32_t floatToQ31(float v);
    static float clampf(float v, float lo, float hi);

    inline float softLimit(float x);
    inline void processOne(float& l, float& r);

    void configureFilters();

    uint32_t m_sampleRate = 48000;
    bool m_enabled = true;
    bool m_harmonicBassEnabled = true;
    bool m_detailEnabled = true;

    // ----------------------------------------------------------
    // Harmonic bass
    // ----------------------------------------------------------
    Biquad m_hpfL, m_hpfR;
    Biquad m_bassShelfL, m_bassShelfR;
    Biquad m_bassPunchL, m_bassPunchR;
    Biquad m_subLP;
    Biquad m_virtualHP;
    Biquad m_virtualLP;

    float m_bassEnv = 0.0f;
    float m_bassEnvAttack = 0.0f;
    float m_bassEnvRelease = 0.0f;
    float m_t2Dc = 0.0f;
    float m_t2DcAlpha = 0.0f;

    // ----------------------------------------------------------
    // Detail enhancer
    // ----------------------------------------------------------
    Biquad m_lowMidHPL, m_lowMidHPR;
    Biquad m_lowMidLPL, m_lowMidLPR;

    Biquad m_midHPL, m_midHPR;
    Biquad m_midLPL, m_midLPR;

    Biquad m_presenceHPL, m_presenceHPR;
    Biquad m_presenceLPL, m_presenceLPR;

    Biquad m_airHPL, m_airHPR;

    float m_lowMidEnv = 0.0f;
    float m_lowMidAttack = 0.0f;
    float m_lowMidRelease = 0.0f;

    float m_midFast = 0.0f;
    float m_midSlow = 0.0f;
    float m_midFastCoef = 0.0f;
    float m_midSlowCoef = 0.0f;

    float m_presenceFast = 0.0f;
    float m_presenceSlow = 0.0f;
    float m_presenceFastCoef = 0.0f;
    float m_presenceSlowCoef = 0.0f;

    float m_airFast = 0.0f;
    float m_airSlow = 0.0f;
    float m_airFastCoef = 0.0f;
    float m_airSlowCoef = 0.0f;

    BDKAudioProfile m_profile = BDK_AUDIO_PROFILE_BOOMBOX3;

    float m_protectionThreshold = 0.043f;
    float m_protectionDenom = 0.22f;
    float m_protectionMin = 0.43f;
    float m_realMixBase = 0.014f;
    float m_realMixScale = 0.028f;
    float m_harmonicMixBase = 0.46f;
    float m_harmonicMixScale = 0.26f;
    float m_h2Gain = 0.86f;
    float m_h3Gain = 0.10f;

    float m_maskThreshold = 0.044f;
    float m_maskDenom = 0.18f;
    float m_maskMax = 0.075f;

    float m_midFactor = 4.2f;
    float m_midMax = 0.055f;
    float m_presenceFactor = 5.0f;
    float m_presenceMax = 0.070f;
    float m_airFactor = 6.3f;
    float m_airMax = 0.060f;

    // -4.5 dB headroom in the two JBL voicing profiles.
    // XM3 / Hi-Fi bypasses DSP entirely.
    const float m_preamp = 0.5956621435f;

    BDKBassDSPStats m_stats{};
};
