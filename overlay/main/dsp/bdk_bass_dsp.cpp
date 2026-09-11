
#include "bdk_bass_dsp.h"

#include <math.h>

static constexpr float BDK_PI = 3.14159265358979323846f;

BDKBassDSP::BDKBassDSP()
{
    init(48000);
}

float BDKBassDSP::clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

float BDKBassDSP::q31ToFloat(int32_t v)
{
    return (float)v * (1.0f / 2147483648.0f);
}

int32_t BDKBassDSP::floatToQ31(float v)
{
    v = clampf(v, -0.99999994f, 0.99999994f);
    return (int32_t)lrintf(v * 2147483647.0f);
}

void BDKBassDSP::Biquad::lowpass(float fs, float freq, float q)
{
    if (fs < 8000.0f) fs = 8000.0f;
    freq = BDKBassDSP::clampf(freq, 5.0f, fs * 0.45f);
    q = BDKBassDSP::clampf(q, 0.1f, 20.0f);

    const float w0 = 2.0f * BDK_PI * freq / fs;
    const float c = cosf(w0);
    const float s = sinf(w0);
    const float alpha = s / (2.0f * q);

    const float B0 = (1.0f - c) * 0.5f;
    const float B1 = 1.0f - c;
    const float B2 = B0;
    const float A0 = 1.0f + alpha;
    const float A1 = -2.0f * c;
    const float A2 = 1.0f - alpha;
    const float inv = 1.0f / A0;

    b0 = B0 * inv; b1 = B1 * inv; b2 = B2 * inv;
    a1 = A1 * inv; a2 = A2 * inv;
}

void BDKBassDSP::Biquad::highpass(float fs, float freq, float q)
{
    if (fs < 8000.0f) fs = 8000.0f;
    freq = BDKBassDSP::clampf(freq, 5.0f, fs * 0.45f);
    q = BDKBassDSP::clampf(q, 0.1f, 20.0f);

    const float w0 = 2.0f * BDK_PI * freq / fs;
    const float c = cosf(w0);
    const float s = sinf(w0);
    const float alpha = s / (2.0f * q);

    const float B0 = (1.0f + c) * 0.5f;
    const float B1 = -(1.0f + c);
    const float B2 = B0;
    const float A0 = 1.0f + alpha;
    const float A1 = -2.0f * c;
    const float A2 = 1.0f - alpha;
    const float inv = 1.0f / A0;

    b0 = B0 * inv; b1 = B1 * inv; b2 = B2 * inv;
    a1 = A1 * inv; a2 = A2 * inv;
}

void BDKBassDSP::Biquad::lowshelf(float fs, float freq, float gainDb, float slope)
{
    if (fs < 8000.0f) fs = 8000.0f;
    freq = BDKBassDSP::clampf(freq, 10.0f, fs * 0.45f);
    slope = BDKBassDSP::clampf(slope, 0.1f, 2.0f);

    const float A = powf(10.0f, gainDb / 40.0f);
    const float w0 = 2.0f * BDK_PI * freq / fs;
    const float c = cosf(w0);
    const float s = sinf(w0);
    const float term = (A + 1.0f / A) * (1.0f / slope - 1.0f) + 2.0f;
    const float alpha = 0.5f * s * sqrtf(term > 0.0f ? term : 0.0f);
    const float beta = 2.0f * sqrtf(A) * alpha;

    const float B0 = A * ((A + 1.0f) - (A - 1.0f) * c + beta);
    const float B1 = 2.0f * A * ((A - 1.0f) - (A + 1.0f) * c);
    const float B2 = A * ((A + 1.0f) - (A - 1.0f) * c - beta);
    const float A0 = (A + 1.0f) + (A - 1.0f) * c + beta;
    const float A1 = -2.0f * ((A - 1.0f) + (A + 1.0f) * c);
    const float A2 = (A + 1.0f) + (A - 1.0f) * c - beta;
    const float inv = 1.0f / A0;

    b0 = B0 * inv; b1 = B1 * inv; b2 = B2 * inv;
    a1 = A1 * inv; a2 = A2 * inv;
}

void BDKBassDSP::Biquad::peaking(float fs, float freq, float gainDb, float q)
{
    if (fs < 8000.0f) fs = 8000.0f;
    freq = BDKBassDSP::clampf(freq, 10.0f, fs * 0.45f);
    q = BDKBassDSP::clampf(q, 0.1f, 20.0f);

    const float A = powf(10.0f, gainDb / 40.0f);
    const float w0 = 2.0f * BDK_PI * freq / fs;
    const float c = cosf(w0);
    const float s = sinf(w0);
    const float alpha = s / (2.0f * q);

    const float B0 = 1.0f + alpha * A;
    const float B1 = -2.0f * c;
    const float B2 = 1.0f - alpha * A;
    const float A0 = 1.0f + alpha / A;
    const float A1 = -2.0f * c;
    const float A2 = 1.0f - alpha / A;
    const float inv = 1.0f / A0;

    b0 = B0 * inv; b1 = B1 * inv; b2 = B2 * inv;
    a1 = A1 * inv; a2 = A2 * inv;
}

void BDKBassDSP::init(uint32_t sampleRate)
{
    m_sampleRate = sampleRate ? sampleRate : 48000;
    configureFilters();
    reset();
}

void BDKBassDSP::setSampleRate(uint32_t sampleRate)
{
    if (!sampleRate || sampleRate == m_sampleRate) return;
    m_sampleRate = sampleRate;
    configureFilters();
    reset();
}


void BDKBassDSP::setProfile(BDKAudioProfile profile)
{
    if (profile < BDK_AUDIO_PROFILE_BOOMBOX3 ||
        profile > BDK_AUDIO_PROFILE_HIFI) {
        profile = BDK_AUDIO_PROFILE_BOOMBOX3;
    }

    if (profile == m_profile) return;

    m_profile = profile;
    m_enabled = (m_profile != BDK_AUDIO_PROFILE_HIFI);

    configureFilters();
    reset();
}

const char* BDKBassDSP::profileName() const
{
    switch (m_profile) {
        case BDK_AUDIO_PROFILE_BOOMBOX3:
            return "XM1 / DSP-1";
        case BDK_AUDIO_PROFILE_XTREME4:
            return "XM2 / HIFI STAGE CORSA";
        case BDK_AUDIO_PROFILE_HIFI:
            return "XM3 / HIFI PURE BYPASS";
        default:
            return "UNKNOWN";
    }
}

void BDKBassDSP::configureFilters()
{
    const float fs = (float)m_sampleRate;

    if (m_profile == BDK_AUDIO_PROFILE_XTREME4) {
        // BDK V1.0.8.2 - XM2 HIFI STAGE CORSA.
        // Corsa B 2 portas / motorista: palco por time alignment.
        // BBR 6in: protecao suave; voicing propositalmente quase neutro.
        m_hpfL.highpass(fs, 45.0f, 0.70710678f);
        m_hpfR.highpass(fs, 45.0f, 0.70710678f);

        m_bassShelfL.lowshelf(fs, 75.0f, 0.7f, 0.65f);
        m_bassShelfR.lowshelf(fs, 75.0f, 0.7f, 0.65f);
        m_bassPunchL.peaking(fs, 92.0f, 0.2f, 0.75f);
        m_bassPunchR.peaking(fs, 92.0f, 0.2f, 0.75f);

        m_subLP.lowpass(fs, 60.0f, 0.70710678f);
        m_virtualHP.highpass(fs, 60.0f, 0.70710678f);
        m_virtualLP.lowpass(fs, 120.0f, 0.70710678f);

        m_bassEnvAttack  = expf(-1.0f / (0.0090f * fs));
        m_bassEnvRelease = expf(-1.0f / (0.190f * fs));
        m_t2DcAlpha = 1.0f - expf(-2.0f * BDK_PI * 16.0f / fs);

        m_lowMidHPL.highpass(fs, 190.0f, 0.70710678f);
        m_lowMidHPR.highpass(fs, 190.0f, 0.70710678f);
        m_lowMidLPL.lowpass(fs, 480.0f, 0.70710678f);
        m_lowMidLPR.lowpass(fs, 480.0f, 0.70710678f);

        m_midHPL.highpass(fs, 850.0f, 0.70710678f);
        m_midHPR.highpass(fs, 850.0f, 0.70710678f);
        m_midLPL.lowpass(fs, 2400.0f, 0.70710678f);
        m_midLPR.lowpass(fs, 2400.0f, 0.70710678f);

        m_presenceHPL.highpass(fs, 1800.0f, 0.70710678f);
        m_presenceHPR.highpass(fs, 1800.0f, 0.70710678f);
        m_presenceLPL.lowpass(fs, 5000.0f, 0.70710678f);
        m_presenceLPR.lowpass(fs, 5000.0f, 0.70710678f);

        m_airHPL.highpass(fs, 8000.0f, 0.70710678f);
        m_airHPR.highpass(fs, 8000.0f, 0.70710678f);

        m_lowMidAttack  = expf(-1.0f / (0.012f * fs));
        m_lowMidRelease = expf(-1.0f / (0.200f * fs));
        m_midFastCoef = expf(-1.0f / (0.0035f * fs));
        m_midSlowCoef = expf(-1.0f / (0.060f * fs));
        m_presenceFastCoef = expf(-1.0f / (0.0025f * fs));
        m_presenceSlowCoef = expf(-1.0f / (0.055f * fs));
        m_airFastCoef = expf(-1.0f / (0.0016f * fs));
        m_airSlowCoef = expf(-1.0f / (0.045f * fs));

        // Protecao de excursao sem transformar o perfil em bass-boost.
        m_protectionThreshold = 0.038f;
        m_protectionDenom = 0.20f;
        m_protectionMin = 0.52f;
        m_realMixBase = 0.006f;
        m_realMixScale = 0.010f;
        m_harmonicMixBase = 0.025f;
        m_harmonicMixScale = 0.025f;
        m_h2Gain = 0.30f;
        m_h3Gain = 0.02f;

        // Detail enhancer praticamente neutro.
        m_maskThreshold = 0.060f;
        m_maskDenom = 0.24f;
        m_maskMax = 0.010f;
        m_midFactor = 0.75f;
        m_midMax = 0.008f;
        m_presenceFactor = 0.80f;
        m_presenceMax = 0.009f;
        m_airFactor = 0.70f;
        m_airMax = 0.007f;
        return;
    }

    // BDK V1.0.8.1 - HIFI BBR6 EXTENDED / Bomber BBR 6in.
    // Fs informado: 69 Hz | resposta informada: 53 Hz..20 kHz.
    m_hpfL.highpass(fs, 45.0f, 0.70710678f);
    m_hpfR.highpass(fs, 45.0f, 0.70710678f);

    m_bassShelfL.lowshelf(fs, 72.0f, 2.2f, 0.65f);
    m_bassShelfR.lowshelf(fs, 72.0f, 2.2f, 0.65f);
    m_bassPunchL.peaking(fs, 90.0f, 0.6f, 0.75f);
    m_bassPunchR.peaking(fs, 90.0f, 0.6f, 0.75f);

    m_subLP.lowpass(fs, 62.0f, 0.70710678f);
    m_virtualHP.highpass(fs, 58.0f, 0.70710678f);
    m_virtualLP.lowpass(fs, 125.0f, 0.70710678f);

    m_bassEnvAttack  = expf(-1.0f / (0.0080f * fs));
    m_bassEnvRelease = expf(-1.0f / (0.180f * fs));
    m_t2DcAlpha = 1.0f - expf(-2.0f * BDK_PI * 15.0f / fs);

    m_lowMidHPL.highpass(fs, 180.0f, 0.70710678f);
    m_lowMidHPR.highpass(fs, 180.0f, 0.70710678f);
    m_lowMidLPL.lowpass(fs, 470.0f, 0.70710678f);
    m_lowMidLPR.lowpass(fs, 470.0f, 0.70710678f);

    m_midHPL.highpass(fs, 850.0f, 0.70710678f);
    m_midHPR.highpass(fs, 850.0f, 0.70710678f);
    m_midLPL.lowpass(fs, 2300.0f, 0.70710678f);
    m_midLPR.lowpass(fs, 2300.0f, 0.70710678f);

    m_presenceHPL.highpass(fs, 1700.0f, 0.70710678f);
    m_presenceHPR.highpass(fs, 1700.0f, 0.70710678f);
    m_presenceLPL.lowpass(fs, 5200.0f, 0.70710678f);
    m_presenceLPR.lowpass(fs, 5200.0f, 0.70710678f);

    m_airHPL.highpass(fs, 6800.0f, 0.70710678f);
    m_airHPR.highpass(fs, 6800.0f, 0.70710678f);

    m_lowMidAttack  = expf(-1.0f / (0.011f * fs));
    m_lowMidRelease = expf(-1.0f / (0.190f * fs));
    m_midFastCoef = expf(-1.0f / (0.0036f * fs));
    m_midSlowCoef = expf(-1.0f / (0.060f * fs));
    m_presenceFastCoef = expf(-1.0f / (0.0024f * fs));
    m_presenceSlowCoef = expf(-1.0f / (0.055f * fs));
    m_airFastCoef = expf(-1.0f / (0.0015f * fs));
    m_airSlowCoef = expf(-1.0f / (0.042f * fs));

    // Protecao mais conservadora para cone de 6 polegadas.
    m_protectionThreshold = 0.038f;
    m_protectionDenom = 0.20f;
    m_protectionMin = 0.50f;
    m_realMixBase = 0.010f;
    m_realMixScale = 0.018f;
    m_harmonicMixBase = 0.08f;
    m_harmonicMixScale = 0.08f;
    m_h2Gain = 0.45f;
    m_h3Gain = 0.04f;

    // Detail enhancer quase neutro: preserva a assinatura HIFI.
    m_maskThreshold = 0.055f;
    m_maskDenom = 0.22f;
    m_maskMax = 0.020f;

    m_midFactor = 1.5f;
    m_midMax = 0.015f;
    m_presenceFactor = 1.6f;
    m_presenceMax = 0.018f;
    m_airFactor = 1.4f;
    m_airMax = 0.015f;
}

void BDKBassDSP::reset()
{
    m_hpfL.clear(); m_hpfR.clear();
    m_bassShelfL.clear(); m_bassShelfR.clear();
    m_bassPunchL.clear(); m_bassPunchR.clear();
    m_subLP.clear();
    m_virtualHP.clear();
    m_virtualLP.clear();

    m_lowMidHPL.clear(); m_lowMidHPR.clear();
    m_lowMidLPL.clear(); m_lowMidLPR.clear();

    m_midHPL.clear(); m_midHPR.clear();
    m_midLPL.clear(); m_midLPR.clear();

    m_presenceHPL.clear(); m_presenceHPR.clear();
    m_presenceLPL.clear(); m_presenceLPR.clear();

    m_airHPL.clear(); m_airHPR.clear();

    m_bassEnv = 0.0f;
    m_t2Dc = 0.0f;

    m_lowMidEnv = 0.0f;
    m_midFast = 0.0f;
    m_midSlow = 0.0f;
    m_presenceFast = 0.0f;
    m_presenceSlow = 0.0f;
    m_airFast = 0.0f;
    m_airSlow = 0.0f;

    m_stats = {};
}

float BDKBassDSP::softLimit(float x)
{
    const float threshold = 0.92f;
    const float ceiling = 0.998f;

    const float ax = fabsf(x);
    if (ax <= threshold) return x;

    m_stats.limiterSamples++;

    const float sign = (x < 0.0f) ? -1.0f : 1.0f;
    const float range = ceiling - threshold;
    const float u = (ax - threshold) / range;
    const float y = threshold + range * (u / (1.0f + u));
    return sign * y;
}

void BDKBassDSP::processOne(float& l, float& r)
{
    const float rawL = l;
    const float rawR = r;
    const float inPeak = fmaxf(fabsf(rawL), fabsf(rawR));
    if (inPeak > m_stats.peakInput) m_stats.peakInput = inPeak;

    // IMPORTANT: capture sub-bass before the BBR protection HPF.
    // This lets 30..55 Hz content create audible harmonics while the
    // 6-inch driver itself is protected from excessive excursion.
    const float rawMono = 0.5f * (rawL + rawR);
    const float sub = m_subLP.process(rawMono);
    const float a = fabsf(sub);

    const float envCoef = (a > m_bassEnv) ? m_bassEnvAttack : m_bassEnvRelease;
    m_bassEnv = envCoef * m_bassEnv + (1.0f - envCoef) * a;

    l = m_hpfL.process(l);
    r = m_hpfR.process(r);

    if (m_harmonicBassEnabled) {
        l = m_bassShelfL.process(l);
        r = m_bassShelfR.process(r);
        l = m_bassPunchL.process(l);
        r = m_bassPunchR.process(r);

        float protection = 1.0f;
        if (m_bassEnv > m_protectionThreshold) {
            protection = 1.0f - ((m_bassEnv - m_protectionThreshold) / m_protectionDenom);
            protection = clampf(protection, m_protectionMin, 1.0f);
        }

        // Real sub component is deliberately small. The perceived extension
        // comes mainly from harmonic synthesis.
        const float realMix = m_realMixBase + m_realMixScale * protection;
        const float realBass = sub * realMix;
        if (fabsf(realBass) > 0.00015f) m_stats.realBassSamples++;
        if (fabsf(realBass) > m_stats.peakRealBass) m_stats.peakRealBass = fabsf(realBass);

        float virtualBass = 0.0f;
        if (m_bassEnv > 0.0009f) {
            float norm = sub / (m_bassEnv * 1.12f + 1.0e-9f);
            norm = clampf(norm, -1.0f, 1.0f);
            const float t2 = 2.0f * norm * norm - 1.0f;
            const float t3 = 4.0f * norm * norm * norm - 3.0f * norm;
            m_t2Dc += m_t2DcAlpha * (t2 - m_t2Dc);
            const float h2 = (t2 - m_t2Dc) * m_bassEnv;
            const float h3 = t3 * m_bassEnv;
            const float harmonicMix = m_harmonicMixBase + m_harmonicMixScale * protection;
            virtualBass = (m_h2Gain * h2 + m_h3Gain * h3) * harmonicMix;
            virtualBass = m_virtualHP.process(virtualBass);
            virtualBass = m_virtualLP.process(virtualBass);
            if (fabsf(virtualBass) > 0.00015f) m_stats.virtualBassSamples++;
            if (fabsf(virtualBass) > m_stats.peakVirtual) m_stats.peakVirtual = fabsf(virtualBass);
        }

        l += realBass + virtualBass;
        r += realBass + virtualBass;
    }

    if (m_detailEnabled) {
        const float lmL = m_lowMidLPL.process(m_lowMidHPL.process(l));
        const float lmR = m_lowMidLPR.process(m_lowMidHPR.process(r));
        const float lmAbs = 0.5f * (fabsf(lmL) + fabsf(lmR));
        const float lmCoef = (lmAbs > m_lowMidEnv) ? m_lowMidAttack : m_lowMidRelease;
        m_lowMidEnv = lmCoef * m_lowMidEnv + (1.0f - lmCoef) * lmAbs;

        float maskCut = 0.0f;
        if (m_lowMidEnv > m_maskThreshold) {
            maskCut = (m_lowMidEnv - m_maskThreshold) / m_maskDenom;
            maskCut = clampf(maskCut, 0.0f, 1.0f) * m_maskMax;
        }
        if (maskCut > 0.002f) {
            l -= lmL * maskCut;
            r -= lmR * maskCut;
            m_stats.maskReductionSamples++;
        }

        const float midL = m_midLPL.process(m_midHPL.process(l));
        const float midR = m_midLPR.process(m_midHPR.process(r));
        const float midAbs = 0.5f * (fabsf(midL) + fabsf(midR));
        m_midFast = m_midFastCoef*m_midFast + (1.0f-m_midFastCoef)*midAbs;
        m_midSlow = m_midSlowCoef*m_midSlow + (1.0f-m_midSlowCoef)*midAbs;
        const float midTransient = fmaxf(0.0f, m_midFast - m_midSlow);
        const float midMix = clampf(midTransient * m_midFactor, 0.0f, m_midMax);
        if (midMix > 0.0012f) {
            l += midL * midMix;
            r += midR * midMix;
            m_stats.midTransientSamples++;
        }

        const float prL = m_presenceLPL.process(m_presenceHPL.process(l));
        const float prR = m_presenceLPR.process(m_presenceHPR.process(r));
        const float prAbs = 0.5f * (fabsf(prL) + fabsf(prR));
        m_presenceFast = m_presenceFastCoef*m_presenceFast + (1.0f-m_presenceFastCoef)*prAbs;
        m_presenceSlow = m_presenceSlowCoef*m_presenceSlow + (1.0f-m_presenceSlowCoef)*prAbs;
        const float prTransient = fmaxf(0.0f, m_presenceFast - m_presenceSlow);
        const float prMix = clampf(prTransient * m_presenceFactor, 0.0f, m_presenceMax);
        if (prMix > 0.0014f) {
            l += prL * prMix;
            r += prR * prMix;
            m_stats.presenceTransientSamples++;
        }

        const float airL = m_airHPL.process(l);
        const float airR = m_airHPR.process(r);
        const float airAbs = 0.5f * (fabsf(airL) + fabsf(airR));
        m_airFast = m_airFastCoef*m_airFast + (1.0f-m_airFastCoef)*airAbs;
        m_airSlow = m_airSlowCoef*m_airSlow + (1.0f-m_airSlowCoef)*airAbs;
        const float airTransient = fmaxf(0.0f, m_airFast - m_airSlow);
        const float airMix = clampf(airTransient * m_airFactor, 0.0f, m_airMax);
        if (airMix > 0.0011f) {
            l += airL * airMix;
            r += airR * airMix;
            m_stats.airTransientSamples++;
        }

        const float detailPeak = fmaxf(fabsf(midL*midMix), fmaxf(fabsf(prL*prMix), fabsf(airL*airMix)));
        if (detailPeak > m_stats.peakDetail) m_stats.peakDetail = detailPeak;
    }

    l *= m_preamp;
    r *= m_preamp;
    l = softLimit(l);
    r = softLimit(r);
    const float outPeak = fmaxf(fabsf(l), fabsf(r));
    if (outPeak > m_stats.peakOutput) m_stats.peakOutput = outPeak;
}


void BDKBassDSP::processBlock(int32_t* interleavedStereo, size_t frames)
{
    if (!interleavedStereo || frames == 0) return;

    if (!m_enabled) {
        m_stats.framesProcessed += (uint32_t)frames;
        return;
    }

    for (size_t i = 0; i < frames; ++i) {
        float l = q31ToFloat(interleavedStereo[i * 2]);
        float r = q31ToFloat(interleavedStereo[i * 2 + 1]);

        processOne(l, r);

        interleavedStereo[i * 2] = floatToQ31(l);
        interleavedStereo[i * 2 + 1] = floatToQ31(r);
    }

    m_stats.framesProcessed += (uint32_t)frames;
}
