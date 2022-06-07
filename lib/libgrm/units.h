#pragma once

#include "maths.h"
#include "numtypes.h"

namespace grm {

struct Units {
  static constexpr f freqOfMidi0 =
      8.1757989156437073336828122976032719176391831356887447145056402949_f;

  static f semitonesToRatio(f p) { return Math::fast_exp2(p / 12_f); }
  static f ratioToSemitones(f ratio) { return f(Math::fast_log2(ratio)) * 12_f; }

  static f midiToHz(f midiNote) { return freqOfMidi0 * semitonesToRatio(midiNote); }

  static f HzToMidi(f frequency) { return 12_f * f(Math::fast_log2(frequency / freqOfMidi0)); }

  // Computes the number of semitones between freq1 and freq2 (in Hz or any other unit of frequency)
  static s9_23 freqDiffToMidi(f freq1, f freq2) {
    assert(freq1 > 0_f);
    assert(freq2 > 0_f);
    return Math::fast_log2(freq1 / freq2).abs() * 12;
  }

  // midiDiffToFreq(a, b) = midiToHz(note1) - midiToHz(note2)
  static f midiDiffToFreq(f note1, f note2) {
    return freqOfMidi0 * (semitonesToRatio(note1) - semitonesToRatio(note2));
  }

  // for amp < 1/(2^shift), returns -inf
  template <int shift = 24>
  static f ampToDb(f amp) {
    assert(amp >= 0_f);
    constexpr f minAmp = 1_f / f(1 << shift);
    if (amp < minAmp) return minus_infinity;
    return 20_f * Math::fast_log10<shift>(amp);
  }

  // for db < ampToDb(1/2^shift), returns 0
  // shift=24 -> minDb=-144.5
  // shift=20 -> minDb=-120.5
  // shift=16 -> minDb=-96.3
  // shift=15 -> minDb=-90.3
  template <int shift = 24>
  static f dbToAmp(f db) {
    constexpr f minAmp = 1_f / f(1 << shift);
    f amp = Math::fast_exp10(db * 0.05_f);
    return amp < minAmp ? 0_f : amp;
  }
};
}  // namespace grm
