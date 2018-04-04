#pragma once

struct Random {
  static inline uint32_t state() { return state_; }
  static inline uint32_t Int() {
    state_ = state_ * 1664525L + 1013904223L;
    return state();
  }
  static inline int16_t Int16() { return Int() >> 16; }
  static inline float Float() { return (float)Int() / 4294967296.0f; }
private:
  static uint32_t state_;
};

uint32_t Random::state_ = 0x21;

constexpr int ipow(int a, int b) {
  return b==0 ? 1 : a * ipow(a, b-1);
}


template<typename T, int SIZE>
class Buffer {
  T buffer_[SIZE] = {0};
  uint32_t cursor_ = SIZE;
public:
  void Write(T x) {
    buffer_[++cursor_ % SIZE] = x;
  }
  T Read(uint32_t n) {
    return buffer_[(cursor_ - n) % SIZE];
  }
  T ReadLast() {
    return buffer_[(cursor_+1) % SIZE];
  }
};


template<int SIZE>
struct Buffer<float, SIZE> {
  void Write(float x) {
    buffer_[++cursor_ % SIZE] = x;
  }
  float Read(int n) {
    return buffer_[(cursor_ - n) % SIZE];
  }
  float ReadLast() {
    return buffer_[(cursor_+1) % SIZE];
  }

  float ReadLinear(float x) {
    uint32_t index = static_cast<uint32_t>(x);
    float fractional = x - static_cast<float>(index);
    float x1 = buffer_[(cursor_ - index+1) % SIZE];
    float x2 = buffer_[(cursor_ - index) % SIZE];
    return x1 + (x2 - x1) * fractional;
  }
private:
  float buffer_[SIZE] = {0};
  uint32_t cursor_ = SIZE;
};

template<int coef_num, int coef_denom>
struct OnePoleLp {
  void Process(float input, float *output) {
    state_ += (input - state_) * coef;
    *output = state_;
  }
private:
  float state_;
  static constexpr float coef = (float)coef_num / (float)coef_denom;
};

// N number of stages, R decimation rate
// careful: gain must be <= 2^16
template<int N, int R>
class CicDecimator {
  int32_t hi[N] = {0};
  int32_t hc[N] = {0};
  static constexpr int gain = ipow(R, N);
public:
  // reads [R*size] input samples, writes [size] output samples:
  void Process(int16_t *input, int16_t *output, size_t size) {
    while(size--) {
      // N integrators
      for (int i=0; i<R; i++) {
        hi[0] += *input++;
        for(int n=1; n<N; n++) {
          hi[n] += hi[n-1];
        }
      }
      // N combs
      int32_t v = hi[N-1];
      for (int n=0; n<N; n++) {
        int32_t in = v;
        v -= hc[n];
        hc[n] = in;
      }
      *output++ = static_cast<int16_t>(v / gain);
    }
  }
};

// N number of stages, R interpolation rate
// careful: gain must be <= 2^16
template<int N, int R>
class CicInterpolator {
  int32_t hi[N] = {0};
  int32_t hc[N] = {0};
  static constexpr int gain = ipow(R, N-1);
public:
  // reads [size] input samples, writes [R*size] output samples:
  void Process(int16_t *input, int16_t *output, size_t size) {
    while(size--) {
      // N combs
      int32_t v = *input++;
      for (int n=0; n<N; n++) {
        int32_t in = v;
        v -= hc[n];
        hc[n] = in;
      }
      // N integrators
      for (int i=0; i<R; i++) {
        hi[0] += i==0 ? v : 0;
        for(int n=1; n<N; n++) {
          hi[n] += hi[n-1];
        }
        *output++ = static_cast<int16_t>(hi[N-1] / gain);
      }
    }
  }
};

template<int N, int R>
class PdmFilter : CicDecimator<N, R> {

  const int16_t setbits[256] = {
#   define S(n) ((2*(n)-8) << 11)
#   define B2(n) S(n),  S(n+1),  S(n+1),  S(n+2)
#   define B4(n) B2(n), B2(n+1), B2(n+1), B2(n+2)
#   define B6(n) B4(n), B4(n+1), B4(n+1), B4(n+2)
    B6(0), B6(1), B6(1), B6(2)
  };

public:
  // reads [8*R*size] binary input sample, outputs [size] samples
  void Process(uint8_t *input, int16_t *output, size_t size) {
    int16_t temp[R * size];

    for (int i=0; i<size * R; i++) {
      temp[i] = setbits[*input];
      input++;
    }

    CicDecimator<N, R>::Process(temp, output, size);
  }
};

template <typename T>
class SlewLimiter {
  T state_ = {0};
  T slew_up_, slew_down_;
public:
  SlewLimiter(T slew_up, T slew_down) :
    slew_up_(slew_up), slew_down_(-slew_down) {}
  void Process(T input, T *output) {
    T error = input - state_;
    if (error > slew_up_) error = slew_up_;
    if (error < slew_down_) error = slew_down_;
    state_ += error;
    *output = state_;
  }

  void Process(T *input, T *output, size_t size) {
    while(size--) Process(*input++, output++);
  }
};
