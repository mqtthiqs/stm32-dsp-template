#pragma once
#include <cstdio>

#include "numtypes.h"

namespace grm {

class PpmWriter {
  FILE* fp;

 public:
  struct Pixel {
    u0_8 r;
    u0_8 g;
    u0_8 b;
  };

  PpmWriter(const char* filename, int width, int height) {
    fp = fopen(filename, "wb");
    if (fp == NULL) {
      std::cerr << "output file \"" << filename << "\" cannot be created" << std::endl;
      exit(1);
      ;
    }
    fprintf(fp, "P6\n");
    fprintf(fp, "%d %d\n", width, height);
    fprintf(fp, "255\n");
  }

  ~PpmWriter() { fclose(fp); }

  void Write(Pixel p) {
    fputc(p.r.repr(), fp);
    fputc(p.g.repr(), fp);
    fputc(p.b.repr(), fp);
  }
};
}  // namespace grm
