// Frame capture, PNG output, and the harness entry point.

#include "StepScreenShot.h"

#include "SimInternal.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include <algorithm>
#include <string>
#include <vector>

namespace {

StepScreenShot::Frame g_last;
std::string g_outDir = ".";
int g_scale = 4;
uint8_t g_on[3] = {0xFF, 0xFF, 0xFF};
uint8_t g_off[3] = {0x00, 0x00, 0x00};
int g_saved = 0;

uint32_t crc32(const uint8_t *data, size_t len, uint32_t crc = 0) {
  static uint32_t table[256];
  static bool ready = false;
  if (!ready) {
    for (uint32_t n = 0; n < 256; n++) {
      uint32_t c = n;
      for (int k = 0; k < 8; k++) {
        c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
      }
      table[n] = c;
    }
    ready = true;
  }
  crc = ~crc;
  for (size_t i = 0; i < len; i++) {
    crc = table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
  }
  return ~crc;
}

void putU32(std::vector<uint8_t> &out, uint32_t v) {
  out.push_back((uint8_t)(v >> 24));
  out.push_back((uint8_t)(v >> 16));
  out.push_back((uint8_t)(v >> 8));
  out.push_back((uint8_t)v);
}

void putChunk(std::vector<uint8_t> &out, const char *type,
              const std::vector<uint8_t> &data) {
  putU32(out, (uint32_t)data.size());
  std::vector<uint8_t> body(type, type + 4);
  body.insert(body.end(), data.begin(), data.end());
  out.insert(out.end(), body.begin(), body.end());
  putU32(out, crc32(body.data(), body.size()));
}

// zlib stream with uncompressed (stored) deflate blocks: no zlib dependency,
// and 1-bit rows keep files small anyway.
std::vector<uint8_t> zlibStore(const std::vector<uint8_t> &raw) {
  std::vector<uint8_t> z = {0x78, 0x01};
  size_t pos = 0;
  do {
    const size_t n = std::min<size_t>(65535, raw.size() - pos);
    const bool final = pos + n == raw.size();
    z.push_back(final ? 1 : 0);
    z.push_back((uint8_t)n);
    z.push_back((uint8_t)(n >> 8));
    z.push_back((uint8_t)~n);
    z.push_back((uint8_t)(~n >> 8));
    z.insert(z.end(), raw.begin() + pos, raw.begin() + pos + n);
    pos += n;
  } while (pos < raw.size());

  uint32_t a = 1, b = 0;
  for (uint8_t v : raw) {
    a = (a + v) % 65521;
    b = (b + a) % 65521;
  }
  putU32(z, (b << 16) | a);
  return z;
}

bool writePng(const std::string &path, const StepScreenShot::Frame &f) {
  const int w = StepScreenShot::Frame::W * g_scale;
  const int h = StepScreenShot::Frame::H * g_scale;
  const int rowBytes = (w + 7) / 8;

  std::vector<uint8_t> raw;
  raw.reserve((size_t)(rowBytes + 1) * h);
  for (int y = 0; y < h; y++) {
    raw.push_back(0); // filter: none
    for (int bx = 0; bx < rowBytes; bx++) {
      uint8_t byte = 0;
      for (int bit = 0; bit < 8; bit++) {
        const int x = bx * 8 + bit;
        if (x < w && f.pixel(x / g_scale, y / g_scale)) {
          byte |= (uint8_t)(0x80 >> bit);
        }
      }
      raw.push_back(byte);
    }
  }

  std::vector<uint8_t> png = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1A, '\n'};
  std::vector<uint8_t> ihdr;
  putU32(ihdr, (uint32_t)w);
  putU32(ihdr, (uint32_t)h);
  ihdr.push_back(1); // bit depth
  ihdr.push_back(3); // indexed color
  ihdr.push_back(0);
  ihdr.push_back(0);
  ihdr.push_back(0);
  putChunk(png, "IHDR", ihdr);
  putChunk(png, "PLTE",
           {g_off[0], g_off[1], g_off[2], g_on[0], g_on[1], g_on[2]});
  putChunk(png, "IDAT", zlibStore(raw));
  putChunk(png, "IEND", {});

  FILE *fp = fopen(path.c_str(), "wb");
  if (fp == nullptr) {
    return false;
  }
  const bool ok = fwrite(png.data(), 1, png.size(), fp) == png.size();
  fclose(fp);
  return ok;
}

bool parseHexColor(const char *s, uint8_t rgb[3]) {
  if (s[0] == '#') {
    s++;
  }
  if (strlen(s) != 6) {
    return false;
  }
  char *end = nullptr;
  const unsigned long v = strtoul(s, &end, 16);
  if (*end != '\0') {
    return false;
  }
  rgb[0] = (uint8_t)(v >> 16);
  rgb[1] = (uint8_t)(v >> 8);
  rgb[2] = (uint8_t)v;
  return true;
}

void usage(const char *argv0) {
  fprintf(stderr,
          "usage: %s --out DIR [--scale N] [--on RRGGBB] [--off RRGGBB] "
          "[--serial]\n",
          argv0);
  exit(2);
}

} // namespace

namespace StepScreenSimInternal {

void publishFrame(const uint8_t *pages, bool inverted) {
  memcpy(g_last.pixels, pages, sizeof(g_last.pixels));
  g_last.inverted = inverted;
  g_last.ms = (uint32_t)(nowUs() / 1000ULL);
}

} // namespace StepScreenSimInternal

namespace StepScreenShot {

bool Frame::blank() const {
  for (uint8_t b : pixels) {
    if (b != 0) {
      return false;
    }
  }
  return true;
}

const Frame &lastFrame() { return g_last; }

void save(const char *name) { save(name, g_last); }

void save(const char *name, const Frame &frame) {
  const std::string path = g_outDir + "/" + name + ".png";
  if (!writePng(path, frame)) {
    fprintf(stderr, "screenshot: failed to write %s\n", path.c_str());
    exit(1);
  }
  printf("  %s.png  (t=%.1fs)\n", name, frame.ms / 1000.0);
  g_saved++;
}

void expect(bool condition, const char *what) {
  if (!condition) {
    fprintf(stderr, "screenshot: scene check failed: %s\n", what);
    exit(1);
  }
}

void setOutputDir(const std::string &dir) { g_outDir = dir; }

void setScale(int scale) { g_scale = scale < 1 ? 1 : scale; }

} // namespace StepScreenShot

int main(int argc, char **argv) {
  std::string outDir;
  bool echoSerial = false;
  for (int i = 1; i < argc; i++) {
    const std::string arg = argv[i];
    if (arg == "--out" && i + 1 < argc) {
      outDir = argv[++i];
    } else if (arg == "--scale" && i + 1 < argc) {
      StepScreenShot::setScale(atoi(argv[++i]));
    } else if (arg == "--on" && i + 1 < argc) {
      if (!parseHexColor(argv[++i], g_on)) {
        usage(argv[0]);
      }
    } else if (arg == "--off" && i + 1 < argc) {
      if (!parseHexColor(argv[++i], g_off)) {
        usage(argv[0]);
      }
    } else if (arg == "--serial") {
      echoSerial = true;
    } else {
      usage(argv[0]);
    }
  }
  if (outDir.empty()) {
    usage(argv[0]);
  }
  mkdir(outDir.c_str(), 0755);
  StepScreenShot::setOutputDir(outDir);

  const std::string logPath = outDir + "/serial.log";
  FILE *serialLog = fopen(logPath.c_str(), "w");
  StepScreenSimInternal::setSerialSink(serialLog, echoSerial);

  printf("Writing screenshots to %s\n", outDir.c_str());
  stepscreenScenes();
  printf("%d screenshot(s); sketch serial output in serial.log\n", g_saved);

  if (serialLog != nullptr) {
    fclose(serialLog);
  }
  return 0;
}
