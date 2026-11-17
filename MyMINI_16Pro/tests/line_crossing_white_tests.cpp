#include "LineCrossingWhiteFilter.h"

#include <cstdio>
#include <cstdlib>

static unsigned checks = 0u;
#define CHECK(value) do { ++checks; if (!(value)) { \
  std::fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #value); \
  std::exit(1); } } while (0)

int main() {
  LineCrossingWhiteFilter consecutive;
  CHECK(!consecutive.update(true));
  CHECK(!consecutive.update(true));
  CHECK(!consecutive.update(true));
  CHECK(consecutive.update(true));

  // Four white frames need not be consecutive, but must fit the six-frame window.
  LineCrossingWhiteFilter intermittent;
  CHECK(!intermittent.update(true));
  CHECK(!intermittent.update(false));
  CHECK(!intermittent.update(true));
  CHECK(!intermittent.update(false));
  CHECK(!intermittent.update(true));
  CHECK(intermittent.update(true));

  // On the next frame, the oldest white expires: only three remain.
  CHECK(!intermittent.update(false));
  CHECK(intermittent.update(true));
  CHECK(!intermittent.update(false));
  CHECK(!intermittent.update(false));
  CHECK(!intermittent.update(false));
  CHECK(!intermittent.update(false));
  CHECK(!intermittent.update(false));
  CHECK(!intermittent.update(false));

  // Every possible six-frame pattern must agree with the 4-of-6 specification.
  for (unsigned pattern = 0u; pattern < 64u; ++pattern) {
    LineCrossingWhiteFilter filter;
    unsigned white = 0u;
    bool released = false;
    for (unsigned frame = 0u; frame < 6u; ++frame) {
      const bool sample = (pattern & (1u << frame)) != 0u;
      if (sample) ++white;
      released = filter.update(sample);
    }
    CHECK(released == (white >= 4u));
  }

  // Alternating one-white/one-black sensor readings never make a both-white frame.
  LineCrossingWhiteFilter paired;
  for (unsigned frame = 0u; frame < 12u; ++frame) {
    const bool leftWhite = (frame % 2u) == 0u;
    const bool rightWhite = !leftWhite;
    CHECK(!paired.update(leftWhite && rightWhite));
  }
  // A new command has fresh history rather than inheriting the previous release.
  LineCrossingWhiteFilter nextCommand;
  CHECK(!nextCommand.update(true));
  std::printf("PASS: %u checks (4-of-6 white release, rolling window, paired sensors)\n", checks);
}
