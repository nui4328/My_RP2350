#include "TurnFrontEdgeGuard.h"

#include <cstdio>
#include <cstdlib>

static unsigned checks = 0u;
#define CHECK(value) do { ++checks; if (!(value)) { \
  std::fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #value); \
  std::exit(1); } } while (0)

int main() {
  // White edges before reaching the selected stop sensor cannot end a turn.
  TurnFrontEdgeGuard beforeTarget(600u);
  CHECK(!beforeTarget.update(false, 1000u, 1000u, 1000u, 1000u));
  CHECK(!beforeTarget.targetReached());
  CHECK(!beforeTarget.update(false, 0u, 0u, 0u, 0u));

  // A clean target frame returns immediately, without another confirmation wait.
  TurnFrontEdgeGuard clean(600u);
  CHECK(clean.update(true, 600u, 600u, 600u, 600u));
  CHECK(clean.targetReached());

  // Left edge black blocks completion. The selected stop sensor can then leave
  // black; its first confirmation stays latched while the edge clears.
  TurnFrontEdgeGuard left(600u);
  CHECK(!left.update(true, 100u, 900u, 100u, 900u));
  CHECK(left.targetReached());
  CHECK(!left.update(false, 599u, 900u, 599u, 900u));
  CHECK(left.update(false, 600u, 900u, 600u, 900u));

  TurnFrontEdgeGuard right(600u);
  CHECK(!right.update(true, 900u, 100u, 900u, 100u));
  CHECK(!right.update(false, 900u, 599u, 900u, 599u));
  CHECK(right.update(false, 900u, 600u, 900u, 600u));

  // Both edges must be white in the same frame, not in separate frames.
  TurnFrontEdgeGuard alternating(600u);
  CHECK(!alternating.update(true, 100u, 900u, 100u, 900u));
  CHECK(!alternating.update(false, 900u, 100u, 900u, 100u));
  CHECK(!alternating.update(false, 100u, 900u, 100u, 900u));
  CHECK(alternating.update(false, 900u, 900u, 900u, 900u));

  // Verify every combination of raw/filtered black on either edge. A single
  // non-white reading must block completion, including residual filtered black.
  for (unsigned nonWhiteMask = 0u; nonWhiteMask < 16u; ++nonWhiteMask) {
    TurnFrontEdgeGuard guard(600u);
    const uint16_t leftRaw = nonWhiteMask & 1u ? 599u : 600u;
    const uint16_t rightRaw = nonWhiteMask & 2u ? 599u : 600u;
    const uint16_t leftFiltered = nonWhiteMask & 4u ? 599u : 600u;
    const uint16_t rightFiltered = nonWhiteMask & 8u ? 599u : 600u;
    CHECK(guard.update(true, leftRaw, rightRaw, leftFiltered, rightFiltered) ==
          (nonWhiteMask == 0u));
  }

  // Each turn starts with its own unconfirmed target.
  TurnFrontEdgeGuard nextTurn(600u);
  CHECK(!nextTurn.targetReached());
  CHECK(!nextTurn.update(false, 900u, 900u, 900u, 900u));
  std::printf("PASS: %u checks (turn target latch, both front edges, raw and filtered white)\n", checks);
}
