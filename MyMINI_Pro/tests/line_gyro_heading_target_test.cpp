#include <cassert>
#include <cmath>
#include <limits>

#include "../src/LineGyroHeadingTarget.h"

int main() {
  LineGyroHeadingTarget outbound;
  assert(outbound.begin(0.0f, -135.0f));
  assert(!outbound.sample(-70.0f));
  assert(!outbound.sample(-134.0f));
  assert(outbound.sample(-136.0f));
  assert(std::fabs(outbound.stopRawYaw() + 135.0f) < 0.01f);

  LineGyroHeadingTarget returnRight;
  assert(returnRight.begin(-315.0f, 180.0f));
  assert(!returnRight.sample(-250.0f));
  assert(!returnRight.sample(-182.0f));
  assert(returnRight.sample(-178.0f));
  assert(std::fabs(returnRight.stopRawYaw() + 180.0f) < 0.01f);

  // A start at -300 used to choose -135 (the wrong direction for this leg).
  LineGyroHeadingTarget secondRoundLeft;
  assert(secondRoundLeft.begin(-300.0f, -135.0f));
  assert(!secondRoundLeft.sample(-350.0f));
  assert(!secondRoundLeft.sample(-400.0f));
  assert(!secondRoundLeft.sample(-493.0f));
  assert(secondRoundLeft.sample(-497.0f));
  assert(std::fabs(secondRoundLeft.stopRawYaw() + 495.0f) < 0.01f);

  LineGyroHeadingTarget startAtTarget;
  assert(startAtTarget.begin(-180.0f, 180.0f));
  assert(!startAtTarget.sample(-180.0f));
  assert(!startAtTarget.sample(-183.0f));
  assert(!startAtTarget.sample(-186.0f));  // Armed after leaving by 5 degrees.
  assert(!startAtTarget.sample(-189.0f));
  assert(!startAtTarget.sample(-182.0f));
  assert(startAtTarget.sample(-179.0f));

  LineGyroHeadingTarget acrossLeft;
  assert(acrossLeft.begin(-100.0f, 180.0f));
  assert(!acrossLeft.sample(-179.0f));
  assert(acrossLeft.sample(-181.0f));
  LineGyroHeadingTarget acrossRight;
  assert(acrossRight.begin(100.0f, -180.0f));
  assert(!acrossRight.sample(179.0f));
  assert(acrossRight.sample(181.0f));

  LineGyroHeadingTarget noisyNear;
  assert(noisyNear.begin(0.0f, 90.0f));
  assert(!noisyNear.sample(88.5f));
  assert(!noisyNear.sample(85.0f));  // One near frame must not latch.
  assert(!noisyNear.sample(89.0f));
  assert(noisyNear.sample(91.0f));
  LineGyroHeadingTarget invalid;
  assert(!invalid.begin(0.0f, std::numeric_limits<float>::quiet_NaN()));
}