#include <cassert>
#include <limits>

#include "../src/TurnGyroTarget.h"

int main() {
  using R = TurnGyroResult;
  TurnGyroTarget right;
  assert(right.begin(0.0f, 90.0f, 1) == R::Running);
  assert(right.sample(45.0f) == R::Running);
  assert(right.remainingDeg() == 45.0f);
  assert(right.lastStepDeg() == 45.0f);
  assert(right.sample(88.0f) == R::Running);
  assert(right.sample(91.0f) == R::Reached);  // Crossed between samples.

  TurnGyroTarget earlyRight;
  assert(earlyRight.beginWithBrakeLead(0.0f, 90.0f, 1, 20.0f) == R::Running);
  assert(earlyRight.sample(69.0f, 0.05f) == R::Running);
  assert(earlyRight.sample(70.0f, 0.05f) == R::Reached);

  TurnGyroTarget earlyLeft;
  assert(earlyLeft.beginWithBrakeLead(0.0f, -90.0f, -1, 20.0f) == R::Running);
  assert(earlyLeft.sample(-69.0f, 0.05f) == R::Running);
  assert(earlyLeft.sample(-71.0f, 0.05f) == R::Reached);

  TurnGyroTarget earlyHome;
  assert(earlyHome.beginWithBrakeLead(90.0f, 0.0f, -1, 20.0f) == R::Running);
  assert(earlyHome.sample(21.0f, 0.05f) == R::Running);
  assert(earlyHome.sample(20.0f, 0.05f) == R::Reached);

  TurnGyroTarget shortTurn;
  assert(shortTurn.beginWithBrakeLead(0.0f, 10.0f, 1, 20.0f) == R::Running);
  assert(shortTurn.sample(4.0f, 0.05f) == R::Running);
  assert(shortTurn.sample(5.0f, 0.05f) == R::Reached);

  TurnGyroTarget earlyWrap;
  assert(earlyWrap.beginWithBrakeLead(179.0f, -179.0f, 1, 20.0f) == R::Running);
  assert(earlyWrap.sample(179.5f, 0.05f) == R::Running);
  assert(earlyWrap.sample(-180.0f, 0.05f) == R::Reached);

  TurnGyroTarget earlyWrongMode;
  assert(earlyWrongMode.beginWithBrakeLead(0.0f, -90.0f, 1, 20.0f) ==
         R::WrongDirection);
  TurnGyroTarget invalidLead;
  assert(invalidLead.beginWithBrakeLead(0.0f, 90.0f, 1, -1.0f) ==
         R::InvalidTarget);

  TurnGyroTarget left;
  assert(left.begin(0.0f, -90.0f, -1) == R::Running);
  assert(left.sample(-80.0f) == R::Running);
  assert(left.sample(-92.0f) == R::Reached);

  TurnGyroTarget home;
  assert(home.begin(90.0f, 0.0f, -1) == R::Running);
  assert(home.sample(30.0f) == R::Running);
  assert(home.sample(-2.0f) == R::Reached);

  TurnGyroTarget alreadyHome;
  assert(alreadyHome.begin(0.0f, 0.0f, 1) == R::Reached);

  TurnGyroTarget wrapRight;
  assert(wrapRight.begin(179.0f, -179.0f, 1) == R::Running);
  assert(wrapRight.sample(-178.0f) == R::Reached);
  TurnGyroTarget wrapLeft;
  assert(wrapLeft.begin(-179.0f, 179.0f, -1) == R::Running);
  assert(wrapLeft.sample(178.0f) == R::Reached);
  TurnGyroTarget multipleTurns;
  assert(multipleTurns.begin(1080.0f, 90.0f, 1) == R::Running);
  assert(multipleTurns.sample(1171.0f) == R::Reached);

  TurnGyroTarget right180;
  assert(right180.begin(0.0f, 180.0f, 1) == R::Running);
  assert(right180.sample(179.0f) == R::Reached);
  TurnGyroTarget left180;
  assert(left180.begin(0.0f, -180.0f, -1) == R::Running);
  assert(left180.sample(-179.0f) == R::Reached);

  TurnGyroTarget wrongMode;
  assert(wrongMode.begin(0.0f, -90.0f, 1) == R::WrongDirection);
  TurnGyroTarget wrongWiring;
  assert(wrongWiring.begin(0.0f, 90.0f, 1) == R::Running);
  assert(wrongWiring.sample(-6.0f) == R::WrongDirection);

  TurnGyroTarget stalled;
  assert(stalled.begin(0.0f, 90.0f, 1) == R::Running);
  for (int i = 0; i < 20; ++i) {
    assert(stalled.sample(0.0f) == R::Running);
  }
  assert(!TurnGyroTarget::timedOut(2999u, 0u, 3000u));
  assert(TurnGyroTarget::timedOut(3000u, 0u, 3000u));
  assert(TurnGyroTarget::timedOut(4u, 0xFFFFFFF0u, 20u));

  TurnGyroTarget invalid;
  const float nan = std::numeric_limits<float>::quiet_NaN();
  assert(invalid.begin(0.0f, nan, 1) == R::InvalidTarget);
  assert(invalid.begin(0.0f, 181.0f, 1) == R::InvalidTarget);
  assert(invalid.begin(nan, 90.0f, 1) == R::InvalidSample);
  assert(invalid.begin(0.0f, 90.0f, 1) == R::Running);
  assert(invalid.sample(nan) == R::InvalidSample);
}
