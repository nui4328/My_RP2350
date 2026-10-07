# Line sensor check on the robot

The default MUX scan is CH0..CH15 on both rows. S0 is the low address bit.
Front uses GP17/16/15/14 and GP27; rear uses GP10/11/12/13 and GP26.
After selecting a channel, the scanner waits 4 us, discards the first ADC
conversion after switching ADC input, then averages two conversions. This
procedure has **not** yet been validated against raw readings from this board.

## Capture raw evidence

Use 115200 baud while `wait_button()` is active, with motors stopped. For each
surface, keep the robot still and issue `sensor_diag`. It reports each channel's
last raw reading, raw range across 32 complete frames, EEPROM calibration
minimum and maximum, normalized 0..1000 reading, and the production midpoint
BLACK/WHITE predicate. Capture the output with the whole row on uniform white,
then on uniform black. A narrow calibration span can magnify small raw jitter.
The saved calibration holds extrema only; it does not record which surface was
white or black. Physical polarity is confirmed only when the two captures show
their ordering.

Repeat the still-white capture with `mux_settle 20` and `mux_settle 50`, then
restore `mux_settle 4`. Compare each channel's raw range. A marked improvement
with settling time implicates MUX/ADC acquisition; unchanged raw jitter needs
electrical or sensor investigation. This command changes only the running
scanner's settle time and does not save EEPROM data or add a filter.

## Exit mapping

The selected channel OR its neighbour toward the row centre is tested. The
same mapping applies to both `f` and `b` rows.

| Selected | Pair | Selected | Pair |
| --- | --- | --- | --- |
| 0 | 0,1 | 8 | 8,7 |
| 1 | 1,2 | 9 | 9,8 |
| 2 | 2,3 | 10 | 10,9 |
| 3 | 3,4 | 11 | 11,10 |
| 4 | 4,5 | 12 | 12,11 |
| 5 | 5,6 | 13 | 13,12 |
| 6 | 6,7 | 14 | 14,13 |
| 7 | 7,8 | 15 | 15,14 |

After a 100 ms arm delay and two white pair samples, two of three black pair
samples end a sensor command. All-white or black-to-white alone does not end
it. `offset=0` keeps the no-brake handoff; `offset>0` applies the existing
35 ms reverse pull after black detection and stops. Distance overloads end
on their estimated distance. A 15 s timeout stops both motors.

## Supervised line check

`examples/05_LineSensorCheck` contains the exact call
`f_line(30, 30, 2.5f, f0, 5)` behind `RUN_LINE_BENCH=false`. Once white and
black captures establish polarity, set it true and run white -> black -> white
on the robot. The opt-in serial trace reports pair raw/normalized samples and
`SENSOR_BLACK`, `DISTANCE`, `TIMEOUT`, `INVALID_SENSOR`, or
`INVALID_ARGUMENT`. For this call,
`SENSOR_BLACK` must appear when the pair first qualifies as black, before the
robot reaches white again. The `offset=5` reverse pull occurs before the
reason line and sketch return. No board was flashed for this check.
