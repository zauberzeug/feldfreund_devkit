# ODrive Calibration

## Flashing ODrive

To flash an ODrive it has to be put into DFU Mode.
For this there is a switch on the black ODrive board.
This switch has to be put into the DFU setting.
Following this, the whole board needs to be power cycled (Including RdyP and the battery connection).
Then the ODrive needs to be connected to the Robot Brain using a micro-USB cable.

Now, from the odrive directory, the following command can be run.
This will flash the STM chip with the newest firmware.

```bash
./flash_odrive.sh fw-v5.6-feldfreund/ODriveFirmware.bin
```

## Calibrating ODrive

To calibrate the ODrives, they have to be in the run mode(switch with DFU and run).
The normal Feldfreund Lizard script will interfere with the calibration script.
In order to prevent this the `calibration.liz` needs to be configured.
This can be done in the `<path_to_fodler>/lizard` folder with the command:

```bash
./configure.py <path_to_file>/calibration.liz <ESP_port>
```

If this is failing, make sure there are no other serial connections running to the ESP.

Before starting the calibration make sure, that the tracks do not touch the ground and the ODrive is connected to the Robot Brain using the micro-USB cable.
Then for the right track run:

```bash
python3 calibrate_two_motors_r.py
```

and for the left track:

```bash
python3 calibrate_two_motors_l.py
```

If the calibration does not work calling it with `sudo` can help.
These scripts will set the motor parameters and start the calibration of the hall sensors.
When you are done, don't forget to reconfigure the ESP with your Feldfreund code.

## Calibrating a delta arm (motors that cannot spin freely)

`calibrate_two_motors_{l,r}.py` run a `FULL_CALIBRATION_SEQUENCE` and carry the *track* parameters.
Do not use them on a delta arm: the arm cannot rotate freely, and the scripts would overwrite the arm
values (`current_lim` 35 instead of 10, `torque_constant` 1.45 instead of 1.0, node IDs 0x200/0x300).

After a firmware flash the config restore brings every calibration *result* back — phase resistance
and inductance, the hall `phase_offset`, `direction`, `hall_polarity`. Only
`encoder.config.pre_calibrated` is rejected by the firmware while the encoder is not ready, so the
board boots with `UNKNOWN_PHASE_ESTIMATE` and the axis never arms. One successful encoder offset scan
makes the flag settable again; nothing else has to be re-measured.

`calibrate_delta_arm.py` re-runs exactly that one step:

```bash
python3 calibrate_delta_arm.py --axis both --window=-10,+10 --calibration-current 8 --commit
```

The scan is open loop and ignores the endstops, so two guards bound it. `--window` limits the travel
at the output and drops the axis to IDLE when it is exceeded; `--calibration-current` lowers the
torque for the scan only and is restored afterwards. Without `--commit` nothing is persisted, and
`--dry-run` reads only.

How far the arm actually moves (measured on ts2, G350 with 8 pole pairs, 48 cpr, gear ratio 12.5):

| `--scan-distance` | peak excursion at the output | net |
| --- | --- | --- |
| 6.283 (2*pi, the minimum) | 3.6 deg | ~0 |
| 12.566 (4*pi, default) | 7.2 deg | ~0 |
| 50.265 (firmware default) | 28.8 deg | ~0 |

The scan sweeps out and back, so the arm returns to where it started, but it needs the full excursion
free in one direction — the sign is not guaranteed, so park the arm mid-range. Below one electrical
revolution there is no complete cycle to average, which is the floor on `--scan-distance`.

Two things that look like failures and are not. The encoder count is rebased onto `count_in_cpr` when
the scan starts, which is a jump by a multiple of `cpr`; the script detects it by rate and does not
count it as travel. And `phase_offset` is only meaningful **modulo one electrical revolution**
(`cpr / pole_pairs` = 6 counts here), so successive runs legitimately report 48, 42 or 6 for the same
physical calibration.

`--commit` writes the flag and saves, which reboots the board. With `startup_closed_loop_control`
the axes then arm by themselves and the arm holds its position — be at the e-stop.

## Further debugging

Further debugging can be done using the `odrivetool`.
While the ODrive is connected via USB the tool can be started from the command line.
For further documentation reference the [ODrive documentation](https://docs.odriverobotics.com/v/0.5.6/getting-started.html).
