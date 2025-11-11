# ODrive Calibration

## Flashing ODrive

To flash an ODrive it has to be put into DFU Mode. For this there is a switch on the black ODrive board. This switch has to be put into the DFU setting. Following this, the whole board needs to be power cycled (Including RdyP and the battery connection).
Then the ODrive needs to be connected to the Robot Brain using a micro-USB cable. Now the command

```bash
./flash_odrive.sh fw-v5.6-feldfreund/ODriveFirmware.bin
```

can be run. This will flash the STM chip with the newest firmware.

## Calibrating ODirve

To calibrate the ODrives, they have to be in the run mode(switch with DFU and run). The normal Feldfreund Lizard script will interfere with the calibration script. In order to prevent this the `calibration.liz` needs to be configured. This can be done in the `<path_to_fodler>/lizard` folder with the command:

```bash
./configure.py <path_to_file>/calibration.liz <ESP_port>
```

If this is failing, make sure there are no other serial connections running to the ESP.

Before starting the calibration make sure, that the tracks do not touch the ground and the ODrive is connected to the Robot Brain using the micro-USB cable. Then for the right track run:

```bash
python3 calibrate_two_motors_r.py
```

and for the left track:

```bash
python3 calibrate_two_motors_l.py
```

These scripts will set the motor parameters and start the calibration of the hall sensors.

When you are done, don't forget to reconfigure the ESP with your Feldfreund code.

## Further debugging

Further debugging can be done using the `odrivetool`. While the ODrive is connected via USB the tool can be started from the command line. For further documentation reference the [ODrive documentation](https://docs.odriverobotics.com/v/0.5.6/getting-started.html).
