# Hardware

Each telephone needs one board. The board holds an original ESP32 DevKit
(ESP-WROOM-32), a Silvertel AG1170 subscriber-line module, a PCM1808
development board, and the level-shift transistors between the ESP32 and
the AG1170.

The KiCad project is `hardware/esp32-phone-exchange/`. Open
`esp32-phone-exchange.kicad_pro` in KiCad 10. The schematic is the source
of the connections below. The symbol drawn for the AG1170 numbers its
pins in schematic order; those numbers are not the Silvertel module pin
numbers. Signal names (`F/R`, `RM`, `SHK`, `PD`, `VIN`, `VOUT`) are the
ones to follow. On the Silvertel module, `PD` is pin 4.

## What to build

| Part | Notes |
| --- | --- |
| ESP32 DevKit, original ESP32 | 30-pin DevKit with a DAC on GPIO25. Not C3, S2, or S3. The KiCad footprint was started from a DevKit outline; the pin names on the board are the classic ESP32 names. |
| Silvertel AG1170 | 5 V subscriber-line module. It supplies loop current and ring voltage for one phone. |
| PCM1808 module | I2S analog-to-digital converter for the phone's microphone path. |
| Three small-signal NPN transistors | E-B-C pinout, such as 2N2222. Q3, Q4, and Q5 on the board. |
| One rotary phone | Loop-disconnect dial, about 10 pulses per second. |

Power the DevKit from its USB connector. The DevKit 5 V pin feeds the
AG1170, the PCM1808 5 V pin, and the logic pull-ups. Do not connect a
second 5 V supply to that rail at the same time.

The RJ11 jack (J1) and the two-pin screw terminal (J2) are wired in
parallel on tip and ring. Use either one for a single phone. Do not
connect this jack to the public telephone network. Ring voltage from the
AG1170 is high enough to hurt, including while the phone is on the hook
and a call is coming in.

## ESP32 pins

| GPIO | Role | Connection |
| --- | --- | --- |
| 25 | DAC1, audio out | AG1170 `VIN` through C6, 100 nF |
| 34 | SHK input | AG1170 `SHK` through R5, 10 kΩ. R6, 100 kΩ, goes from this node to ground. |
| 18 | F/R | Base of Q3 through R2, 1 kΩ |
| 19 | RM | Base of Q4 through R3, 1 kΩ |
| 4 | PD | Base of Q5 through R8, 1 kΩ |
| 1 | I2S MCLK | PCM1808 `SCK`. Also the DevKit serial TX pin. |
| 26 | I2S BCLK | PCM1808 `BCK` |
| 27 | I2S LRCK | PCM1808 `LRC` |
| 35 | I2S data in | PCM1808 `DOUT` |

GPIO1 can be the only master-clock pin available on this DevKit. The
original ESP32 can drive I2S MCLK from GPIO0, GPIO1, or GPIO3, and GPIO0
is not broken out. Once the codec starts, GPIO1 is a 2.048 MHz clock and
the serial monitor stops.

## F/R and RM

`F/R` chooses the ringing polarity and `RM` turns ringing on. Both are
5 V inputs. Each has an NPN level shifter, and the transistor inverts
the GPIO:

```text
  +5 V
    |
   5 kΩ          R1 for F/R, R4 for RM
    |
    +---- AG1170 input
    |
    C
  GPIO -- 1 kΩ -- B   Q3 for F/R (GPIO18), Q4 for RM (GPIO19)
    E
    |
   GND
```

GPIO low turns the transistor off, and the 5 kΩ resistor pulls the
AG1170 input high. GPIO high turns the transistor on and pulls the
AG1170 input low. The firmware already accounts for that inversion.

## Power-down

`PD` is a different circuit. It must be driven open-collector, with no
pull-up.

```text
  GPIO4 -- 1 kΩ -- B   Q5
                 C ---- AG1170 PD (module pin 4)
                 E ---- GND
```

GPIO low leaves `PD` open and the AG1170 DC/DC converter runs. GPIO high
pulls `PD` to ground and the converter shuts down. A high logic level on
`PD`, or a resistor from `PD` up to 5 V, damages the converter. Keep the
wire to pin 4 short.

The firmware leaves the converter off while the phone is on-hook and
idle, and wakes it about once a second to read the hook switch. Outputs,
including `SHK`, are not valid until the converter has been up for 50 ms.

## Hook switch

`SHK` goes to GPIO34 through the divider on the schematic: R5 is 10 kΩ
from `SHK` to the GPIO, and R6 is 100 kΩ from the GPIO to ground. Those
two values were measured on a built board. GPIO34 is high when the
handset is off the hook. The firmware ignores edges shorter than the
off-hook debounce (250 ms), which is how a rotary dial can pulse the
same wire without hanging the phone up.

`SHK` on the 5 V AG1170 is not a stiff 5 V logic output. Silvertel
specifies it as a 62 kΩ pull-up inside the module, and the high level
is only guaranteed (4.4 V minimum) while the pin sources 10 µA or less.
The resistors outside the module sit in series with that pull-up, so
the voltage at GPIO34 is

```text
Vgpio = 5 V × R6 / (62 kΩ + R5 + R6)
```

With R5 = 10 kΩ and R6 = 100 kΩ that is about 2.9 V. That is above the
ESP32 input-high threshold (0.75 × 3.3 V, about 2.5 V) and below the
3.6 V pin limit.

A 10 kΩ / 20 kΩ divider is the pair that would make 3.3 V if `SHK` were
a hard 5 V source. On this pin the same pair is about 1.1 V, because the
30 kΩ external load is small compared with the 62 kΩ pull-up and most of
the 5 V is dropped inside the module. 1.1 V is below the ESP32 input-high
threshold, so the handset looks on-hook. The 100 kΩ bottom resistor is
what keeps the off-hook level up near 2.9 V.

## Audio

Receive audio is the ESP32 DAC. C6 (100 nF) couples GPIO25 to the AG1170
audio input, `VIN`.

Transmit audio is the PCM1808. C5 (100 nF) couples the AG1170 audio
output, `VOUT`, to the PCM1808 left input, `LIN`. The ESP32 is the I2S
master. On the PCM1808 module, `FMY`, `MD0`, and `MD1` are tied to
ground, which selects slave mode and 24-bit Philips I2S. The module's
3.3 V pin comes from the DevKit 3V3 pin. Its 5 V pin comes from the
board 5 V rail. Grounds are common.

The DAC path only has 8 bits of useful resolution. A full-scale 16-bit
sample is what the handset can play loudly. The softphone's transmit
gain exists for that reason.

## Power filter

The DevKit 5 V rail is `VPWR` on the AG1170 (the module's supply pin).
C3 (220 µF, 10 V) and C4 (100 nF) sit across that rail.

The AG1170 analog supply, `+VA`, is fed from 5 V through R7 (10 Ω), with
C1 (100 nF) and C2 (100 µF, 6.3 V) to ground. That is the Silvertel
supply arrangement for the audio section. All grounds on the ESP32, the
AG1170, the PCM1808, and the transistors are the same net.

## Bill of materials

Values are the ones in the schematic.

| Ref | Value | Function |
| --- | --- | --- |
| U1 | ESP32 DevKit | Controller, DAC, Wi-Fi |
| U2 | AG1170 | Subscriber line interface |
| U3 | PCM1808 module | Phone-to-network audio |
| Q3, Q4, Q5 | NPN, EBC | F/R, RM, and PD |
| R1, R4 | 5 kΩ | Pull-ups to 5 V on F/R and RM |
| R2, R3, R8 | 1 kΩ | Transistor base resistors |
| R5 | 10 kΩ | SHK series resistor |
| R6 | 100 kΩ | SHK divider to ground |
| R7 | 10 Ω | Feed resistor for +VA |
| C1, C4, C5, C6 | 100 nF | +VA, 5 V, and the two audio couplings |
| C2 | 100 µF, 6.3 V | +VA reservoir |
| C3 | 220 µF, 10 V | 5 V reservoir |
| J1 | RJ11 | Phone, in parallel with J2 |
| J2 | 2-pin screw terminal | Phone, in parallel with J1 |

## Enclosure

`hardware/esp32-phone-exchange/esp32-phone-exchange-enclosure.stl` is a
printable case for this board. `esp32-phone-exchange.stl` in the same
folder is the 3D model of the PCB itself, exported from KiCad.

## Ordering the PCB

Gerbers exported from this project are in
`hardware/esp32-phone-exchange/gerbers/`. The board is two layers,
1.6 mm, about 103 mm by 70 mm. Zip this folder when a board house asks
for an upload. The export recorded in the job file is from KiCad 10.0.7.
After any schematic or layout change, plot a new set from Pcbnew rather
than reusing these files.
