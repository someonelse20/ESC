# ESC

![alt text](/assets/3d.png)

A custom designed electronic speed controller that supports up to 7 cell LiPo batteries with 30 amps (untested).
I made this as part of my overall project of making a drone from scratch.

## Pinout

| Pin # | Function  |
| ----- | --------- |
| 1     | PWM IN    |
| 2     | Direction |
| 3     | GND       |

Pull the Direction pin low (default) for clockwise rotation or high (3.3v) for counter clockwise rotation.
Alternatively, leave the solder bridge on the bottom of the board open for clockwise and closed for counter clockwise.

## PCB Layout

![alt text](/assets/board.png)

## Schematic

![alt text](/assets/schematic.png)

## BOM

| Name       | Purpose             | Quantity | Total Cost (USD) | Link                              | Distributor |
| ---------- | ------------------- | -------- | ---------------- | --------------------------------- | ----------- |
| Tachometer | Test motor speed    | 1        | 21.99            | [Amazon](https://a.co/d/0gPeHyXq) | Amazon      |
| PCB        | The assembled board | 2        | 140.29           | [JLCPCB](https://jlcpcb.com/)     | JLCPCB      |
