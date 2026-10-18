# Supported sensors and protocols

Summarised from the Sangbay datasheets: S-M4T 485 V1.3.2 (2024-11-09), S-M4T, S-M4, K-5S,
GT-TS01A, SLTD-CH4-100, and the heads S4-H2S-100, MJC4-2.8J, N-CH4.

## Which `type`

| Product | What it is | `type` |
|---|---|---|
| S-M4T 485 | module, RS485 (yellow B, green A), 5 V | `module` |
| S-M4T | module, TTL UART (green TX, yellow RX), 5–24 V | `module` |
| S-M4 | module, TTL UART, 5 V | `module` |
| K-5S | module with temperature compensation, TTL UART, 5 V | `module` |
| S4-H2S-100 | electrochemical H2S head, 0–100 ppm, nA output | plugs into a module |
| MJC4-2.8J | catalytic combustible gas head, 0–100 %LEL, bridge output | plugs into a module |
| N-CH4 | infrared methane head, analog 0.4–2 V (its UART protocol is not documented) | plugs into a module |
| GT-TS01A | laser (TDLAS) methane sensor, 0–5 %VOL, TTL | `gt_ts01a` |
| SLTD-CH4-100 | laser methane remote sensor, 100 m, 0–100000 ppm·m, TTL | `sltd_ch4` |

TTL modules and the lasers need a USB-TTL (3.3 V) adapter, one per sensor. RS485 modules share
one USB-RS485 adapter, each with its own address.

## Modules: Modbus RTU

9600 8N1, CRC16 Modbus, function 0x03 read, 0x10 write. Factory address 0xFF.

| Register | | |
|---|---|---|
| `0x2011` | Modbus address 0..254 | read / write (unlock first) |
| `0x2027` | gas code (70 gases: 0x01 CO, 0x02 H2S, 0x03 EX, 0x04 O2, 0x06 CH4, ...) | read |
| `0x202B` | range | read |
| `0x2030` | unit code (1 %LEL, 2 ppm, 3 ppb, 4 %VOL, 6 mg/m3, ...) | read / write |
| `0x2031` | decimal places: value = raw / 10^decimals | read / write |
| `0x4FFF` | write `0x55AA` to remove write protection | write |
| `0x6000` | alarm status, below | read |
| `0x6001` | concentration, raw | read |
| `0x6002` | AD value | read |
| `0x6006` | write: calibration command; read: operating status | read / write |

`0x6000`: bit 15 = 0 while preheating, bits 0–14 = remaining time × 0.1 s. Bit 15 = 1:
bit 0 low alarm, 1 high alarm, 2 STEL, 3 TWA, 4 over range, 5 sensor lost or damaged,
6 high concentration protection. The driver reports bit 5 as `fault`, the others as `alarm`.

Setting the address: `FF 10 4F FF 00 01 02 55 AA FB D0` (unlock), then
`FF 10 20 11 00 01 02 00 01 0D 77` (address 1). `sangbay_tool set-address` does both.

Calibration (`0x6006`, commands 0x1000 zero … 0x1005, up to 6 points) is not exposed by the
driver: the modules are calibrated in the factory.

## GT-TS01A

115200 8N1, sends by itself every 2 s, 29 ASCII bytes:

```
+000.00 +22.3 +101.67 00 37<CR><LF>
CH4 %VOL  temp C  pressure kPa  status  XOR of the first 25 bytes (hex)
```

Status bits: 4 temperature control, 3 air pressure, 2 light too weak, 1 light too strong,
0 absorption peak position. Any of them is a `fault`. The temperature is the compensation
reference, not the ambient temperature.

## SLTD-CH4-100

115200 8N1. Commands `AA len cmd valH valL [data] crcL crcH 55`, answered
`BB len cmd valH valL result crcL crcH 66`:

| Command | |
|---|---|
| `0x10` measurement on | valH = green indicator laser on, valL = static (1) / dynamic (0) |
| `0x11` measurement off | |
| `0x09` zero calibration, `0x0A` temperature compensation, `0x12` temperature calibration | not used |

While measuring it sends every 50 ms: `CC 09 01 c0 c1 c2 c3 l0 l1 t0 t1 crcL crcH 77`,
concentration uint32 ppm·m, light intensity, laser temperature (little endian). 100000 means
over range. The driver switches measurement on at start and off when it stops.
