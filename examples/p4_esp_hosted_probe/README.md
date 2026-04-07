# p4_esp_hosted_probe

Minimal smoke test for the ESP-Hosted-MCU SDIO link between the
ESP32-P4 and the onboard ESP32-C6 companion chip on the Waveshare
ESP32-P4-WIFI6-POE-ETH board. Calls `hostedInitBLE()` (the
Arduino-ESP32 3.3.7 wrapper around `esp_hosted_init` +
`esp_hosted_connect_to_slave` + `esp_hosted_bt_controller_init/enable`)
and reports whether the SDIO bus came up, what slave firmware
version the C6 is running, and whether the BT controller is active.

This is **step 1 of bringing up BLE on this board** — verify the
hardware path before bothering to build the full BLE gateway. If the
probe reports `slave firmware version: 0.0.0` (or `rpc_core: Response
not received`), the C6 is either unflashed or running an
incompatible image. Run [`p4_c6_ota_updater`](../p4_c6_ota_updater/)
once to fix that.

## Build, flash, run

```sh
cd examples/p4_esp_hosted_probe
pio run -t upload
pio device monitor    # 115200 bps
```

No external libraries — the probe uses only the Arduino-ESP32
core's built-in `hostedInitBLE()` exposed via `esp32-hal-hosted.h`.
The matching IDF managed component (`espressif/esp_hosted`) is
already declared as a dependency by Arduino-ESP32 3.3.7's own
`idf_component.yml`, so PlatformIO pulls it automatically when
`CONFIG_ESP_HOSTED_ENABLE_BT_NIMBLE=y` is set in
`sdkconfig.defaults`.

## Expected output (success)

```text
[hosted] boot
[hosted] chip model=18 rev=103 cores=2
[hosted] free_heap=562608 free_psram=33549824
[hosted] calling hostedInitBLE()...
[hosted] SDIO pins: clk=18 cmd=19 d0=14 d1=15 d2=16 d3=17 reset=54
[hosted] hostedInitBLE() OK
[hosted] host esp_hosted version: 2.11.6
[hosted] slave firmware version: 2.11.6
[hosted] hostedIsInitialized=1 hostedIsBLEActive=1
[hosted] PASS: C6 + BT controller are up — proceed with BLE gateway
```

## Expected output (failure: C6 firmware missing)

```text
[hosted] boot
[hosted] calling hostedInitBLE()...
[hosted] SDIO pins: clk=18 cmd=19 d0=14 d1=15 d2=16 d3=17 reset=54
E (5131) rpc_core: Response not received for [0x15e](Req_GetCoprocessorFwVersion)
E (5131) ARDUINO: Could not get slave firmware version: ESP_FAIL
[hosted] hostedInitBLE() OK
[hosted] host esp_hosted version: 2.11.6
[hosted] slave firmware version: 0.0.0 (unknown — slave may not have responded)
```

`hostedInitBLE()` may still return `true` in this state because the
Arduino wrapper's BT-controller-init path is silently skipped when
`slave_version < 2.6.0` and the version check fails open. The
"PASS" message at the end is misleading in this case — what
matters is whether `slave firmware version` is non-zero. Fix:
flash the C6 via [`p4_c6_ota_updater`](../p4_c6_ota_updater/), then
re-flash the probe and re-check.

## SDIO pin reference

The SDIO pinout is taken from the Arduino-ESP32 `esp32p4` board
variant's `BOARD_SDIO_ESP_HOSTED_*` macros and matches the
Waveshare board's wiring exactly:

| Signal | ESP32-P4 GPIO |
|---|---|
| CLK | 18 |
| CMD | 19 |
| D0 | 14 |
| D1 | 15 |
| D2 | 16 |
| D3 | 17 |
| Slave RESET | 54 |

If you ever want to use this code on a different ESP32-P4 board
whose SDIO wiring differs, override the variant macros via
`-DBOARD_SDIO_ESP_HOSTED_*` in `platformio.ini` `build_flags` or
call `hostedSetPins(clk, cmd, d0, d1, d2, d3, rst)` before
`hostedInitBLE()`.
