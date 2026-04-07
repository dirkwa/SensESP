# p4_c6_ota_updater

One-shot updater that flashes the ESP-Hosted-MCU slave firmware
onto the ESP32-C6 companion chip on a Waveshare ESP32-P4-WIFI6-POE-ETH
board, over SDIO from the P4 side, with the binary downloaded
over HTTP from Espressif's CDN. No jumpers, no download mode, no
extra serial adapter — uses only the existing USB-C and Ethernet
connections.

## When you need this

The Waveshare board ships with no compatible `esp_hosted_mcu` slave
firmware on the C6. The host-side `esp_hosted` component bundled with
Arduino-ESP32 3.3.7 expects a slave running v2.11.6 (or whatever
version matches the host). Without a matching slave, calls to
`esp_hosted_init()` succeed at the SDIO bus level but every RPC
request times out — including the version query and the BT
controller init — so BLE never comes up.

Symptoms when you skip this step (run [`p4_esp_hosted_probe`](../p4_esp_hosted_probe/)
first to confirm):

```text
E (5131) rpc_core: Response not received for [0x15e](Req_GetCoprocessorFwVersion)
[hosted] slave firmware version: 0.0.0
```

After running this updater once, the probe will report:

```text
[hosted] slave firmware version: 2.11.6
[hosted] hostedIsInitialized=1 hostedIsBLEActive=1
```

…and the [`p4_ble_gateway`](../p4_ble_gateway/) example will
actually be able to scan BLE traffic.

## How it works

1. Brings up the P4's native RMII Ethernet via SensESP's M2
   `EthernetProvisioner` (Waveshare PoE config). DHCP from any
   server on the LAN.
2. Subscribes to the SensESP `NetworkStateProducer` and waits for
   `kNetworkConnected`.
3. Calls `hostedInitBLE()` to bring up the SDIO bus to the C6.
   This succeeds at the bus level even when the slave firmware is
   absent — it just means subsequent RPC calls will fail until the
   slave boots a real image.
4. Calls `hostedHasUpdate()` which queries the slave version. With
   a blank C6 the slave reads as 0.0.0; with the host at 2.11.6,
   `host > slave` so the function returns `true` and logs the
   matching CDN URL via `hostedGetUpdateURL()`. The URL format is
   `https://espressif.github.io/arduino-esp32/hosted/<target>-v<x>.<y>.<z>.bin`,
   where `<target>` is `esp32c6` for this board.
5. Streams the binary down over HTTPS using `HTTPClient` (1.19 MB
   for v2.11.6, takes ~15 s on a 100 Mbps link). Each chunk is
   passed straight to `hostedWriteUpdate()` which writes it to the
   C6 via SDIO using the `esp_hosted_slave_ota_*` IDF API.
6. Calls `hostedEndUpdate()` and `hostedActivateUpdate()`. The
   activate may legitimately fail on a slave that previously had
   no firmware at all — that is not fatal; the new image is
   already written, the next boot picks it up.
7. Reboots the P4 (which power-cycles the C6 too via the SDIO
   reset GPIO) and re-runs the whole flow. On the second boot the
   slave reports the same version as the host, `hostedHasUpdate()`
   returns `false`, and the updater is a no-op.

## Build, flash, run

```sh
cd examples/p4_c6_ota_updater
pio run -t upload
pio device monitor
```

The first boot after flashing this firmware will perform the OTA
once. After that the firmware enters a "boot → check → no update
needed → reboot" loop — that is intentionally idempotent so the
flow is safe to leave running. To stop the loop, just flash a
different firmware to the P4.

## Expected output (first run)

```text
[c6_ota] boot
[c6_ota] chip model=18 rev=103 cores=2
[c6_ota] free_heap=538580 free_psram=33549824
[c6_ota] pre-init host=2.11.6 slave=0.0.0
[c6_ota] waiting for ethernet link + DHCP...
[c6_ota] network up: 192.168.99.13 (gw 192.168.99.1, mac 80:F1:B2:D2:CC:6E)
[c6_ota] === starting C6 slave OTA update ===
[c6_ota] post-init host=2.11.6 slave=0.0.0
[c6_ota] update URL: https://espressif.github.io/arduino-esp32/hosted/esp32c6-v2.11.6.bin
[c6_ota] downloading 1191424 bytes...
[c6_ota]   16384 / 1191424 bytes
[c6_ota]   32768 / 1191424 bytes
   ... (15 seconds, 1.19 MB streamed) ...
[c6_ota]   1191424 / 1191424 bytes
[c6_ota] download+stream complete: 1191424 bytes in 15533 ms
[c6_ota] activating new slave firmware...
[c6_ota] === DONE — rebooting in 3 seconds ===
[c6_ota] OTA result: OK
```

## Expected output (subsequent runs)

```text
[c6_ota] post-init host=2.11.6 slave=2.11.6
[c6_ota] hostedHasUpdate() returned false. Either the slave is
         already at the host's version or the slave is newer than
         the host. Nothing to do.
[c6_ota] OTA result: OK
```

## Prerequisites

- The P4 needs to reach `https://espressif.github.io` over HTTP/HTTPS,
  i.e. the local network needs upstream internet routing. On a
  bench setup with a NetworkManager `ipv4.method shared` interface,
  the host's MASQUERADE rule handles this automatically.
- PR #940 (M2 transport-agnostic networking) — this example uses
  `EthernetConfig::waveshare_esp32p4_poe()` from M2.

## Why not use the USB-C download-mode flash flow instead?

The Waveshare wiki documents an alternative flashing path: pull
`C6_IO9` low at power-on to put the C6 into download mode, put the
P4 into download mode too, then flash the C6 via the USB-C
pass-through using `C6_U0RXD`/`C6_U0TXD`. That works, but:

- It requires physical access to the `C6_IO9` test point (uncertain
  whether it is broken out to a labelled header on every SKU).
- It needs a specific power-sequencing dance between the BOOT
  buttons.
- It is a one-shot operation that you cannot trigger over the
  network for a deployed device.

The OTA flow in this updater works over the same Ethernet link the
device already uses for its normal traffic, requires no jumpers,
and can be re-run remotely whenever the slave firmware needs an
update.
