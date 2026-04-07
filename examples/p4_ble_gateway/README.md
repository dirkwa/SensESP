# p4_ble_gateway

Signal K BLE gateway running on the Waveshare ESP32-P4-WIFI6-POE-ETH
board. Scans for nearby BLE devices, forwards their advertisements
and (optionally) GATT data to a [signalk-server](https://github.com/SignalK/signalk-server)
instance speaking the `ble-provider-api` dual-channel protocol.

This example is the third milestone (M3) in the ESP32-P4 SensESP work
that started with the transport-agnostic networking refactor in
[RFC #849](https://github.com/SignalK/SensESP/issues/849). It builds
directly on top of the M2 `EthernetProvisioner` and the
`NetworkProvisioner::mac_address()` API, and uses the M2 SK WebSocket
client to share the SignalK auth token across two parallel HTTP/WS
channels.

## Architecture

```
                         BLE peripherals nearby
                                  │
                                  │  GAP advertisements
                                  ▼
                ┌─────────────────────────────────┐
                │   ESP32-C6 (companion chip)     │
                │   esp_hosted_mcu slave fw       │
                │   NimBLE BLE controller         │
                └────────────┬────────────────────┘
                             │ SDIO 4-bit @ 40 MHz
                             ▼
┌──────────────────────────────────────────────────────────────────────┐
│  ESP32-P4 main MCU                                                   │
│                                                                      │
│  Arduino-ESP32 3.3.7  +  esp32-hal-hosted.c  +  esp-nimble-cpp 2.5.0 │
│  hostedInitBLE() → esp_hosted_bt_controller_init/enable             │
│  NimBLEDevice / NimBLEScan / NimBLEClient (host stack on the P4)    │
│                                                                      │
│  SensESP framework: hostname, web UI, OTA, NetworkStateProducer     │
│  EthernetProvisioner (M2) over native RMII to onboard IP101GRI PHY  │
│  SKWSClient: SignalK access-request flow + auth token persistence   │
│                                                                      │
│  Custom BLE-gateway code in this example:                            │
│   • NimBLE scan callback → in-memory advertisement dedup             │
│   • Background FreeRTOS task → batched HTTP POST every 2s            │
│   • Custom esp_websocket_client → /ble/gateway/ws GATT control       │
│   • 3 GATTSession slots → per-peripheral state machine               │
└──────────────────────────────┬───────────────────────────────────────┘
                               │ native RMII Ethernet (PoE optional)
                               ▼
                       Signal K server
              /signalk/v2/api/ble/gateway/advertisements (HTTP POST)
              /signalk/v2/api/ble/gateway/ws (WebSocket, GATT control)
```

The gateway speaks the **`ble-provider-api`** branch of signalk-server:

- **HTTP POST** `/signalk/v2/api/ble/gateway/advertisements` —
  `{ gateway_id, devices: [{ mac, rssi, name, manufacturer_data:
  {<decimal_company_id>: "<hex>"} }] }`. Posted every
  `POST_INTERVAL_MS` (2 s by default) from a background FreeRTOS
  task. Bearer-JWT auth header when SignalK security is enabled.
- **WebSocket** `/signalk/v2/api/ble/gateway/ws?token=<JWT>` —
  bidirectional. On connect the gateway sends a `hello` with
  hostname, MAC, firmware version, and GATT slot capacity. The
  server replies with `hello_ack` and from then on may send
  `gatt_subscribe` / `gatt_write` / `gatt_close` commands. The
  gateway responds with `gatt_connected` / `gatt_data` /
  `gatt_disconnected` / `gatt_error` and emits a periodic `status`
  every 30 s.

The protocol is implemented inside `src/main.cpp`; see the
[`ble-provider-api` branch of signalk-server](https://github.com/SignalK/signalk-server/tree/ble-provider-api)
for the server side.

## Hardware

- **Board**: Waveshare ESP32-P4-WIFI6-POE-ETH (ESP32-P4 ECO2 / rev v1.3
  silicon, 16 MB flash, 32 MB PSRAM, native RMII Ethernet to an
  IP101GRI PHY, optional 802.3af PoE through the RJ45 magjack,
  ESP32-C6-MINI companion chip on SDIO for WiFi6 / BLE5).
- **USB-C**: a single USB-C connector wired through a CH340 USB-UART
  bridge (QinHeng `1a86:55d3`) to the P4's UART0 (GPIO 37 TX / 38 RX).
  The P4's native USB-OTG is not exposed on this SKU, so
  `ARDUINO_USB_CDC_ON_BOOT` is left undefined and Arduino `Serial`
  reaches the host via UART0. On Linux the device shows up as
  `/dev/ttyACM0` on recent kernels (`cdc-acm` bound to the newer
  CH340 variant) or `/dev/ttyUSB0` on older systems with the legacy
  vendor driver.
- **Ethernet**: standard RJ45 patch cable to a switch / router that
  serves DHCP. For isolated bench testing, a Linux host running
  `nmcli connection add type ethernet ifname ethX con-name test
  ipv4.method shared ipv4.addresses 192.168.99.1/24` will spin up a
  NetworkManager-managed `dnsmasq` and provide both DHCP and NAT to
  any upstream interface.

## Prerequisites

1. **C6 slave firmware must be flashed.** This board ships with no
   compatible `esp_hosted_mcu` slave firmware on the C6, so
   `hostedInitBLE()` will fail and the gateway will refuse to start.
   Run the [`p4_c6_ota_updater`](../p4_c6_ota_updater/) example once
   to download `esp_hosted_mcu` v2.11.6 from Espressif's CDN and
   flash it to the C6 over SDIO. After running it, verify with the
   [`p4_esp_hosted_probe`](../p4_esp_hosted_probe/) example —
   slave firmware version should report a non-zero number.

2. **Signal K server on the `ble-provider-api` branch.** The protocol
   endpoints (`/signalk/v2/api/ble/gateway/*`) only exist on that
   branch — they are not in any released signalk-server yet. Clone
   <https://github.com/SignalK/signalk-server>, check out
   `ble-provider-api`, build with `npm install && npm run build`,
   start with `npm start` (or systemd if you have a service unit).
   It will listen on port 3000 by default.

3. **PR #940 (M2 transport-agnostic networking)**. This example uses
   `EthernetConfig::waveshare_esp32p4_poe()` and
   `NetworkProvisioner::mac_address()` from M2. The example consumes
   the local SensESP tree via `symlink://../..` so it picks them up
   automatically as long as you're on a branch that has M2 in it.

## Build and flash

```sh
cd examples/p4_ble_gateway
pio run -t upload   # PlatformIO auto-detects the serial port
pio device monitor  # 115200 bps
```

The build pulls in `h2zero/esp-nimble-cpp` 2.5.0 from GitHub via
`lib_deps` (an IDF-flavoured C++ NimBLE wrapper that supports
ESP32-P4 + ESP-Hosted BT VHCI; the more familiar Arduino-flavoured
[`h2zero/NimBLE-Arduino`](https://github.com/h2zero/NimBLE-Arduino)
does *not* compile on P4 because of an unconditional `esp_bt.h`
include — see [NimBLE-Arduino#906](https://github.com/h2zero/NimBLE-Arduino/issues/906)).
The API surface is identical (`NimBLEDevice`, `NimBLEScan`,
`NimBLEClient` etc.), so existing NimBLE-Arduino code drops in with
no source changes.

Flash size will be ~1.7 MB (about 86 % of the `min_spiffs.csv` 1.9 MB
app slot). RAM usage at boot is ~38 KB internal SRAM; the rest of the
576 KB SRAM stays free for NimBLE buffers and the SensESP HTTP server.

## Expected serial output

```text
[hosted] boot
...
I (4257) net_state: Ethernet got IP
I (4928) BLE-GW: ESP-Hosted slave firmware: 2.11.6
I (4946) BLE-GW: BLE scan started
I (4946) BLE-GW: Setup complete, heap=387528
I (4952) signalk_ws_client.cpp: Initiating websocket connection with server...
I (4975) signalk_ws_client.cpp: Server security disabled (404 on access request) — connecting without token
I (4993) BLE-GW: SK server connected — starting BLE gateway services
I (4997) BLE-WS: Connecting to ws://192.168.99.1:3000/signalk/v2/api/ble/gateway/ws?token=
I (5310) BLE-WS: Connected
I (5311) BLE-WS: Sent hello
I (5355) BLE-WS: Hello acknowledged
I (8963) BLE-GW: POST: forwarded 1 device(s), heap=328580
I (10979) BLE-GW: POST: forwarded 1 device(s), heap=328320
...
```

The number of devices per POST tracks how many unique BLE MACs the
scanner has seen since the last batch (re-deduplicated per MAC).

## Verifying on the server

After ~10 s the gateway should appear in the server's BLE provider
list. From any host that can reach the SignalK server:

```sh
curl -s http://<signalk-server-ip>:3000/signalk/v2/api/ble/gateways \
    | python3 -m json.tool
```

```json
[
    {
        "gatewayId": "signalk-ble-gw",
        "providerId": "ble:gateway:signalk-ble-gw",
        "online": true,
        "ipAddress": "::ffff:192.168.99.13",
        "mac": "80:F1:B2:D2:CC:6E",
        "hostname": "signalk-ble-gw",
        "firmware": "2.0.0",
        "gattSlots": { "total": 3, "available": 3 },
        "deviceCount": 4
    }
]
```

The `mac` field is populated from
`SensESPApp::get()->get_network_provisioner()->mac_address()` — the
M2 transport-agnostic accessor — which on this board returns the
ETH MAC. On a future SensESP build that runs over WiFi or PPP, the
same call would return the WiFi STA / PPP MAC, and the gateway
identification would still work without any change to this example.

## Configuration

| Knob | Where | Default | Notes |
|---|---|---|---|
| `GATEWAY_HOSTNAME` | `platformio.ini` build_flags | `signalk-ble-gw` | Used as both the SensESP hostname and the BLE provider `gateway_id`. Override per-device when running multiple gateways on the same SK server. |
| `POST_INTERVAL_MS` | `src/main.cpp` | 2000 | How often the background task drains the advertisement buffer to the server. Lower for fresher data, higher for less network chatter. |
| `MAX_GATT_SESSIONS` | `src/main.cpp` | 3 | Concurrent GATT peripheral connections this gateway will accept. Bound by NimBLE host config and SDIO bandwidth — 3 is comfortable on this board. |
| `STATUS_INTERVAL_MS` | `src/main.cpp` | 30000 | How often the gateway sends a `status` message over the BLE WS so the server can show fresh `freeHeap` / `uptime` / slot counts. |
| `GATT_RECONNECT_DELAY_MS` | `src/main.cpp` | 3000 | Base backoff after a GATT connection failure (multiplied by attempt number). |
| `GATT_MAX_RETRIES` | `src/main.cpp` | 5 | Per-session retry budget before giving up and emitting `gatt_error` to the server. |
| OTA password | `setup()` `enable_ota("…")` | `ble-gw-ota` | Standard SensESP OTA password. Change before deployment. |

Once the device has booted and joined the network at least once,
its hostname / SK server URL / OTA password / etc. are also
configurable via the SensESP web UI at `http://<gateway-ip>/`.

## What this example proves about the M2 + M3 stacks

This is the first SensESP application that exercises every layer of
the transport-agnostic networking refactor in a realistic
production-style configuration:

- **Two parallel WS connections to the same SK server.** SensESP's
  built-in `SKWSClient` handles `/signalk/v1/stream` (the regular SK
  delta stream) for the device's own sensors; the BLE gateway uses
  the new `ble-provider-api` `/signalk/v2/api/ble/gateway/ws`
  endpoint via a separately-managed `esp_websocket_client`. They
  share authentication state through `SKWSClient::get_auth_token()`.
- **`NetworkProvisioner::mac_address()` → wire protocol field.** The
  `mac` field that signalk-server stores for this gateway comes
  directly from M2's transport-agnostic accessor. There is no
  WiFi-specific code in the gateway path.
- **Cross-task / cross-stack data flow.** A NimBLE callback running
  on the C6's BT host task queues advertisements into a mutex-guarded
  buffer; a FreeRTOS task on the P4 drains the buffer and POSTs over
  HTTPS using the SensESP-acquired auth token; a separate WebSocket
  client handles GATT command/response over the same TCP/IP stack.
  All three are framework-agnostic — no piece of this code knows or
  cares whether the underlying link is WiFi, Ethernet, or future
  PPP/cellular.
