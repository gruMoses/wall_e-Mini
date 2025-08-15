## Bluetooth setup and pairing

Adapter MAC: 88:A2:9E:03:1A:07

### One-time prerequisites
- Ensure the Bluetooth service is running: `systemctl is-active bluetooth`
- Install tooling: `sudo apt-get update && sudo apt-get install -y bluez`

### Enable legacy SDP (compat mode) for SPP
BlueZ must run with `--compat` to allow PyBluez (or sdptool) to register the Serial Port Profile (SPP).

Create a systemd drop-in and restart the service:
```bash
sudo mkdir -p /etc/systemd/system/bluetooth.service.d
printf "[Service]\nExecStart=\nExecStart=/usr/libexec/bluetooth/bluetoothd --compat\n" | sudo tee /etc/systemd/system/bluetooth.service.d/override.conf >/dev/null
sudo systemctl daemon-reload
sudo systemctl restart bluetooth
```

If the controller powers off during the restart, power it back on:
```bash
bluetoothctl power on
```

### Make the Pi always discoverable (installed by our service)
We install a small systemd unit that, on boot, sets:
- `agent on`, `default-agent`
- `pairable on`
- `discoverable on`
- `discoverable-timeout 0` (never times out)

To (re)install manually:
```bash
sudo install -m 0755 /home/pi/bin/bt_always_discoverable.sh /usr/local/bin/bt_always_discoverable.sh
sudo install -m 0644 /home/pi/bin/bt_always_discoverable.service /etc/systemd/system/bt_always_discoverable.service
sudo systemctl daemon-reload
sudo systemctl enable --now bt_always_discoverable.service
```

### Pair from Android
```bash
bluetoothctl
scan on
# find your phone MAC as it appears
pair <PHONE_MAC>
trust <PHONE_MAC>
connect <PHONE_MAC>
```

### Protocol notes (final)
- We use Classic Bluetooth SPP (RFCOMM). Android connects with SPP UUID `00001101-0000-1000-8000-00805F9B34FB`.
- The server sends a greeting: `SRV:HELLO ver=2 sn=<nonce>` (nonce is informational).
- We accept two message formats from Android without authentication:
  - `CMD2:<left_i>;<right_i>;<seq>;<ts_ms>;<sn_hex>;<hmac_hex>`
    - HMAC field is ignored; only structure and increasing `<seq>` are enforced.
    - `<left_i>`/`<right_i>` are signed integers in approximately [-1000, 1000].
  - `V1:<left_f>;<right_f>;<seq>` where floats are in [-1.0, 1.0].
- Sequence numbers must strictly increase per connection.

### Run an RFCOMM SPP server (for testing)
Install PyBluez from apt (avoids pip restrictions):
```bash
sudo apt-get install -y python3-bluez
```

Start the simple SPP server (root required for SDP advertise):
```bash
sudo python3 /home/pi/pi_app/cli/spp_server.py
```
This advertises the SPP UUID `00001101-0000-1000-8000-00805F9B34FB` as "WALL-E Control" so Android can open an RFCOMM socket instead of defaulting to an audio profile.

### Useful checks
```bash
systemctl is-active bluetooth
bluetoothctl show
rfkill list
```

### Things that mattered (what we learned)
- BlueZ must run in `--compat` mode or SDP registration from PyBluez will fail (`No such file or directory`/permission errors).
- Advertising SPP requires root (run the server with `sudo`).
- Keep the adapter discoverable and pairable persistently via a boot-time script/service; otherwise Android may prefer audio profiles.
- Fragmented RFCOMM reads are common; reassemble lines before parsing.
- Android devices can default to A2DP/HFP; explicitly connecting with SPP by UUID avoids the “audio device” path.


