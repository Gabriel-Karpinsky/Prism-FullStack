# Lab Bring-Up Checklist

Use this checklist for the first bench test of the MVP stack.

## 1. Flash the Arduino Mega

From [firmware/arduino-mega](/E:/_Data/_TUE/Prism/Codex web/firmware/arduino-mega):

```bash
pio run --target upload
```

Verify before power-up:

- stepper drivers are set to `128` microsteps
- yaw/pitch step and direction pins match [HardwareConfig.h](/E:/_Data/_TUE/Prism/Codex web/firmware/arduino-mega/include/HardwareConfig.h)
- the Arduino trigger output is wired if you want the Garmin sync pulse
- the gantry can move safely within the configured soft limits

## 2. Install the Pi services

```bash
cd /opt/cliffscanner
sudo bash deploy/pi/install-edge-daemon.sh
sudo bash deploy/pi/install-control-api-service.sh
sudo bash deploy/pi/configure-tailscale-serve.sh
```

## 3. Confirm both services are up

```bash
systemctl status cliffscanner-edge --no-pager
systemctl status cliffscanner-control-api --no-pager
curl http://127.0.0.1:9090/health
curl http://127.0.0.1:8080/healthz
```

## 4. Confirm the Arduino link

```bash
journalctl -u cliffscanner-edge -n 100 --no-pager
curl http://127.0.0.1:9090/api/hardware/state
```

Expected checks:

- `connected` should be `true`
- `mode` should not be `fault`
- no serial-port error should be present in `faults`

## 5. Motion bench test

Before enabling full travel, reduce scan bounds or keep a hand on emergency stop.

From the web UI:

1. acquire control
2. press `Connect` if needed
3. jog yaw positive and negative
4. jog pitch positive and negative
5. home back toward zero
6. verify reported yaw/pitch follow the physical gantry direction

If an axis moves backward, swap either the driver wiring or the direction polarity in firmware logic.

## 6. First scan test

1. set resolution to `low`
2. start scan
3. verify the gantry follows a raster pattern
4. verify the edge daemon updates `coverage` and `scanProgress`
5. verify the browser map fills cell by cell

## 7. Garmin validation

If scans move correctly but map values are wrong:

- verify `EDGE_LIDAR_BUS` and `EDGE_LIDAR_ADDRESS`
- run `i2cdetect -y 1` on the Pi
- confirm the Garmin has stable power and ground
- temporarily switch `EDGE_USE_MOCK_LIDAR=true` to separate motion issues from lidar issues

## 8. Safety checks

- unplug the browser client or stop the edge daemon and verify the Arduino watchdog faults motion
- trigger `ESTOP` from the UI and verify the gantry stops
- use `CLEAR_FAULT` only after the mechanism is safe to move again
