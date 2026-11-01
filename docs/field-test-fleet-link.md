# Field Test: Fleet Link & Zone Arbiter

- **Date:** 2026-10-24
- **Robot:** Ferb (collector 2)
- **Target Systems:** Perry (zone arbiter, overhead tracker) & Phineas (collector 1)
- **Platform:** Raspberry Pi 5 · Ubuntu 26.04 LTS · ROS 2 Lyrical

---

## Objective

Validate network connectivity, DDS discovery, topic heartbeats, and service invocation between Ferb's onboard computer and Perry's `zone_arbiter` on the shared track network.

---

## Pre-flight Checklist

1. **Network connection:** Ensure Ferb's Pi 5 is connected to the designated fleet Wi-Fi network (hotspot or router).
   ```bash
   ip a show wlan0
   ping -c 3 perry.local
   ```
2. **Time synchronization:** Confirm `chrony` is synchronized across all 3 Pis to prevent message timestamp rejections:
   ```bash
   chronyc sources
   chronyc tracking
   ```
3. **ROS domain environment:** Ensure all nodes share the agreed `ROS_DOMAIN_ID`:
   ```bash
   echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
   # If not set: export ROS_DOMAIN_ID=0
   ```
4. **Fallback peer discovery:** If the Wi-Fi router or phone hotspot blocks UDP multicast, define static peers:
   ```bash
   export ROS_STATIC_PEERS="<perry_ip>;<phineas_ip>"
   ```

---

## Test Steps

### Step 1. Listen for the Zone Occupant Heartbeat

Perry's `zone_arbiter` continuously publishes `/zone/occupant` at 2 Hz with `RELIABLE` and `TRANSIENT_LOCAL` QoS.

Run on Ferb:
```bash
ros2 topic echo /zone/occupant
```

**Expected output:**
```yaml
data: ""
---
```
*(An empty string indicates that the delivery zone is currently unoccupied and free. A non-empty string such as `"phineas"` indicates that the other robot holds the lease. Heartbeats should arrive consistently every 0.5 s).*

### Step 2. Request Zone Access for Ferb

Request a 60-second delivery lease from the arbiter:
```bash
ros2 service call /zone/request multirobot_interfaces/srv/RequestZone "{robot: ferb}"
```

**Expected response:**
```yaml
granted: true
reason: "granted"
```

### Step 3. Confirm Occupant Status

Verify that the occupant heartbeat now reflects Ferb's lease:
```bash
ros2 topic echo /zone/occupant --once
```

**Expected output:**
```yaml
data: "ferb"
```

### Step 4. Release Zone and Confirm Delivery Count

Simulate releasing the zone after delivering a green cube:
```bash
ros2 service call /zone/release multirobot_interfaces/srv/ReleaseZone "{robot: ferb, delivered: true, color: 'green'}"
```

**Expected response:**
```yaml
ok: true
delivered_total: 1
```

Check that the zone occupant returns to empty:
```bash
ros2 topic echo /zone/occupant --once
```

**Expected output:**
```yaml
data: ""
```

---

## Troubleshooting

- **No messages received on `/zone/occupant`:**
  - Verify that Perry is running `zone_arbiter_node`.
  - Check QoS settings: the subscriber must use `transient_local` durability and `reliable` reliability.
  - If multicast is blocked, ensure `ROS_STATIC_PEERS` includes Perry's IP address.
- **Service call times out / service not available:**
  - Run `ros2 service list` to verify `/zone/request` and `/zone/release` are visible over the DDS network.
  - Test connectivity with `ros2 doctor --report`.
- **Clock desynchronization:**
  - Overhead poses and sensor messages with timestamps older than `0.5 s` will be discarded by `PeerAvoidance`. Verify `chrony` status.
- **Safety reminder:**
  - When running live drive tests, `obstacle_guard` stops the motors if `/zone/occupant` is missing for more than 2.0 s (`fleet/ok=false`). Always keep a hand near the robot's hardware battery switch.
