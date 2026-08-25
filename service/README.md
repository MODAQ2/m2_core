# MODAQ 2.0 System Service Files

This directory contains `systemd` service files and configuration used to automatically start MODAQ 2.0 and synchronize the system clock on boot.

---

## Files Overview

| File | Description |
|------|-------------|
| `modaq.service` | Launches the MODAQ 2.0 ROS2 application on system startup |
| `phc2sys.service` | Synchronizes the system clock to the PTP Hardware Clock (PHC) |
| `ptp4l.service` | Runs the PTP daemon to sync the PHC to a grandmaster clock |
| `ptp4l.conf` | Configuration file for the `ptp4l` PTP daemon |

---

## PTP Setup

PTP (Precision Time Protocol) keeps the system clock tightly synchronized to a grandmaster clock over the network. Both `ptp4l` and `phc2sys` must be configured and running before MODAQ 2.0 starts.

### Step-by-Step

1. **Find your NIC** — Identify which network interface your PTP master clock is connected to:
   ```bash
   ifconfig
   ```
   Look for the interface name (e.g., `eth0`, `enp3s0`, `enp2s0`).

2. **Update `ptp4l.conf`** — Replace `enp2s0` with your interface name:
   ```bash
   # Example: change [enp2s0] to [eth0]
   nano ptp4l.conf
   ```

3. **Update `phc2sys.service`** — Replace `enp2s0` in the `ExecStart` line with your interface name:
   ```bash
   nano phc2sys.service
   # Change: ExecStart=/usr/sbin/phc2sys -w -s enp2s0 -u 5
   # To:     ExecStart=/usr/sbin/phc2sys -w -s eth0 -u 5
   ```

4. **Copy the config file:**
   ```bash
   sudo cp ptp4l.conf /etc/linuxptp/ptp4l.conf
   ```

5. **Copy the service files:**
   ```bash
   sudo cp ptp4l.service  /lib/systemd/system/ptp4l.service
   sudo cp phc2sys.service /lib/systemd/system/phc2sys.service
   ```

6. **Reload systemd and enable the services:**
   ```bash
   sudo systemctl daemon-reload
   sudo systemctl enable ptp4l.service
   sudo systemctl enable phc2sys.service
   ```

7. **Disable NTP** — NTP and PTP must not run simultaneously:
   ```bash
   sudo timedatectl set-ntp false
   ```

8. **Start the services:**
   ```bash
   sudo systemctl start ptp4l.service
   sudo systemctl start phc2sys.service
   ```

9. **Verify hardware timestamping support** on your NIC:
   ```bash
   # Replace eth0 with your interface name
   ethtool -T eth0
   ```
   The output should include `SOF_TIMESTAMPING_TX_HARDWARE` and `SOF_TIMESTAMPING_RX_HARDWARE`.

---

## MODAQ 2.0 Setup

### Installation

```bash
sudo cp modaq.service /etc/systemd/system/modaq.service
sudo systemctl daemon-reload
sudo systemctl enable modaq.service
```

### Starting / Stopping

```bash
# Start manually
sudo systemctl start modaq.service

# Stop
sudo systemctl stop modaq.service

# Restart
sudo systemctl restart modaq.service
```

### Viewing Logs

```bash
journalctl -u modaq.service -f
```

### Details

- Waits 2 seconds before starting to allow system services to settle
- Sources both the ROS2 Humble global environment and the local MODAQ2 workspace
- Runs `ros2 launch m2_launch m2_launch.py`

---

## Service Configuration Details

### `phc2sys.service`

Synchronizes the Linux system clock to the PTP Hardware Clock (PHC).

| Option | Value | Description |
|--------|-------|-------------|
| `-w` | — | Wait until `ptp4l` is synchronized before starting |
| `-s <NIC>` | Your interface | Use the PHC on this network interface as the clock source |
| `-u 5` | 5 seconds | Print clock offset statistics every 5 seconds |

### `ptp4l.conf`

| Parameter | Value | Description |
|-----------|-------|-------------|
| `time_stamping` | `hardware` | Use hardware timestamping for highest accuracy |
| `slaveOnly` | `1` | Sync to a master clock only, never become master |
| `clock_servo` | `linreg` | Use linear regression servo for smoother clock adjustment |
| `logging_level` | `4` | Standard informational logging |
| `step_threshold` | `15.0` | Allow a single step correction up to 15 seconds for initial sync |

---

## Useful Commands

```bash
# Check status of all services
sudo systemctl status ptp4l.service
sudo systemctl status phc2sys.service
sudo systemctl status modaq.service

# View live logs
journalctl -u ptp4l.service -f
journalctl -u phc2sys.service -f
journalctl -u modaq.service -f

# Check current time sync status
timedatectl status
```

---

## Requirements

- Ubuntu with ROS2 Humble or Jazzy installed
- MODAQ2 workspace built at `/home/m2/MODAQ2/` - modify if needed
- Network interface with PTP/hardware timestamping support
- `linuxptp` package installed:
  ```bash
  sudo apt install linuxptp
  ```
