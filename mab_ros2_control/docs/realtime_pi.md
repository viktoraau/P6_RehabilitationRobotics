# Realtime Scheduling on the Pi

`ros2_control` benefits a lot from realtime scheduling once the hardware loop starts pushing past `100 Hz`.

This package includes a helper script:

```bash
mab_ros2_control/scripts/setup_realtime_pi.sh status
sudo mab_ros2_control/scripts/setup_realtime_pi.sh apply
```

What it does:

- creates a `realtime` group if it does not exist
- adds your user to that group
- writes `/etc/security/limits.d/90-ros2-control-realtime.conf`

Installed limits:

```text
@realtime   -  rtprio   99
@realtime   -  nice    -20
@realtime   -  memlock unlimited
```

After applying the change:

1. Log out and back in.
2. Verify the shell limits changed:

```bash
ulimit -r
ulimit -l
```

3. Re-run the helper status check:

```bash
mab_ros2_control/scripts/setup_realtime_pi.sh status
```

If `ros2_control_node` still reports that FIFO scheduling could not be enabled, start by confirming:

- your user is really in the `realtime` group after re-login
- `ulimit -r` is non-zero in the shell that launches ROS
- the bringup is launched from a fresh shell after the login refresh
