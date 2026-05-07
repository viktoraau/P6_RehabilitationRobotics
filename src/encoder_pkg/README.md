# encoder_pkg — Encoder Diagnostic Reader

Simple diagnostic node for checking encoder readings during hardware bring-up and debugging.

---

## Node: `encoder_data`

Subscribes to `/joint_states` and prints **joint 3** position and effort to the terminal.

- Position is printed in **degrees × 10** (e.g., 3600 = 360°)
- Effort is printed in **Nm**

```bash
ros2 run encoder_pkg encoder_data
```

Expected output:
```
[encoder_reader]: Joint 3 position: 0.0 deg, Torque: 0.0 Nm
```

---

## Notes

- Only monitors `joint_3` — edit `encoder_data.py` to change which joint is printed
- `testdir.py` is a helper for checking file paths during development (not a ROS node)
- For full joint state monitoring, use `ros2 topic echo /joint_states` instead
