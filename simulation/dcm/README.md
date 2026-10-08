# DCM estimator check (MuJoCo)

Runs the firmware `tilt_estimation::DcmEstimator` (C++, via ctypes) inside the MuJoCo model during
FLASH stepping in place and compares it with the simulator's true CoM / DCM.

```bash
cmake -S simulation/dcm -B simulation/dcm/build
cmake --build simulation/dcm/build
./.venv/bin/python simulation/dcm/estimator_check.py            # summary table
./.venv/bin/python simulation/dcm/estimator_check.py --csv dcm_trace.csv
```

The torque model (STS3215 envelope, kp 28, kd 0.65) is shared with `simulation/rocking_sim.py`.
