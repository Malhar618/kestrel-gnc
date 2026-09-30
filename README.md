# kestrel-gnc
Embedded-style C++ GNC flight software for a quadrotor (then a second vehicle), flown against
its own 6-DOF simulator and verified by unit tests, Monte Carlo, software-in-the-loop and
processor-in-the-loop on an emulated STM32.

**Status:** M2 done: cascaded position/attitude/rate control and a motor mixer in the flight software, flying the simulated quadrotor.

## Quick start
    cmake --preset dev && cmake --build --preset dev && ctest --preset dev

Fly a scenario and look at it. Closed loop (the flight software's controller): `hold`,
`pos_step`, `att_step`, `yaw_step`, `square`, `wind_hold`. Open loop (motors only): `hover`,
`free_fall`, `yaw_spin`, `pitch_kick`.

    ./build/dev/kestrel_sim square build/square.csv
    uv run tools/plot_run.py build/square.csv      # figures in build/figures/
    uv run tools/replay_rerun.py build/square.csv  # 3D replay in the Rerun viewer

## Roadmap
- [x] M0 Repo, build, CI
- [x] M1 6-DOF quadrotor plant
- [x] M2 Cascaded PID on truth state
- [ ] M3 Error-state EKF in the loop (MVP)
- [ ] M4 Monte Carlo + CI regression
- [ ] M5 Minimum-snap guidance + LQR
- [ ] HW Real drone: sim-to-real (Jetson + PX4)
- [ ] M6 Software-in-the-loop over MAVLink
- [ ] M7 Processor-in-the-loop on an emulated STM32 (Renode)
- [ ] M8 Real hardware or second vehicle

Conventions: [docs/conventions.md](docs/conventions.md) · Architecture: [docs/architecture.md](docs/architecture.md) · Control: [docs/control.md](docs/control.md)
