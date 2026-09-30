# kestrel-gnc
Embedded-style C++ GNC flight software for a quadrotor (then a second vehicle), flown against
its own 6-DOF simulator and verified by unit tests, Monte Carlo, software-in-the-loop and
processor-in-the-loop on an emulated STM32.

**Status:** M1 in progress: math core (quaternions, DCMs, Euler 3-2-1, RK4) done.

## Quick start
    cmake --preset dev && cmake --build --preset dev && ctest --preset dev

## Roadmap
- [x] M0 Repo, build, CI
- [ ] M1 6-DOF quadrotor plant
- [ ] M2 Cascaded PID on truth state
- [ ] M3 Error-state EKF in the loop (MVP)
- [ ] M4 Monte Carlo + CI regression
- [ ] M5 Minimum-snap guidance + LQR
- [ ] HW Real drone: sim-to-real (Jetson + PX4, winter break)
- [ ] M6 Software-in-the-loop over MAVLink
- [ ] M7 Processor-in-the-loop on an emulated STM32 (Renode)
- [ ] M8 Real hardware or second vehicle

Conventions: [docs/conventions.md](docs/conventions.md) · Architecture: [docs/architecture.md](docs/architecture.md)
