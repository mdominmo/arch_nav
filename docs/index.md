
<p align="center">
  <img src="img/arch_nav_logo.png" alt="Arch Nav logo" width="200">
</p>

<h1 align="center">Arch Nav</h1>

<p align="center">
  <em>Platform-agnostic UAV navigation kernel</em>
</p>

<p align="center">
  <a href="getting-started/installation/">Get Started</a> &nbsp;|&nbsp;
  <a href="https://github.com/mdominmo/arch-nav">GitHub</a>
</p>

---

## What is Arch Nav

Arch Nav is a navigation kernel that decouples UAV application logic from the underlying autopilot. It provides a single API that works across PX4, ArduPilot, or any autopilot for which a driver plugin exists. Applications built on Arch Nav can switch autopilot stacks without rewriting navigation code.

---

## Who is this for

- **Drone developers** who need a clean abstraction over different autopilot stacks.
- **Research groups** building experimental navigation and planning systems without being locked to a single platform.
- **Companies** that develop and maintain robust UAV control software across multiple vehicle platforms.

---

## Development

- **[Build & Test](development/build-and-test.md)** — How to build the library, run tests, and integrate in your project.
- **[Writing Drivers](development/writing-drivers.md)** — Step-by-step guide to implementing a driver for a new autopilot.
- **[Concurrency Notes](development/concurrency-notes.md)** — Threading model and synchronization guarantees.

---

## Getting Started

- **[Installation](getting-started/installation.md)** — Dependencies and build instructions.
- **[First Run](getting-started/first-run.md)** — A minimal mission flow from arm to waypoint following.

---

## Citing this tool

```bibtex
@software{dominguez2025archnav,
  author       = {Dominguez, Manuel},
  title        = {Arch Nav: A Platform-Agnostic UAV Navigation Kernel},
  year         = {2025},
  url          = {https://github.com/mdominmo/arch-nav}
}
```

---

<p align="center"><em>Created by Manuel Dominguez</em></p>
