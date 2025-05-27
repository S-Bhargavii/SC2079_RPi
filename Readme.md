# SC2079 RPi Code

This repository contains the Raspberry Pi (RPi) codebase developed by **Group 3** for the **SC2079 Multidisciplinary Project (MDP)** at Nanyang Technological University (NTU).

The Raspberry Pi acts as the central controller, handling communication with the Android tablet, STM32 microcontroller, and an external image recognition server. It coordinates tasks such as movement commands, image streaming, and sensor data processing.

---

## 📡 Communication Protocols

The system uses a mix of wired and wireless communication protocols for robust and flexible data exchange:

- **Bluetooth** – For sending commands and receiving data from the STM32 microcontroller.
- **Wi-Fi** – For communication with the Android tablet and the image recognition server.
- **USB Serial** – Direct serial communication with STM32 for reliable control.
- **`imagezmq`** – Lightweight image transmission from RPi to the image recognition server.
- **`zmq` (ZeroMQ)** – High-performance asynchronous messaging between different components.

---

## 🧵 Concurrency

To handle multiple tasks simultaneously and improve system responsiveness, this project utilizes:

- **Multi-threading** – Used for non-blocking communication (e.g., listening and sending over Bluetooth or serial ports concurrently).
- **Multi-processing** – Used for computationally intensive tasks like handling image recognition and streaming, to fully utilize multiple CPU cores.

---

## 📁 Repository Structure

- `checklist/` – Pre-run checks to validate component connectivity.
- `communication/` – Communication handling between RPi, tablet, STM32, and servers.
- `servers/` – Server-side interfaces for image recognition or task coordination.
- `task_1/` – Logic for Task 1: navigation and basic obstacle avoidance.
- `task_2/` – Logic for Task 2: symbol/image recognition and advanced control.
- `test/` – Unit and integration tests for various modules.
- `utility/` – Helper scripts and utility functions.

---
