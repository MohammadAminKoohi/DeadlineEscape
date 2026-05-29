# Escape from Shadow 🕹️

An interactive, IoT-based survival/top-down shooter game. In this project, the player uses a custom hardware controller (ESP32) to navigate their character through a dark, procedurally generated maze, surviving and escaping from actively pursuing shadows.

* **Course:** Embedded Systems - CE4621
* **Professor:** Dr. Ansari
* **Semester / Year:** Spring 2026

## Team Members

| Name | Student ID | GitHub Profile |
| :--- | :--- | :--- |
| MohammadAmin Koohi | 402106401 | [@mohammadaminkoohi](https://github.com/mohammadaminkoohi) |
| AmirHossein MohammadZadeh | 402106434 | [@ahmz1833](https://github.com/ahmz1833) |
| SeyedAhmad MousaviAval | 402106648 | [@seyedahmadmosaviawal](https://github.com/seyedahmadmosaviawal) |

---

## Project Architecture

The project is built on an **Authoritative Server** architecture, ensuring low bandwidth usage and stable gameplay. It is divided into three main components:

* **`/esp32` (Hardware Controller):** Contains the C++ (Arduino) code for the ESP32 microcontroller. It reads physical movement via an MPU-6050 accelerometer and aiming/shooting actions via an analog joystick. The normalized data is streamed to the server via WebSockets at a highly optimized rate of 5 Hz.

* **`/server` (Authoritative Node.js Server):** The core engine of the game. It handles all physics, collision detection (AABB), procedural maze generation, enemy AI, and line-of-sight (FOV) filtering. It processes inputs and broadcasts the current game state to the client at 10 Hz.

* **`/client` (Thin Web Client):** An HTML5 Canvas frontend that receives the game state and purely renders it. It features a follow-camera, flashlight mechanics (limited 50-degree FOV), and adaptive audio (adjusting heartbeat/footsteps volume based on the Euclidean distance to the enemy).

---

## Getting Started

### Prerequisites
* [Node.js](https://nodejs.org/) (v14 or higher)
* Arduino IDE or PlatformIO (for compiling ESP32 code)
* An ESP32 development board, MPU-6050 module, and an Analog Joystick.

### Installation & Execution
*(Detailed setup instructions for each component will be added here as development progresses.)*

1. **Hardware:** Flash the code inside `/esp32` to your board and update the WiFi credentials.
2. **Server:** Navigate to `/server`, run `npm install`, and start the server with `node server.js`.
3. **Client:** Open the `index.html` file inside the `/client` directory in any modern browser.

---
*Developed as a course project.*
