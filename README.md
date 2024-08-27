# Achieving Stable Flight with Custom Drone Control Systems

This project focuses on achieving stable flight for a drone by replacing the stock logic hardware and software with custom-built components. The goal is to interface with the high-power components of a race drone using a custom-designed control system, ensuring stability, responsiveness, and safety in flight.

## Table of Contents

- [Hardware Considerations](#hardware-considerations)
- [Software Considerations](#software-considerations)
- [Flight Software Demonstration](#flight-software-demonstration)
- [Custom Circuit Board Features](#custom-circuit-board-features)
- [Flight Software Adjustments](#flight-software-adjustments)
- [Video Demonstration](#video-demonstration)

## Hardware Considerations

- **High-Power Components Integration**: Reused high-power components from a race drone, replacing the main computer and software with custom-built logic hardware.
- **IMU Mounting**: The IMU is securely mounted with vibration dampening to ensure accurate data collection.
- **Custom Circuit Board**: Designed to facilitate easy integration with peripherals and improve repairability and cost efficiency.

## Software Considerations

- **Serial Communication**: Implemented for wireless receiver, motor controller, and IMU, ensuring seamless data exchange between components.
- **Receiver Signal Loss Protocol**: Developed a protocol to handle loss of signal from the receiver to maintain safety.
- **IMU Data Filtering**: Applied filtering techniques to minimize noise while maintaining low latency for responsive control.
- **PID Feedback Loop**: Optimized for speed and consistency, with special attention to the I term to balance fast response and error accumulation.
- **Polling Architecture**: Chosen to optimize the performance given the single-core limitation of the microcontroller, ensuring consistent loop execution.
- **Motor Arming Safety**: Safety features are implemented to prevent accidental motor arming.

## Flight Software Demonstration

The initial PID feedback loop was overly sensitive to IMU noise, leading to an unstable drone. Additional filtering introduced latency, making the drone unflyable. The solution was to reduce filtering and rely more on the I term in the PID loop, allowing error to build up over time and averaging out random spikes in IMU data.

**Key Points:**
- Less filtering allows faster IMU data processing.
- The I term accumulates error, smoothing out noise over time.
- Careful management of integral windup to prevent loss of control under extreme conditions.

## Custom Circuit Board Features

- **Detachable Microcontroller Socket**: Designed for easy repair and cost efficiency.
- **Peripheral Connections**: Simplified connections for IMU, motor controller, receiver, and Raspberry Pi for computer vision.
- **Vibration Dampening**: Soft-mounted using silicon dampeners to reduce vibrations and enhance stability.

## Flight Software Adjustments

Further adjustments were made to the control logic to maintain full orientational control even during mid-air throttle idling. Additional tweaks were introduced to prevent wobble caused by rapid motor deceleration or acceleration, likely due to slight physical differences between motors. These improvements have enabled more aerobatic movements, similar to those achievable with commercial drone software.

## Video Demonstration

Watch the flight demonstration video [here](https://www.youtube.com/watch?v=OeXxldQP_O4).
