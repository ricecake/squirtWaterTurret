# Project: Over-Engineered Sentry Turret Thingamajig

> "I saw a pigeon, and I thought... 'I could automate that.' So I did. You're welcome, world."

## Overview

Welcome to the pinnacle of "because I could" engineering. This project is a sophisticated, and frankly, quite unnecessary, automated turret system. It's designed to track and... *interact*... with targets, using a combination of cutting-edge computer vision and robust, real-time motor control. Think of it as a very, very smart rock.

It was born from a simple question: "What if my cat had a laser pointer, but with, like, *way* more microcontrollers?" The result is this beautiful monstrosity, a testament to what happens when you have too many spare parts and not enough adult supervision.

## How It (Allegedly) Works

The system is a glorious chimera, a two-headed beast of Python and C++.

### The "Brains": The All-Seeing Eye

This is the part that does the thinking. Or at least, a convincing imitation of it. Running on a platform that can handle the heavy lifting (like a PC or a Raspberry Pi), the Python-based computer vision component uses a Luxonis OAK-D camera to see the world.

*   **`depthai`:** This is the magic that lets us tap into the OAK-D's powerful neural processing capabilities. We use it for real-time pose estimation, identifying unsuspecting targets (people, mostly) with alarming accuracy.
*   **`sqlite-vec`:** Because just seeing people isn't enough, we had to give it a memory. This amazing little library allows us to perform vector similarity searches *on-device*. In layman's terms, it can recognize people it's seen before. Whether this is a feature or a threat is up for debate.
*   **Target Serialization:** Once a target is identified and deemed worthy of attention, its coordinates are serialized and sent over to the "Brawn" for... *processing*.

### The "Brawn": The Pointy End

This is where the thinking turns into doing. The firmware, written in C++ and running on an ESP32, is the muscle of the operation. It's a lean, mean, motor-controlling machine.

*   **Dual Stepper Motors:** For precise and surprisingly quiet movement. It can pan and tilt with a grace that belies its crude purpose.
*   **Dual-Mode Operation:** The firmware is a versatile beast. It can take its orders from the high-tech "Brains" over a serial connection, or it can operate independently using a simpler, but effective, **LD2450 Radar Sensor**. So, even if the all-seeing eye is offline, it can still detect movement. It's resourceful like that.
*   **Real-Time, Interrupt-Driven:** It's built to react, and react fast. Because when you're dealing with... *dynamic*... targets, every millisecond counts.

## "Features" (Or Bugs, Depending on Your Perspective)

*   **Autonomous Target Acquisition and Tracking:** It finds things and points at them. All by itself.
*   **Person Recognition:** It remembers faces. And maybe holds a grudge.
*   **Dual-Input System:** Can be driven by complex CV data or a simple radar. It's not picky.
*   **Modular Design:** The "Brains" and "Brawn" are separate, so you can swap out one without (completely) breaking the other.
*   **Vaguely Humorous and Sarcastic Tone:** A key feature, baked right into the documentation.

## The Guts (Technology Stack)

### Hardware
*   **Luxonis OAK-D:** For seeing things.
*   **ESP32:** For doing things.
*   **Dual NEMA-17 Stepper Motors:** For moving things.
*   **LD2450 Radar Sensor:** For when you want to see things, but with radio waves.
*   **A 3D-Printed Mount:** Because duct tape, while effective, is not a permanent solution.

### Software & Libraries
*   **Python:** The language of the "Brains."
*   **C++ (Arduino Framework):** The language of the "Brawn."
*   **`depthai`:** The all-powerful eye in the sky.
*   **`sqlite-vec`:** The elephant's memory.
*   **`AccelStepper` & `MultiStepper`:** For making the motors go brrr.
*   **A custom serialization protocol:** Because why use a standard when you can invent your own?

## A Vague "How to Use" Section

1.  Assemble the hardware. Good luck.
2.  Flash the firmware to the ESP32. You'll probably need the Arduino IDE or PlatformIO.
3.  Run the Python script on a machine connected to the OAK-D.
4.  Connect the "Brains" to the "Brawn" via serial.
5.  Power it all on.
6.  Pray.

## Disclaimer

Look, this is a toy. A very, *very* elaborate toy. It's not a weapon, it's not a security system, and it's definitely not a substitute for a responsible adult. If you build this, you are responsible for what it does. Don't point it at people, pets, or anything you value. The author of this project is not liable for any robot uprisings, interdimensional portals, or startled cats. You have been warned.

Now, go build something ridiculous.