# FastNonAccelStepper

A non-blocking stepper motor control library for the ESP32 using MCPWM. This library allows for precise control of stepper motors by utilizing the hardware MCPWM module on the ESP32, which offloads the work from the CPU for smoother, higher-frequency operation.

The library is inspired by the awesome [FastAccelStepper](https://github.com/gin66/FastAccelStepper) library. 
The FastAccelStepper is perfectly suited for applications which require smooth position/velocity trajectories. 
For my application, see [DIY-Sim-Racing-FFB-Pedal](https://github.com/ChrGri/DIY-Sim-Racing-FFB-Pedal), acceleration patterns aren't needed, as the trajectories are intriniscly smoothed by the servo control-loop. 
Instead, the focus was to reduce the libraries complexity by removing the trajectory/ramp planner and focus on faster update intervals for super-fast control loops. 
In other words, the library will tell the servo its target position via the pulse/dir interface as fast as the servos interface will allow. 
For the [iSV57 servo (affiliate link)](https://www.omc-stepperonline.com/de/nema-23-integrierter-easy-servo-motor-130w-3000rpm-0-45nm-63-73oz-in-20-50vdc-buerstenloser-dc-servomotor-kurze-welle-isv57t-130s?tracking=6721c5865911c) for instance, the pulse/dir interface allows pulses of up to 300kHz.


## Features

- Non-blocking stepper motor control on ESP32
- Uses MCPWM hardware for high-frequency pulse generation
- Provides functions to set target position, speed, and current position
- Efficient CPU usage for real-time applications

## Installation

### Using Arduino IDE
1) Download the library
2) Add library to Arduino IDE

### Using PlatformIO
In Visual Studio Code, add the following to your `platformio.ini` file under `lib_deps`:

   ```ini
   lib_deps =
       https://github.com/ChrGri/FastNonAccelStepper.git
   ```


## Example usage
Please refer to [examples](https://github.com/ChrGri/FastNonAccelStepper/blob/main/examples/simple/simple.ino) for basic usage.

## Support 

If you like this library, feel free to support via <br>
<a href="https://www.buymeacoffee.com/Captainchris"><img src="https://www.buymeacoffee.com/assets/img/custom_images/orange_img.png" height="20px"></a> <br>
[![ko-fi](https://ko-fi.com/img/githubbutton_sm.svg)](https://ko-fi.com/captainchris88)

Thank you :heart:


