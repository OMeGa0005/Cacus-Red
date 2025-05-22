# task 1 random number generator

First name: Benjamin
Last name: Redpath
Student #: 48034865

# functionality

Will send the ultrasonic distances over ble

- **ble_thread**
    recieves ble

- **display_thread**
    displays location on display

# folder structure 
```
repo
|___mycode
    |___apps
    |   |___prac3
    |       |___disco
    |       |   |   CMakeLists.txt
    |       |   |   prj.conf
    |       |   |   README.md
    |       |   |   dtc_shell.overlay
    |       |   |
    |       |   |___boards
    |       |   |   |   disco_l475_iot.overlay
    |       |   |
    |       |   |___src
    |       |       |   main.c
    |       |       |   ble.c
    |       |       |   ble.h
    |       |       |   ultrasonic.c
    |       |       |   ultrasonic.h
    |       |       |   build
    |       |___m5
    |       |   |   CMakeLists.txt
    |       |   |   prj.conf
    |       |   |   README.md
    |       |   |   dtc_shell.overlay
    |       |   |
    |       |   |___boards
    |       |   |   |   disco_l475_iot.overlay
    |       |   |
    |       |   |___src
    |       |       |   main.c
    |       |       |   display.c
    |       |       |   display.h
    |       |       |   ble.c
    |       |       |   ble.h
    |       |       |   build
    |___mylib
```

# references
- [Provided rgb example on git](https://github.com/Seeed-Studio/Grove_Chainable_RGB_LED/blob/master/ChainableLED.cpp#L109)
- [Zephyr Threads Documentation](https://docs.zephyrproject.org/latest/kernel/services/threads/index.html)
- [sensors](https://docs.zephyrproject.org/latest/hardware/peripherals/sensor/index.html)
- [filesystem](https://docs.zephyrproject.org/latest/services/file_system/index.html)
- [STM32 disco datasheet](https://www.st.com/resource/en/user_manual/um2153-discovery-kit-for-iot-node-multichannel-communication-with-stm32l4-stmicroelectronics.pdf)

# instructions regarding your source
- navigate to repo
- west build -b m5stack_core2/esp32/procpu mycode/apps/prac3/m5 --pristine
- west flash

# user instruction
- monitor terminals in putty