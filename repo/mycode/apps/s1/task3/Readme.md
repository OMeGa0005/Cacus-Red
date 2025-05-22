# task 1 random number generator

First name: Benjamin
Last name: Redpath
Student #: 48034865

# functionality

The task uses multithreading to change the colour of a seeed argb every 2 seconds  
- **cmd_time_get**
    is called from time shell cmd, will return the time in seconds or formated h, m, s if f arg is given, has error handling
- **cmd_led_toggle**
    called from led shell cmd, can toggle or set leds 1 and 2, has error handling 

# folder structure 
```
repo
|___mycode
    |___apps
    |   |___s1
    |       |___task2
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
    |       |       |   build
    |       |
    |       |___task1
    |       |___task3
    |___mylib
```

# references
- [Provided rgb example on git](https://github.com/Seeed-Studio/Grove_Chainable_RGB_LED/blob/master/ChainableLED.cpp#L109)
- [Zephyr Threads Documentation](https://docs.zephyrproject.org/latest/kernel/services/threads/index.html)
- [Seeed led datasheet](https://files.seeedstudio.com/wiki/Grove-Chainable_RGB_LED/res/P9813_datasheet.pdf)
- [STM32 disco datasheet](https://www.st.com/resource/en/user_manual/um2153-discovery-kit-for-iot-node-multichannel-communication-with-stm32l4-stmicroelectronics.pdf)

# instructions regarding your source
- navigate to repo
- west build -b disco_l475_iot1 mycode/apps/s1/task1 --pristine
- west flash --runner jlink

# user instruction
- monitor terminal in putty
- sample commands, (time, time f, led s 01, led t 11)