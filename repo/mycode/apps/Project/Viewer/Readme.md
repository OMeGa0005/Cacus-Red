# task 1 random number generator

First name: Benjamin
Last name: Redpath
Student #: 48034865

# functionality

Keeps track of real time using rtc,
    can set rtc
    can read from rtc
Can read data from sensors,
    temp 
    humidity
    pressure
    magnetism
Can display data in multiple ways
    on call
    sampled
    output to file
    graphically sampled

- **samplethread**
    Will sample the data currently being sampled and then dispaly it as json

- **cmd**
    can take commands to execute actions

# folder structure 
```
repo
|___mycode
    |___apps
    |   |___prac2
    |       |___task1
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
    |       |       |   task1.c
    |       |       |   task2.c
    |       |       |   task3.c
    |       |       |   task5.c
    |       |       |   task7.py
    |       |       |   task1.h
    |       |       |   task2.h
    |       |       |   task3.h
    |       |       |   task5.h
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
- west build -b disco_l475_iot1 mycode/apps/s1/task1 --pristine
- west flash --runner jlink

# user instruction
- monitor terminal in putty
- view graphics in python