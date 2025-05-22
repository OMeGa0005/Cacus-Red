# task 1 random number generator

First name: Benjamin
Last name: Redpath
Student #: 48034865

# functionality

The task uses multithreading to generate and display an 8 character randon number through serial
- **rngThread**
    Generates the random number using zephyr rng api, then passes it using a queue
- **displayThread**
    Recieves the random number through a queue and displays it using the printf function

# folder structure 
```
repo
|___mycode
    |___apps
    |   |___s1
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
    |       |       |   build
    |       |
    |       |___task2
    |       |___task3
    |___mylib
```

# references
- [Zephyr RNG Documentation](https://docs.zephyrproject.org/latest/services/crypto/random/index.html)
- [Zephyr Threads Documentation](https://docs.zephyrproject.org/latest/kernel/services/threads/index.html)

# instructions regarding your source
- navigate to repo
- west build -b disco_l475_iot1 mycode/apps/s1/task1 --pristine
- west flash --runner jlink

# user instruction
- monitor terminal in putty