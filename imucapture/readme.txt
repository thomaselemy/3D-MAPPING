Ensure that you have cmake installed. Cmake 3.1.3 is the earliest
version that is allowed. To check your version of cmake, run

    cmake --version



How to build:

  In the root directory (where this readme is found), create an
  empty directory of any name. Go into the newly created
  directory and run

      cmake ../

  This command will create a bunch of necessary files in the new
  directory, including a makefile. Once the process finishes,
  run the command

      make

  in the same directory and it will produce an executable file
  named "imucapture". To run "imucapture", run

      sudo ./imucapture

  Sudo is required to provide access to USB devices. If the IMU
  device is detected, it will only output the data to a file
  called "IMU.txt". There will be no output to the console. If
  the device is not detected, it will exit also outputting
  error information to the console.
