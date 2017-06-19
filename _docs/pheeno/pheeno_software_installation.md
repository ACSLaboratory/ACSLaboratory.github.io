---
title: Pheeno Software Installation
category: Pheeno
order: 3
---

## Installation

Currently, Pheeno files can run on all major platforms (Windows, Linux, and OS X) all that is required is cloning the repository to your system.

```bash
$ git clone https://github.com/ACSLab/PheenoRobot.git
```

The following dependencies/programs are required to use these files:
- [OpenCV](http://opencv.org/)
- [Arduino IDE](https://www.arduino.cc/en/Main/Software)
- [pySerial](https://github.com/pyserial/pyserial)
- [picam](https://www.raspberrypi.org/learning/getting-started-with-picamera/)
    - **NOTE:** This should be a default package on the Raspberry Pi.
- [NumPy](http://www.numpy.org/)
    - **NOTE:** The SciPy stack is rather difficult to install on a Windows environment. [This](http://www.lfd.uci.edu/~gohlke/pythonlibs/) guide may help. Make sure to install all the dependencies for each version part of the SciPy stack!
- [Tkinter](http://tkinter.unpythonic.net/wiki/How_to_install_Tkinter)
    - **NOTE:** Tkinter is typically installed by default. If it is not, please refer to the link about for installation instructions.

Currently, Python 2.7+ is the only supported platform. Getting these files to work Python 3 is possible if one was able to install OpenCV on Python 3. Full Python 3 compatibility will be implemented as soon as an OpenCV Python 3 implementation is made stable.

## Getting Started

This section will give a brief introduction to installing the Arduino program to your computer, uploading a script to Pheeno, and installing the pre-made libraries onto your computer.

### Installing Arduino

If you do not have the Arduino software on your computer, first go to the [Arduino software download page](https://www.arduino.cc/en/Main/Software). Download the appropriate software for your operating system and install it!

### Installing the Pheeno Libraries

To use the pre-made libraries for Pheeno you must first download the files and place them in the right place on your computer so the Arduino software can access them. From the [GitHub](https://github.com/ACSLab/PheenoRobot) repository for Pheeno download the Code folder. Inside this folder should be an Arduino subfolder with folders labeled Encoder, LSM303, and Pheeno. Copy those folders to your "Documents/Arduino/libraries" folder. Now open the Arduino software. If all was done correctly, when you go to 'File --> Examples', you should see 'Encoder', 'LSM303', and 'Pheeno' at the bottom (you may have to scroll down if you have a lot of Arduino libraries already)! If this does not work or the above instructions are unclear, it is recommended to look through the [Arduino Library Guide](https://www.arduino.cc/en/Guide/Libraries/).
