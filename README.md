Installation 
============

This SNRMRRobot controller interface program tested on Ubuntu 14.04.

### 0. update and upgrade the packages
```
sudo apt-get update
sudo apt-get upgrade
```

### 1. gcc and g++:
```
sudo apt-get install build-essential g++
```

### 2. cmake:
```
sudo apt-get install cmake
```
For the GUI interface of cmake:
```
sudo apt-get install cmake-curses-gui
```
### 3. git:
```
sudo apt-get install git
```

### 4. qt4:
```
sudo apt-get install libqt4-dev libqt4-gui libqt4-sql qt4-dev-tools qt4-Designer qt4-qtconfig
```

### 5. OpenGL:
```
sudo apt-get install freeglut3 freeglut3-dev
```

### 6. FoxToolkit
Before you install this library, you have to install X11 first!

```
sudo apt-get install xorg openbox
sudo apt-get install libxft-dev
```

#### (0). Download the source code from the homepage of FoxToolkit, we used fox-1.6.50 here. 
www.fox-toolkit.org/download.html

#### (1). Extract the package
```
tar -xvzf fox-1.6.50.tar.gz
```

#### (2). Configure: 
```
cd fox-1.6.50
sudo ./configure
```

You can disable OpenGL support by configuring as:
```
sudo ./configure --disable-opengl
```

#### (3). Build the library 
```
sudo make
```

#### (4). Install the library into the disk (optional), the default target directory is /usr/local
```
sudo make install
```

### 7. OpenIGTLink library
#### (0). Obtain the source code from the repository using Git
```
git clone https://github.com/openigtlink/OpenIGTLink.git
```

#### (1). Using CMake for configuration, the library requires CMake version higher than 2.4
```
mkdir OpenIGTLink-build
cd OpenIGTLink-build
cmake ../OpenIGTLink
make
```

#### (2). Install the libray into the disk (optional), the default target directory is /usr/local
```
sudo make install
```

### 8. SNRMRRobot controller 
#### (0). Download the source code
```
git clone https://github.com/SNRLab/SNRMRRobotController.git
```

#### (1). Create a new folder for build
```
mkdir SNRMRRobotController-build
```

###### !!! Remember to add the "icon" folder to the build folder

```
cd SNRMRRobotController
cp -r icon ../SNRMRRobotController-build
```

#### (2). Configuration
```
cd ../SNRMRRobotController-build
ccmake ../SNRMRRobotController
```

###### There are several parameters need to change here:
(a). OpenIGTLink_DIR: add the path where you make the OpenIGTLink library (e.g:/home/SNR/OpenIGTLink-build) <br>
(b). USE_ARTAPT:OFF<br>
(c). FOX_INCLUDE_DIR:/usr/local/include/fox-1.6.50<br>
(d). FOX_LINK_DIR:/usr/local/lib<br>

#### (3). Make the executable file
```
make
```

#### (4). Run it
```
./interface
```

### 9. Then if you could see the interface which looks like the figure below

![](https://github.com/ytzhao/RobotController/blob/master/Image/interfce.jpg)

###  :beers: :beers: :beers: Congratulation!!! You've already set up the develop environment for this project
