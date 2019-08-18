<p align=center>
<img src="https://raw.githubusercontent.com/octoberskyTW/mazu-sim/master/Mazu-sim_logo.jpg" alt="Mazu Logo" height=400px>
</p>

# mazu-sim
Mazu simulation(mazu-sim) is 6DoF Rocket Simulation Platform.

It base on [Modeling INS/GPS/Star-Tracker in 6 DoF: Simulating NGC of a Three-Stage Rocket Booster in CADAC++](https://www.amazon.com/Modeling-INS-GPS-Star-Tracker-DoF/dp/1518899315)

The aim is to provide to verify flight software, and delivery accuracy hardware-in-the-loop and flight testing.

# Getting Started
## 0. Clone the Repo
```
$ git clone --recursive https://github.com/octoberskyTW/mazu-sim
```
## 1. Dependencies
 - GCC 5.4 up
 - Python 2.7
 - cpplint
```
$ sudo apt-get install htop cmake vim tree git-core libopenblas-dev libboost-dev libboost-all-dev bison curl flex python-pip
$ pip install cpplint
```

## 2. Third-Party Installation
 - NASA Trick 17.0.5
   - Official Github [NASA/trick](https://github.com/nasa/trick)
   ```
   $ sudo apt-get install bison curl flex g++ libx11-dev libxml2-dev libxt-dev \
            libmotif-common libmotif-dev make openjdk-8-jdk python2.7-dev swig \
            zlib1g-dev llvm llvm-dev clang libclang-dev libudunits2-dev
   $ ./configure
   $ make
   $ sudo make install
   ```
 - Armadillo Matrix Library-9.100.6
   - [Official Armadillo](http://arma.sourceforge.net/)
   ```
   $ ./configure
   $ make
   $ sudo make install
   ``` 
- Google Test
  - Official Github [google test](https://github.com/google/googletest)
  ```bash
  sudo apt-get install libgtest-dev
  sudo apt-get install cmake # install cmake
  cd /usr/src/gtest
  sudo cmake CMakeLists.txt
  sudo make
  # copy or symlink libgtest.a and libgtest_main.a to your /usr/lib folder
  sudo cp *.a /usr/lib
  ```
- Redis-Server
  - Download Source: https://redis.io/download
  ```bash
    $ wget http://download.redis.io/releases/redis-5.0.5.tar.gz
    $ tar xzf redis-5.0.5.tar.gz
    $ cd redis-5.0.5
    $ make
    $ sudo make install
  ```
## 3. Build and Run a sample code
```
$ make
$ make run-sample_code
```

# Build and run the specific project
- Tutorial project: sample_code
  ```
  $ make project=sample_code
  $ make run-sample_code
  ```
- ARRC Rocket project: egse_dm
  ```
  $ make project=egse_dm
  $ make run-egse_dm
  ```
# Doxygen
```
sudo apt-get install graphviz doxygen
doxygen Doxyfile
```
open the docs/index.html in browser
