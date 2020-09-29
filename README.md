<p align=center>
<img src="docs/Mazu-sim_logo.jpg" alt="Mazu Logo" height=400px>
</p>

# mazu-sim

Mazu simulation(mazu-sim) is 6DoF Rocket Simulation Platform.

It base on [Modeling INS/GPS/Star-Tracker in 6 DoF: Simulating NGC of a Three-Stage Rocket Booster in CADAC++](https://www.amazon.com/Modeling-INS-GPS-Star-Tracker-DoF/dp/1518899315)

The aim is to provide to verify flight software, and delivery accuracy hardware-in-the-loop and flight testing.

## Getting Started
### 0. Clone the Repo
```
$ git clone --recursive https://github.com/octoberskyTW/mazu-sim
```
### 1. Dependencies
 - GCC 5.4 up
 - Python 2.7
 - cpplint
```
$ sudo apt-get install htop cmake vim tree git-core libopenblas-dev libboost-dev libboost-all-dev bison curl flex python-pip clang-format cppcheck
$ pip install cpplint
```

### 2. Third-Party Installation
 - Armadillo Matrix Library-9.700.2
   - [Official Armadillo](http://arma.sourceforge.net/)
   ```
   third-party/armadillo $ tar xvf armadillo-9.700.2.tar.xz
   third-party/armadillo $ cd armadillo-9.700.2
   third-party/armadillo/armadillo-9.700.2 $ ./configure
   third-party/armadillo/armadillo-9.700.2 $ make
   third-party/armadillo/armadillo-9.700.2 $ sudo make install
   ``` 
- Google Test
  - Official Github [google test](https://github.com/google/googletest)
  ```bash
  $ sudo apt-get install libgtest-dev
  $ sudo apt-get install cmake # install cmake
  $ cd /usr/src/gtest
  $ sudo mkdir build && cd build
  $ sudo cmake .. && sudo make
  # copy or symlink libgtest.a and libgtest_main.a to your /usr/local/lib folder
  $ sudo cp *.a /usr/local/lib
  ```
 - NASA Trick 17.5 dev
   - Official Github [NASA/trick](https://github.com/nasa/trick)
   ```
   $ sudo apt-get install bison curl flex g++ libx11-dev libxml2-dev libxt-dev \
            libmotif-common libmotif-dev make openjdk-8-jdk python2.7-dev swig \
            zlib1g-dev llvm llvm-dev clang libclang-dev libudunits2-dev
   third-party/trick $ ./configure
   third-party/trick $ make
   third-party/trick $ sudo make install
   ```
- Redis-Server
  - Download Source: https://redis.io/download
  ```bash
    third-party/redis $ tar xzf redis-5.0.5.tar.gz
    third-party/redis $ cd redis-5.0.5
    third-party/redis/redis-5.0.5 $ make
    third-party/redis/redis-5.0.5 $ sudo make install
  ```
### 3. Build and Run a sample code
```
$ make
$ make run-sample
```

## Build and run the specific project
- Tutorial project: sample
  ```
  $ make project=sample
  $ make run-sample
  ```
- Rocket project: skyline
  ```
  $ make project=skyline
  $ make run-skyline
  ```
## Doxygen
```
$ sudo apt-get install graphviz doxygen
$ doxygen Doxyfile
```
open the docs/build/index.html in browser

<p align=center>
<img src="docs/mazu-sim_software_stack.png" alt="Software Stack" height=400px>
</p>

# Companies/Organization using mazu-sim
<p align=center>
<img src="https://scontent.ftpe8-1.fna.fbcdn.net/v/t31.0-8/1796904_588326357979241_7725115450624607776_o.png?_nc_cat=105&_nc_sid=09cbfe&_nc_ohc=Fu1zcfz43HoAX-mgu6V&_nc_ht=scontent.ftpe8-1.fna&oh=5a9de391e58a5dfa10c971abcae1957c&oe=5F7D22DE" height=200px>
</p>
- [Linux for Rocket Flight Control](https://elinux.org/images/1/1e/ELCE-2019-RT-Linux-for-Rocket-Flight-Control-Kang-Huang.pdf
