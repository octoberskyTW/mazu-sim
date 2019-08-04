# mazu-sim
Mazu simulation(mazu-sim) is 6DoF Rocket Simulation Platform.

It is ported from [Modeling INS/GPS/Star-Tracker in 6 DoF: Simulating NGC of a Three-Stage Rocket Booster in CADAC++](https://www.amazon.com/Modeling-INS-GPS-Star-Tracker-DoF/dp/1518899315)

The aim is to provide to verify flight software, and delivery accuracy hardware-in-the-loop and flight testing.

# Getting Started
## 0. Clone the Repo and sub repo
```
$ git clone https://github.com/octoberskyTW/mazu-sim
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
   - [Official Github Armadillo](http://arma.sourceforge.net/)
   ```
   $ ./configure
   $ make
   $ sudo make install
   ``` 

## 3. Build and Run a sample code
```
$ make
$ make run-sample_code
```
