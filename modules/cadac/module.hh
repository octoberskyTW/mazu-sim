#ifndef __MODULE_HH__
#define __MODULE_HH__

#include <armadillo>
#include "vehicle.hh"
#include "matrix_tool.hh"

class Sensor : public FH_module {
 public:
  char name[32];
  // Preserve for output I/O to FC
};

class Actuator : public FH_module {
 public:
  char name[32];
  // Preserve for recieving input data from FC
};

class FC_Algorithm : public FH_module {
 public:
  char name[32];
  // Preserve for output I/O to FC
};

class Dynamics : public FH_module {
 public:
  char name[32];
  // Preserve for output I/O to test instrument such like SimGen,
  // Ratetable...etc
};

#endif  // __MODULE_HH__