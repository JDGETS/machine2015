#define CHECK_FORCE_STOP_MACRO { Vehicle::CheckForceStop(); if(Vehicle::FORCE_STOP) return; }

namespace Vehicle{
  extern bool FORCE_STOP;
  void CheckForceStop();
}