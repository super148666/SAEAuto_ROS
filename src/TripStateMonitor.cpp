/**
 * Subject interface for Trip State
 * Gabriel Meyer-Lee
 * 10/9/2017
 */

#include "TripStateMonitor.h"

TripStateMonitor::TripStateMonitor() {
  tripState = 0;
}

TripStateMonitor::~TripStateMonitor() {}

void TripStateMonitor::attach(TripStateObs* obs) {
  observers.push_back(obs);
}

void TripStateMonitor::notify() {
  for(int i = 0; i < observers.size(); i++) {
    observers[i]->trip(tripState);
  }
}

void TripStateMonitor::notify_u() {
  for(int i = 0; i < observers.size(); i++) {
    observers[i]->untrip();
  }
}

void TripStateMonitor::trip(int tripState) {
  this->tripState = tripState;
  notify();
}

void TripStateMonitor::untrip() {
  this->tripState = 0;
  notify_u();
}
