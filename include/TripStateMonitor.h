/*
 * TripStateMonitor.h
 * This object simply keeps track of the trip state and publishes it to
 * anything that needs to know the trip state.
 * Copyright (C) 10.9.2017 Gabriel Meyer-Lee
 *
 * Distributed under terms of the MIT license.
 */

#ifndef TRIPSTATEMONITOR_H
#define TRIPSTATEMONITOR_H

#include <vector>

#include "TripStateObs.h"

class TripStateMonitor {
  public:
    /**
     * Inputs: none
     * Output: none
     * Constructor
     */
    TripStateMonitor();

    /**
     * Inputs: none
     * Output: none
     * Destructor
     */
    virtual ~TripStateMonitor();

    /**
     * Inputs: int tripState detailing what's caused the trip
     * Output: none
     * Tripping this object simply causes it to trip every object which
     * needs to know the trip state.
     */
    void trip( int tripState );

    /**
     * Inputs: Pointer to a Trip State Observer
     * Outputs: none
     * Adds the given obervers to the list of things that need to know the
     * trip state
     */
    void attach( TripStateObs* obs );

    /**
     * Inputs: none
     * Output: none
     * Untripping this object simply causes it to untrip every object which
     * needs to know the trip state.
     */
    void untrip();

  private:

    int tripState; // the current trip state
    std::vector<TripStateObs*> observers; // things that need to know trip state

    /**
     * Inputs: none
     * Output: none
     * Notifies observers of the new trip state
     */
    void notify();
    /**
     * Inputs: none
     * Output: none
     * Notifies observers that the program has been untripped
     */
    void notify_u();
};

#endif /* !TRIPSTATEMONITOR_H */
