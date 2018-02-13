/*
 * TripStateObs.h
 * Abstract Class for anything that needs to know the trip state.
 * Copyright (C) 10.9.2017 Gabriel Meyer-Lee
 *
 * Distributed under terms of the MIT license.
 */

#ifndef TRIPSTATEOBS_H
#define TRIPSTATEOBS_H


class TripStateObs {
  public:
    /**
     * Input: integer specifying what has caused the trip.
     * Output: none.
     * Does whatever the implementing class does when tripped.
     */
    virtual void trip( int tripState ) { };
    /**
     * Input: none
     * Output: none
     * Does whatever the implementing class does when untripped.
     */
    virtual void untrip() { };
};

#endif /* !TRIPSTATEOBS_H */
