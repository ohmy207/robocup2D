 // -*-c++-*-

/*
 *Copyright (C) TJNU
 */

/////////////////////////////////////////////////////////////////////

#ifndef BHV_BASIC_DEFENSIVE_KICK_H
#define BHV_BASIC_DEFENSIVE_KICK_H

#include <rcsc/player/soccer_action.h>

using namespace rcsc;

class Bhv_BasicDefensiveKick
      : rcsc::SoccerBehavior {

public:
    Bhv_BasicDefensiveKick()
    {
    }

    bool execute( rcsc::PlayerAgent * agent );

};

#endif
