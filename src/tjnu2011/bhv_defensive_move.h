// -*-c++-*-

/*
 *Copyright (C) TJNU
 */

/////////////////////////////////////////////////////////////////////

#ifndef ROBOSAMPAD_BHV_DEFENSIVE_MOVE_H
#define ROBOSAMPAD_BHV_DEFENSIVE_MOVE_H


#include <rcsc/geom/vector_2d.h>
#include <rcsc/player/soccer_action.h>
#include "WorldModel.h"

class Bhv_DefensiveMove
    : public rcsc::SoccerBehavior {
private:
    const rcsc::Vector2D M_home_pos;
    const bool M_turn_at;
    const bool M_turn_neck;
    WorldModelHighLevel WM;

public:
    Bhv_DefensiveMove( const rcsc::Vector2D & home_pos,
                   const bool turn_at = true,
                   const bool turn_neck = true )
        : M_home_pos( home_pos )
        , M_turn_at( turn_at )
        , M_turn_neck( turn_neck )
      {

      }

    bool execute( rcsc::PlayerAgent * agent );

private:
    double getDashPower( const rcsc::PlayerAgent * agent,
                         const rcsc::Vector2D & target_point );

};

#endif
