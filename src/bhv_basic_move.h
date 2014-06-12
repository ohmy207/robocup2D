// -*-c++-*-

/*
 *Copyright:

 Copyright (C) Hidehisa AKIYAMA

 This code is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 3, or (at your option)
 any later version.

 This code is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this code; see the file COPYING.  If not, write to
 the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.

 *EndCopyright:
 */

/////////////////////////////////////////////////////////////////////

#ifndef BHV_BASIC_MOVE_H
#define BHV_BASIC_MOVE_H

#include <rcsc/geom/vector_2d.h>
#include <rcsc/player/soccer_action.h>

class Bhv_BasicMove
    : public rcsc::SoccerBehavior {
private:
    const rcsc::Vector2D M_home_pos;
    bool isSoftMarking;
    bool amIMarking;
public:
    Bhv_BasicMove()
      { isSoftMarking = false; }

    bool execute( rcsc::PlayerAgent * agent );

private:

    bool crossMarkSideBack( rcsc::PlayerAgent * agent );
    bool crossMarkCenterBack( rcsc::PlayerAgent * agent );
    bool crossMarkDefensiveHalf( rcsc::PlayerAgent * agent );
    bool crossMarkOffensiveHalf( rcsc::PlayerAgent * agent );

    void softMarkSideBack( rcsc::PlayerAgent * agent, rcsc::Vector2D & homePos );
    void softMarkSideBack2( rcsc::PlayerAgent * agent, rcsc::Vector2D & homePos );
    void softMarkCenterBack( rcsc::PlayerAgent * agent, rcsc::Vector2D & homePos );


    bool espBodyTurns( const rcsc::PlayerAgent * agent, rcsc::AngleDeg & bodyAngle );

    double getDashPower( const rcsc::PlayerAgent * agent,
                         const rcsc::Vector2D & target_point );

};

#endif
