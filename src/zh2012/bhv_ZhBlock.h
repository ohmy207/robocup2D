/*
 *Copyright (C) TJNU
 */

#ifndef BHV_ZHBLOCK_H
#define BHV_ZHBLOCK_H

#include <rcsc/player/soccer_action.h>
#include <rcsc/geom/vector_2d.h>

#include <vector>

namespace rcsc {
class WorldModel;
}

class Bhv_ZhBlock
    : public rcsc::SoccerBehavior {
private:

public:  

  static bool isInBlockPoint;
  static int timeAtBlockPoint;

    bool execute( rcsc::PlayerAgent * agent );
    bool execute2010( rcsc::PlayerAgent * agent );

private:


    bool doInterceptBall( rcsc::PlayerAgent * agent );
    bool doInterceptBall2011( rcsc::PlayerAgent * agent );

    bool doBlockMove( rcsc::PlayerAgent * agent );
    bool doBlockMove2011( rcsc::PlayerAgent * agent );

    rcsc::Vector2D getBlockPoint( rcsc::PlayerAgent * agent );
    rcsc::Vector2D getBlockPoint2011( rcsc::PlayerAgent * agent );
    rcsc::Vector2D getBlockPoint2011_backUp( rcsc::PlayerAgent * agent );

    double getBlockDashPower( const rcsc::PlayerAgent * agent,
                              const rcsc::Vector2D & blockPos );
};
#endif
