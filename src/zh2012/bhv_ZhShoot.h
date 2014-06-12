/*
 *Copyright (C) TJNU
 */

#ifndef BHV_ZHSHOOT_H
#define BHV_ZHSHOOT_H

#include <rcsc/player/soccer_action.h>

#include <rcsc/geom/vector_2d.h>


class Bhv_ZhShoot
    : public rcsc::BodyAction {

private:

    bool canScore( rcsc::PlayerAgent * agent, const rcsc::Vector2D & shootTarget,
                   const double one_step_speed );

public:

    Bhv_ZhShoot()
      { }


    bool execute( rcsc::PlayerAgent * agent );

};


#endif
