/*
 *Copyright (C) TJNU
 */

#ifndef BHV_ZHMARK_H
#define BHV_ZHMARK_H

#include <rcsc/geom/vector_2d.h>
#include <rcsc/player/soccer_action.h>

class Bhv_ZhMark
    : public rcsc::SoccerBehavior {

private:
    const rcsc::Vector2D HOME_POS;

public:

      Bhv_ZhMark( const rcsc::Vector2D & home_pos )
        : HOME_POS( home_pos )
      { }

      bool execute( rcsc::PlayerAgent * agent );

};

#endif
