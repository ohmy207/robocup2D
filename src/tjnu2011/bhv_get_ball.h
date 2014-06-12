// -*-c++-*-

/*
 *Copyright:

 Copyright (C) Hidehisa AKIYAMA

 This code is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2, or (at your option)
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

// -*-c++-*-

/*
 *Modified copyright (C) TJNU
 */

/////////////////////////////////////////////////////////////////////

#ifndef BHV_GET_BALL_H
#define BHV_GET_BALL_H

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <rcsc/player/soccer_action.h>
#include <rcsc/geom/vector_2d.h>
#include <rcsc/geom/rect_2d.h>

#include <rcsc/soccer_math.h>
#include <rcsc/timer.h>
#include <rcsc/common/logger.h>
#include <rcsc/common/server_param.h>

#include <rcsc/player/debug_client.h>
#include <rcsc/player/intercept_table.h>
#include <rcsc/player/player_agent.h>

#include <rcsc/action/basic_actions.h>
#include <rcsc/action/body_go_to_point.h>
#include <rcsc/action/neck_turn_to_ball_or_scan.h>

#include "bhv_basic_tackle.h"
#include "bhv_danger_area_tackle.h"
#include <rcsc/action/body_intercept2009.h>
#include "yu_modify/neck_check_ball_owner.h"

namespace rcsc {
class WorldModel;
}

class Bhv_GetBall
    : public rcsc::SoccerBehavior {
private:

    rcsc::Rect2D M_bounding_rect;

public:

    struct Param {
        rcsc::Vector2D point_; //!< target point
        int cycle_; //!< estimated reach steps
        double stamina_; //!< left stamina

        Param()
            : point_( rcsc::Vector2D::INVALIDATED )
            , cycle_( 1000 )
            , stamina_( 0.0 )
          { }
    };

    explicit
    Bhv_GetBall( const rcsc::Rect2D & bounding_rect )
        : M_bounding_rect( bounding_rect )
      { }

    bool execute( rcsc::PlayerAgent * agent );


    static
    void simulate( const rcsc::WorldModel & wm,
                   const rcsc::Vector2D & center_pos,
                   const rcsc::Rect2D & bounding_rect,
                   const double & dist_thr,
                   const bool save_recovery,
                   Param * param );

    static
    int predictSelfReachCycle( const rcsc::WorldModel & wm,
                               const rcsc::Vector2D & target_point,
                               const double & dist_thr,
                               const bool save_recovery,
                               double * stamina );

};

#endif
