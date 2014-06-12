#ifndef BHV_GOALIE_DANGER_AREA_MOVE_H
#define BHV_GOALIE_DANGER_AREA_MOVE_H

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <rcsc/geom/vector_2d.h>
#include <rcsc/player/soccer_action.h>

#include <boost/shared_ptr.hpp>

namespace rcsc {
class Formation;
}

class Bhv_GoalieOneToOne
    : public rcsc::SoccerBehavior {
public:

    Bhv_GoalieOneToOne()
     { }

    bool execute( rcsc::PlayerAgent * agent );

private:

    rcsc::Vector2D getTargetPoint( rcsc::PlayerAgent * agent );

};

#endif
