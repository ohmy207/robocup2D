// -*-c++-*-

/*
 *Copyright (C) TJNU
 */

/////////////////////////////////////////////////////////////////////

#ifndef BEST_SHOOT_POINT_H
#define BEST_SHOOT_POINT_H

#include "WorldModel.h"
#include <rcsc/player/soccer_action.h>

class Best_ShootPoint{

public :
    Best_ShootPoint()
    {

    }

    Vector2D Point( PlayerAgent * agent , int & point );

};

#endif
