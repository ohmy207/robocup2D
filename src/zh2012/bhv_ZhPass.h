/*
 *Copyright (C) TJNU
 */

#ifndef BHV_ZHPASS_H
#define BHV_ZHPASS_H

#include <rcsc/player/soccer_action.h>

#include <rcsc/geom/vector_2d.h>



class Bhv_ZhPass
    : public rcsc::BodyAction {

private:

   rcsc::Vector2D opp[20];
   rcsc::Vector2D tmm[20];
   rcsc::AngleDeg tAngle[20];

   int receiver[20];

   int oCount[20];
   int tCount[20];

   int oMax;
   int tMax;

public:


    Bhv_ZhPass();

    ~Bhv_ZhPass();

    int sign ( float n );
    int cycles( rcsc::PlayerAgent * agent, rcsc::Vector2D target, float speed, int i, rcsc::Vector2D o[] );
    int throughCycles( rcsc::PlayerAgent * agent, rcsc::Vector2D target, float speed, int i, rcsc::Vector2D o[] );

    int cycles2( rcsc::PlayerAgent * agent, rcsc::Vector2D target, float speed, int i, rcsc::Vector2D o[], float *x, float *y);
    rcsc::Vector2D finalPoint( rcsc::PlayerAgent * agent, float dir, float speed);

    bool DirectPass( rcsc::PlayerAgent * agent, rcsc::Vector2D & target );
    bool LeadingPass( rcsc::PlayerAgent * agent, rcsc::Vector2D & target );
    bool ThroughPass( rcsc::PlayerAgent * agent, rcsc::Vector2D & target );
    bool CrossPass( rcsc::PlayerAgent * agent, rcsc::Vector2D & target );
    bool BrainCross( rcsc::PlayerAgent * agent, rcsc::Vector2D & target );
    bool BrainCross2( rcsc::PlayerAgent * agent, rcsc::Vector2D & target );
    bool SRPD( rcsc::PlayerAgent * agent );

    bool execute( rcsc::PlayerAgent * agent );

};


#endif
