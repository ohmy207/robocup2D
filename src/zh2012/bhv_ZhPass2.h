/*
 *Copyright (C) TJNU
 */

#ifndef BHV_ZHPASS2_H
#define BHV_ZHPASS2_H

#include <rcsc/player/soccer_action.h>
#include <rcsc/player/player_agent.h>

#include <rcsc/geom/vector_2d.h>

class Bhv_ZhPass2
    : public rcsc::BodyAction {

private:

   rcsc::Vector2D opp[20];
   rcsc::Vector2D tmm[20];
   rcsc::AngleDeg tAngle[20];

   int receiver[20];
   int oppNo[20];

   int oCount[20];
   int tCount[20];

   const rcsc::PlayerType * oppPlayerType[20];
   const rcsc::PlayerType * tmmPlayerType[20];

   int oMax;
   int tMax;
    int i;

public:

    Bhv_ZhPass2();

    ~Bhv_ZhPass2();

    int sign ( float n );
    rcsc::Vector2D getSortedTmm ( rcsc::PlayerAgent * agent, rcsc::Vector2D point, int rank , int &num, int maxCount);
    int cycles( rcsc::PlayerAgent * agent, rcsc::Vector2D target, float speed, int i, rcsc::Vector2D o[], int x );
    int crossCycles( rcsc::PlayerAgent * agent, rcsc::Vector2D target, float speed, rcsc::Vector2D o[] );
    int leadingCycles( rcsc::PlayerAgent * agent, rcsc::Vector2D target, float speed, int tmmNum );
    int throughCycles( rcsc::PlayerAgent * agent, rcsc::Vector2D target, float speed, rcsc::Vector2D tmmPos );
    int selfCyclesSRP( rcsc::PlayerAgent * agent, rcsc::Vector2D target, float speed );

    //具体传球
    bool canPassToNo ( rcsc::PlayerAgent * agent, int no );
    bool DirectPass( rcsc::PlayerAgent * agent, rcsc::Vector2D & target );
    bool LeadingPass( rcsc::PlayerAgent * agent, rcsc::Vector2D & target );
    bool LeadingPass2011( rcsc::PlayerAgent * agent, rcsc::Vector2D & targetzz );
    bool ThroughPass( rcsc::PlayerAgent * agent, rcsc::Vector2D & target );
    bool PowerfulThroughPass( rcsc::PlayerAgent * agent, rcsc::Vector2D & target );
    bool PowerfulThroughPass2011( rcsc::PlayerAgent * agent, rcsc::Vector2D & target );
    bool CrossPass( rcsc::PlayerAgent * agent, rcsc::Vector2D & target );
    bool BrainCross2( rcsc::PlayerAgent * agent, rcsc::Vector2D & target );
    bool SRPD( rcsc::PlayerAgent * agent );
    bool SRPtoCenter( rcsc::PlayerAgent * agent );
    bool SRPtoOutside( rcsc::PlayerAgent * agent );
    bool DKhBClear( rcsc::PlayerAgent * agent );

    bool execute( rcsc::PlayerAgent * agent );
};
#endif
