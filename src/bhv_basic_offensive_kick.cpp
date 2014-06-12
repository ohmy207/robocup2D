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

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

//你妹的头文件
#include "bhv_basic_offensive_kick.h"
#include "zh2012/bhv_ZhPass.h"
#include "zh2012/bhv_ZhPass2.h"
#include <rcsc/action/body_advance_ball.h>
#include <rcsc/action/body_dribble.h>
#include <rcsc/action/body_hold_ball.h>
#include <rcsc/action/body_kick_one_step.h>
#include <rcsc/action/body_clear_ball.h>
#include <rcsc/action/body_pass.h>
#include <rcsc/action/neck_scan_field.h>
#include <rcsc/action/neck_turn_to_low_conf_teammate.h>

#include <rcsc/player/player_agent.h>
#include <rcsc/player/debug_client.h>

#include <rcsc/common/logger.h>
#include <rcsc/common/server_param.h>
#include <rcsc/geom/sector_2d.h>

using namespace rcsc;

/*-------------------------------------------------------------------*/
/*!

 */
bool
Bhv_BasicOffensiveKick::execute( PlayerAgent * agent )
{
    dlog.addText( Logger::TEAM,
                        __FILE__": Bhv_BasicOffensiveKick" );

    const WorldModel & wm = agent->world();

    Vector2D ball = wm.ball().pos();
    Vector2D me = wm.self().pos();

    int num = wm.self().unum();

    const PlayerPtrCont & opps = wm.opponentsFromSelf();
    const PlayerObject * nearest_opp
        = ( opps.empty()
            ? static_cast< PlayerObject * >( 0 )
            : opps.front() );
    const double nearest_opp_dist = ( nearest_opp
                                      ? nearest_opp->distFromSelf()
                                      : 1000.0 );
    const Vector2D nearest_opp_pos = ( nearest_opp
                                             ? nearest_opp->pos()
                                             : Vector2D( -1000.0, 0.0 ) );

    Vector2D pass_point;



   bool shouldAvoid = true;


      if( ball.x < 36.0 && ball.x > -36.0 )
           shouldAvoid = true;

      if( ball.x > 36.0 && ball.absY() < 15.0 )
           shouldAvoid = false;


    float minDist = 9.0;

      if( ball.x > 30.0 )
           minDist = 8.0;

      if( ball.x > 36.0 && ball.absY() < 20.0 )
           minDist = 7.0;

      if( ball.x > 36.0 && ball.absY() < 10.0 )
           minDist = 6.0;

      if( (num == 9 || num == 10) && ball.absY() > 25.0 )
           minDist = 6.0;
      
      if( num > 6 && ball.absY() < 12.0 )
	   minDist = 4.5;

    Vector2D drib_target( 51.0, wm.self().pos().absY() );
    if( drib_target.y < 20.0 ) drib_target.y = 20.0;
    if( drib_target.y > 32.0 ) drib_target.y = 31.0;
    if( wm.self().pos().y < 0.0 ) drib_target.y *= -1.0;


      if( (num == 7 || num == 8) && ball.x < 25.0 && ball.x > wm.offsideLineX() - 8.0 )
            drib_target = Vector2D( me.x + 10.0, 22.0 );

      if( num == 6 && ball.x > 28.0 )
            drib_target = Vector2D( 40.0, 0.0 );

      if( ball.x > 25.0 && ball.x < 33.0 )
            drib_target.y = me.y;

      if( num > 9 && me.absY() > 25.0 )
            drib_target = Vector2D( me.x + 13.0, 31.2 );



    Vector2D goal = Vector2D(51.0, 0.0);


     if( wm.self().unum() == 11 && (ball.x > 25.0 || ball.x > wm.offsideLineX() - 10.0) )
         goal = Vector2D(47.0, 0.0);

     if( me.x > 51.0 )
         goal.x -= 3.0;
     if( wm.self().unum() == 11 && ball.x > wm.offsideLineX() - 10.0 &&
         wm.countOpponentsIn(Circle2D(Vector2D(me.x+5.0,me.y) ,5.0),
                             3,false) > 1 && ball.x < 30.0 )
         drib_target = me + Vector2D::polar2vector( 20.0,
                       (Vector2D(me.x + 4.0,me.y+sign(me.y)*12.0)-me).dir() );


     if( ball.x > 33.0 )
            drib_target = me + Vector2D::polar2vector( 20.0, (goal-me).dir() );

{
  
  int oMax = 0;
  Vector2D opp[11];
  const PlayerPtrCont::const_iterator o_end = wm.opponentsFromSelf().end();
  int i = 0;
  for ( PlayerPtrCont::const_iterator o = wm.opponentsFromSelf().begin();
        o != o_end;
        ++o )
  {

      if( (*o)->posCount() > 20 )
         continue;

      if( (*o)->pos().dist( ball ) > 15 )
        continue;

      opp[i] = (*o)->pos() + (*o)->vel();

      i++;
  }
  oMax = i;

if (oMax > 0)
{
  double modifier = 0.0;
  for (int i=0; i <oMax; i++)
  {
    double z = (opp[i]-ball).dir().degree() - (drib_target-ball).dir().degree();
    int manfi = 1;
    if (z < 0)
    {
      manfi = -1;
      z *= -1;
    }
    double d = opp[i].dist(ball);

    if( (wm.self().unum() < 9 && ball.x < -15.0) ||
        (wm.self().unum() < 6 && ball.x < -36.0) )
       d = pow (d/23 , 2) * 15;
    else
       d = pow (d/23 , 2) * 30;

    d = atan(d);
    d = d * 180.0 / 3.14;
    d = 90.0 - d;
    d *= 2.0;
    z = z * 3.14 / 180.0;
    z = 1.0 - sin (z/2.0);
    z*=-1.0;
    modifier += (z*d*manfi);
  }

  int finalDir = (drib_target-me).dir().degree() + (modifier / oMax);


     if( finalDir < 22.5 && finalDir >= -22.5 )
          finalDir = 0.0;
     else if( finalDir < 67.5 && finalDir >= 22.5 )
          finalDir = 45.0;
     else if( finalDir < 112.5 && finalDir >= 67.5 )
          finalDir = 90.0;
     else if( finalDir < 157.5 && finalDir >= 112.5 )
          finalDir = 135.0;
     else if( finalDir < -157.5 || finalDir >= 157.5 )
          finalDir = 180.0;
     else if( finalDir > -67.5 && finalDir <= -22.5 )
          finalDir = -45.0;
     else if( finalDir > -112.5 && finalDir <= -67.5 )
          finalDir = -90.0;
     else if( finalDir > -157.5 && finalDir <= -112.5 )
          finalDir = -120.0;
     else
     {
          std::cout<<"\nCycle: "<<wm.time().cycle()<<" Dribble Dir Not Found~> Dribble Dir => 0.0 "<<"\n";
          finalDir = 0.0;
     }


  if ( ((drib_target-ball).dir().degree() <= 0 && ball.y <31) ||
       ((drib_target-ball).dir().degree() >= 0 && ball.y >-31) )
    drib_target = me +
         Vector2D::polar2vector( 20.0, finalDir );
}
}

    if( ball.x > 36.0 && ball.absY() < 15.0 )
       drib_target = rcsc::Vector2D(55.0,0);
    
    if( ball.x > 47.0 && ball.absY() < 15.0 )
       drib_target = rcsc::Vector2D(47.0,0);
    
    rcsc::Vector2D center4metr = me + rcsc::Vector2D::polar2vector( 4, (rcsc::Vector2D(52.5,0)-me).dir() );
    rcsc::Vector2D up4metr = me + rcsc::Vector2D::polar2vector( 4, (rcsc::Vector2D(me.x,me.y-4)-me).dir() );
    rcsc::Vector2D down4metr = me + rcsc::Vector2D::polar2vector( 4, (rcsc::Vector2D(me.x,me.y+4)-me).dir() );
    
    int centerOpponents = wm.countOpponentsIn( rcsc::Circle2D(center4metr,4),3,true );
    int upOpponents = wm.countOpponentsIn( rcsc::Circle2D(up4metr,4),3,true  );
    int downOpponents = wm.countOpponentsIn( rcsc::Circle2D(down4metr,4),3,true  );
    
    
    if( ball.x > 36.0 && ball.absY() < 15.0 )
    {
       if( ball.y < 0 && ( (centerOpponents > 0 && downOpponents == 0) ||
                           (centerOpponents > 1 && downOpponents < centerOpponents) ) )
            drib_target = rcsc::Vector2D( me.x+3, me.y+6 );

       if( ball.y > 0 && ( (centerOpponents > 0 && upOpponents == 0) ||
                           (centerOpponents > 1 && upOpponents < centerOpponents) ) )
            drib_target = rcsc::Vector2D( me.x+3, me.y-6 );
    }

    const AngleDeg drib_angle = ( drib_target - wm.self().pos() ).th();

// ************************************************************


    Vector2D posGoalie = Vector2D(-100.0,0.0);

    const PlayerPtrCont::const_iterator end = wm.teammatesFromSelf().end();
    for ( PlayerPtrCont::const_iterator p = wm.teammatesFromSelf().begin();
          p != end;
          ++p )
    {
        if ( (*p)->goalie() )
           posGoalie = (*p)->pos();
    }

    if( me.dist( posGoalie ) < 3.0 || me.dist(rcsc::Vector2D(-52.5,0.0)) < 7.0 )
    {
      if( Bhv_ZhPass2().DKhBClear( agent ) )
      {
         agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
         return true;
      }
      else if( Body_ClearBall().execute( agent ) )
      {
         agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
         return true;
      }

    }

////////////////////////////////////////////////////////////////////////////////


  if( Bhv_ZhPass2().SRPtoCenter( agent ) )
  {
     return true;
  }

  if( Bhv_ZhPass2().SRPD( agent ) )
  {
     return true;
  }

// 检查传球

    rcsc::Vector2D tttarget = rcsc::Vector2D (0.0,0.0);

    if( nearest_opp_dist < minDist )
    {
        if ( Bhv_ZhPass2().execute( agent ) )
        {
            agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
            return true;
        }
    }

  if( Bhv_ZhPass2().SRPtoOutside( agent ) )
  {
     return true;
  }

    if( ball.x < -36.0 && ball.absY() < 20.0 &&
        wm.countTeammatesIn( Circle2D( me , 7.0 ) , 3 , true ) > 2 && 
        wm.countOpponentsIn( Circle2D( me , 5.0 ) , 3 , false ) > 0 &&
        wm.teammatesFromSelf().front()->distFromSelf() < 4.0 )
    {
       if( Bhv_ZhPass2().DKhBClear( agent ) )
       {
          agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
          return true;
       }
    }

    if( ball.x < -27.0 && ball.absY() < 20.0 &&
        wm.countTeammatesIn( Circle2D( me , 3.0 ) , 3 , false ) >= 2 &&wm.countOpponentsIn( Circle2D( me , 3.0 ) , 3 , false ) >= 1 )
    {
       if( Bhv_ZhPass2().DKhBClear( agent ) )
       {
          agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
          return true;
       }
    }

    if( ball.x < -27.0 && ball.absY() < 22.0 &&
        wm.countOpponentsIn( Circle2D( me , 2.0 ) , 4 , false ) >= 1 &&
        wm.opponentsFromSelf().front()->distFromSelf() < 3.0 )
    {
       if( Bhv_ZhPass2().DKhBClear( agent ) )
       {
          agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
          return true;
       }
    }

    const PlayerObject * opp_goalie = wm.getOpponentGoalie();

      if( opp_goalie && opp_goalie->distFromSelf() < 2.0 )
      {
         Body_KickOneStep( Vector2D(52.5,sign(ball.y)*5.0 ),
                                 ServerParam::i().ballSpeedMax()
                                 ).execute( agent );
         agent->setNeckAction( new Neck_ScanField() );
         return true;
      }

    if ( nearest_opp_dist < 5.0
         && nearest_opp_dist > ( ServerParam::i().tackleDist()
                                 + ServerParam::i().defaultPlayerSpeedMax() * 1.5 )
         && wm.self().body().abs() < 70.0 &&
         !(ball.x > 36.0 && ball.absY() < 22.0) )
    {
        Vector2D body_dir_drib_target
            = wm.self().pos()
            + Vector2D::polar2vector(5.0, wm.self().body());

             if( ball.x > 33.0 )
                body_dir_drib_target = me + Vector2D::polar2vector( 20.0, (goal-me).dir() );

        int max_dir_count = 0;
        wm.dirRangeCount( wm.self().body(), 20.0, &max_dir_count, NULL, NULL );

        if ( body_dir_drib_target.x < ServerParam::i().pitchHalfLength() - 1.0
             && body_dir_drib_target.absY() < ServerParam::i().pitchHalfWidth() - 1.0
             && max_dir_count < 3
             )
        {
            const Sector2D sector( wm.self().pos(),
                                         0.5, 10.0,
                                         wm.self().body() - 30.0,
                                         wm.self().body() + 30.0 );
            if ( ! wm.existOpponentIn( sector, 10, true ) )
            {
                dlog.addText( Logger::TEAM,
                                    __FILE__": (execute) dribble to my body dir" );
                agent->debugClient().addMessage( "OffKickDrib(1)" );
                Body_Dribble( body_dir_drib_target,
                                    1.0,
                                    ServerParam::i().maxDashPower(),
                                    2,
                                    shouldAvoid
                                    ).execute( agent );
                agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
                return true;
            }
        }
    }


    if ( nearest_opp_pos.x < wm.self().pos().x + 0.0 ) //+1.0
    {
        const Sector2D sector( wm.self().pos(),
                                     0.5, 15.0,
                                     drib_angle - 30.0,
                                     drib_angle + 30.0 );
        // 检查对手球员,包括门将
        if ( ! wm.existOpponentIn( sector, 10, true ) )
        {
            const int max_dash_step
                = wm.self().playerType()
                .cyclesToReachDistance( wm.self().pos().dist( drib_target ) );
            if ( wm.self().pos().x > 35.0 )
            {
                drib_target.y *= ( 10.0 / drib_target.absY() );
            }


            if( ball.x > 33.0 ) 
                 drib_target = me + Vector2D::polar2vector( 20.0, (goal-me).dir() );


            dlog.addText( Logger::TEAM,
                                __FILE__": (execute) fast dribble to (%.1f, %.1f) max_step=%d",
                                drib_target.x, drib_target.y,
                                max_dash_step );
            agent->debugClient().addMessage( "OffKickDrib(2)" );

            Body_Dribble( drib_target,
                                1.0,
                                ServerParam::i().maxDashPower(),
                                std::min( 5, max_dash_step ),
                                shouldAvoid
                                ).execute( agent );
        }
        else
        {
            dlog.addText( Logger::TEAM,
                                __FILE__": (execute) slow dribble to (%.1f, %.1f)",
                                drib_target.x, drib_target.y );
            agent->debugClient().addMessage( "OffKickDrib(3)" );

            Body_Dribble( drib_target,
                                1.0,
                                ServerParam::i().maxDashPower(),
                                2,
                                shouldAvoid
                                ).execute( agent );

        }
        agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
        return true;
    }

    // 当对手球员是远离我的时候
    if ( nearest_opp_dist > 4.5 )
    {
        dlog.addText( Logger::TEAM,
                            __FILE__": opp far. dribble(%.1f, %.1f)",
                            drib_target.x, drib_target.y );
        agent->debugClient().addMessage( "OffKickDrib(4)" );
        Body_Dribble( drib_target,
                            1.0,
                            ServerParam::i().maxDashPower(),
                            1,
                            shouldAvoid
                            ).execute( agent );
        agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
        return true;
    }

    // 对手球员接近了
    // 如果能传球
    if ( Bhv_ZhPass2().execute( agent ) )
    {
        dlog.addText( Logger::TEAM,
                            __FILE__": (execute) pass",
                            __LINE__ );
        agent->debugClient().addMessage( "OffKickPass(3)" );
        agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
        return true;
    }

    // 对手球员又远离我了
    if ( nearest_opp_dist > 3.0 )
    {
        Body_Dribble( drib_target,
                            1.0,
                            ServerParam::i().maxDashPower(),
                            1,
                            shouldAvoid
                            ).execute( agent );
        agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
        return true;
    }


    if ( nearest_opp_dist > 2.5 )
    {
        Body_Dribble( rcsc::Vector2D(-30.0,0.0),
                            1.0,
                            ServerParam::i().maxDashPower(),
                            1,
                            shouldAvoid
                            ).execute( agent );
        agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
        return true;
    }

    if ( wm.self().pos().x > wm.offsideLineX() - 10.0 )
    {

        Body_Dribble( rcsc::Vector2D(-30.0,0.0),
                            1.0,
                            ServerParam::i().maxDashPower(),
                            1,
                            shouldAvoid
                            ).execute( agent );
        agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
        return true;
    }
    else
    {

        Body_Dribble( rcsc::Vector2D(-30.0,0.0),
                            1.0,
                            ServerParam::i().maxDashPower(),
                            1,
                            shouldAvoid
                            ).execute( agent );
        agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
        return true;

    }

    return true;

}


/*原版
bool
Bhv_BasicOffensiveKick::execute( PlayerAgent * agent )
{

    dlog.addText( Logger::TEAM,
                  __FILE__": Bhv_BasicOffensiveKick" );

    const WorldModel & wm = agent->world();

    const PlayerPtrCont & opps = wm.opponentsFromSelf();
    const PlayerObject * nearest_opp
        = ( opps.empty()
            ? static_cast< PlayerObject * >( 0 )
            : opps.front() );
    const double nearest_opp_dist = ( nearest_opp
                                      ? nearest_opp->distFromSelf()
                                      : 1000.0 );
    const Vector2D nearest_opp_pos = ( nearest_opp
                                       ? nearest_opp->pos()
                                       : Vector2D( -1000.0, 0.0 ) );

    Vector2D pass_point;
    if ( Body_Pass::get_best_pass( wm, &pass_point, NULL, NULL ) )
    {
        if ( pass_point.x > wm.self().pos().x - 1.0 )
        {
            bool safety = true;
            const PlayerPtrCont::const_iterator opps_end = opps.end();
            for ( PlayerPtrCont::const_iterator it = opps.begin();
                  it != opps_end;
                  ++it )
            {
                if ( (*it)->pos().dist( pass_point ) < 4.0 )
                {
                    safety = false;
                }
            }

            if ( safety )
            {
                dlog.addText( Logger::TEAM,
                              __FILE__": (execute) do best pass" );
                agent->debugClient().addMessage( "OffKickPass(1)" );
                Body_Pass().execute( agent );
                agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
                return true;
            }
        }
    }

    if ( nearest_opp_dist < 7.0 )
    {
        if ( Body_Pass().execute( agent ) )
        {
            dlog.addText( Logger::TEAM,
                          __FILE__": (execute) do best pass" );
            agent->debugClient().addMessage( "OffKickPass(2)" );
            agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
            return true;
        }
    }

    // dribble to my body dir
    if ( nearest_opp_dist < 5.0
         && nearest_opp_dist > ( ServerParam::i().tackleDist()
                                 + ServerParam::i().defaultPlayerSpeedMax() * 1.5 )
         && wm.self().body().abs() < 70.0 )
    {
        const Vector2D body_dir_drib_target
            = wm.self().pos()
            + Vector2D::polar2vector(5.0, wm.self().body());

        int max_dir_count = 0;
        wm.dirRangeCount( wm.self().body(), 20.0, &max_dir_count, NULL, NULL );

        if ( body_dir_drib_target.x < ServerParam::i().pitchHalfLength() - 1.0
             && body_dir_drib_target.absY() < ServerParam::i().pitchHalfWidth() - 1.0
             && max_dir_count < 3
             )
        {
            // check opponents
            // 10m, +-30 degree
            const Sector2D sector( wm.self().pos(),
                                   0.5, 10.0,
                                   wm.self().body() - 30.0,
                                   wm.self().body() + 30.0 );
            // opponent check with goalie
            if ( ! wm.existOpponentIn( sector, 10, true ) )
            {
                dlog.addText( Logger::TEAM,
                              __FILE__": (execute) dribble to my body dir" );
                agent->debugClient().addMessage( "OffKickDrib(1)" );
                Body_Dribble( body_dir_drib_target,
                              1.0,
                              ServerParam::i().maxDashPower(),
                              2
                              ).execute( agent );
                agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
                return true;
            }
        }
    }

    Vector2D drib_target( 50.0, wm.self().pos().absY() );
    if ( drib_target.y < 20.0 ) drib_target.y = 20.0;
    if ( drib_target.y > 29.0 ) drib_target.y = 27.0;
    if ( wm.self().pos().y < 0.0 ) drib_target.y *= -1.0;
    const AngleDeg drib_angle = ( drib_target - wm.self().pos() ).th();

    // opponent is behind of me
    if ( nearest_opp_pos.x < wm.self().pos().x + 1.0 )
    {
        // check opponents
        // 15m, +-30 degree
        const Sector2D sector( wm.self().pos(),
                               0.5, 15.0,
                               drib_angle - 30.0,
                               drib_angle + 30.0 );
        // opponent check with goalie
        if ( ! wm.existOpponentIn( sector, 10, true ) )
        {
            const int max_dash_step
                = wm.self().playerType()
                .cyclesToReachDistance( wm.self().pos().dist( drib_target ) );
            if ( wm.self().pos().x > 35.0 )
            {
                drib_target.y *= ( 10.0 / drib_target.absY() );
            }

            dlog.addText( Logger::TEAM,
                          __FILE__": (execute) fast dribble to (%.1f, %.1f) max_step=%d",
                          drib_target.x, drib_target.y,
                          max_dash_step );
            agent->debugClient().addMessage( "OffKickDrib(2)" );
            Body_Dribble( drib_target,
                          1.0,
                          ServerParam::i().maxDashPower(),
                          std::min( 5, max_dash_step )
                          ).execute( agent );
        }
        else
        {
            dlog.addText( Logger::TEAM,
                          __FILE__": (execute) slow dribble to (%.1f, %.1f)",
                          drib_target.x, drib_target.y );
            agent->debugClient().addMessage( "OffKickDrib(3)" );
            Body_Dribble( drib_target,
                          1.0,
                          ServerParam::i().maxDashPower(),
                          2
                          ).execute( agent );

        }
        agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
        return true;
    }

    // opp is far from me
    if ( nearest_opp_dist > 5.0 )
    {
        dlog.addText( Logger::TEAM,
                      __FILE__": opp far. dribble(%.1f, %.1f)",
                      drib_target.x, drib_target.y );
        agent->debugClient().addMessage( "OffKickDrib(4)" );
        Body_Dribble( drib_target,
                      1.0,
                      ServerParam::i().maxDashPower() * 0.4,
                      1
                      ).execute( agent );
        agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
        return true;
    }

    // opp is near

    // can pass
    if ( Body_Pass().execute( agent ) )
    {
        dlog.addText( Logger::TEAM,
                      __FILE__": (execute) pass",
                      __LINE__ );
        agent->debugClient().addMessage( "OffKickPass(3)" );
        agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
        return true;
    }

    // opp is far from me
    if ( nearest_opp_dist > 3.0 )
    {
        dlog.addText( Logger::TEAM,
                      __FILE__": (execute) opp far. dribble(%f, %f)",
                      drib_target.x, drib_target.y );
        agent->debugClient().addMessage( "OffKickDrib(5)" );
        Body_Dribble( drib_target,
                      1.0,
                      ServerParam::i().maxDashPower() * 0.2,
                      1
                      ).execute( agent );
        agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
        return true;
    }

    if ( nearest_opp_dist > 2.5 )
    {
        dlog.addText( Logger::TEAM,
                      __FILE__": hold" );
        agent->debugClient().addMessage( "OffKickHold" );
        Body_HoldBall().execute( agent );
        agent->setNeckAction( new Neck_TurnToLowConfTeammate() );
        return true;
    }

    {
        dlog.addText( Logger::TEAM,
                      __FILE__": clear" );
        agent->debugClient().addMessage( "OffKickAdvance" );
        Body_AdvanceBall().execute( agent );
        agent->setNeckAction( new Neck_ScanField() );
    }

    return true;

}
*/
