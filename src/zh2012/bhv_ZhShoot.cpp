/*
 *Copyright (C) TJNU
 */


#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "bhv_ZhShoot.h"

#include <rcsc/common/logger.h>

#include <rcsc/player/player_agent.h>
#include <rcsc/player/debug_client.h>

#include <rcsc/action/body_hold_ball.h>
#include <rcsc/action/neck_turn_to_goalie_or_scan.h>
#include <rcsc/action/body_smart_kick.h>


bool Bhv_ZhShoot::execute( rcsc::PlayerAgent * agent )
{

    if ( ! agent->world().self().isKickable() )
        return false;

    rcsc::Vector2D ball = agent->world().ball().pos();

    bool shooting1 = false;
    bool shooting2 = false;


    rcsc::Vector2D shootTarget = rcsc::Vector2D(52.5,0.0);

    //这次是25个target
    rcsc::Vector2D target[25];

    target[0]  = rcsc::Vector2D(52.5, 0.0 );
    target[1]  = rcsc::Vector2D(52.5, 3.3 );
    target[2]  = rcsc::Vector2D(52.5, -3.3);
    target[3]  = rcsc::Vector2D(52.5, 6.0 );
    target[4]  = rcsc::Vector2D(52.5, -6.0);
    target[5]  = rcsc::Vector2D(52.5, 5.0 );
    target[6]  = rcsc::Vector2D(52.5, -5.0);
    target[7]  = rcsc::Vector2D(52.5, 3.0 );
    target[8]  = rcsc::Vector2D(52.5, -3.0);
    target[9]  = rcsc::Vector2D(52.5, 2.0 );
    target[10] = rcsc::Vector2D(52.5, -2.0);
    target[11] = rcsc::Vector2D(52.5, 1.0 );
    target[12] = rcsc::Vector2D(52.5, -1.0);
    target[13] = rcsc::Vector2D(52.5, 0.5 );
    target[14] = rcsc::Vector2D(52.5, -0.5);
    target[15] = rcsc::Vector2D(52.5, 1.5 );
    target[16] = rcsc::Vector2D(52.5, -1.5);
    target[17] = rcsc::Vector2D(52.5, 2.5 );
    target[18] = rcsc::Vector2D(52.5, -2.5);
    target[19] = rcsc::Vector2D(52.5, 3.5 );
    target[20] = rcsc::Vector2D(52.5, -3.5);
    target[21] = rcsc::Vector2D(52.5, 4.5 );
    target[22] = rcsc::Vector2D(52.5, -4.5);
    target[23] = rcsc::Vector2D(52.5, 5.5 );
    target[24] = rcsc::Vector2D(52.5, -5.5);

    for( int k = 0; k < 25; k++ )
    {
       if( agent->world().gameMode().type() == rcsc::GameMode::PenaltyTaken_ &&
           ball.x < 0 )
          target[k].x *= -1.0;
    }

    if( fabs( ball.dist(target[3]) - ball.dist(target[4]) ) > 12.0 )
    {
       for( int zh = 0; zh < 25; zh++ )
       {
          target[zh].y = std::min( target[zh].y, 4.5 );
       }
    }
    else if( fabs( ball.dist(target[3]) - ball.dist(target[4]) ) > 8.0 )
    {
       for( int kh = 0; kh < 25; kh++ )
       {
          target[kh].y = std::min( target[kh].y, 5.0 );
       }
    }
    else if( fabs( ball.dist(target[3]) - ball.dist(target[4]) ) > 6.0 )
    {
       for( int kh = 0; kh < 25; kh++ )
       {
          target[kh].y = std::min( target[kh].y, 5.5 );
       }
    }
    else if( fabs( ball.dist(target[3]) - ball.dist(target[4]) ) > 3.0 )
    {
       for( int kh = 0; kh < 25; kh++ )
       {
          target[kh].y = std::min( target[kh].y, 5.9 );
       }
    }


    double first_speed = 2.8;

    rcsc::Vector2D one_step_vel
        = rcsc::KickTable::calc_max_velocity( ( shootTarget - agent->world().ball().pos() ).th(),
                                        agent->world().self().kickRate(),
                                        agent->world().ball().vel() );
    double one_step_speed = one_step_vel.r();


     for( int i = 0; i < 25; i++ )
     {
       if( canScore( agent, target[i], one_step_speed ) )
        {
            shootTarget = target[i];
            shooting1 = true;
            break;
        }
     }


    if( shooting1 == false )
    {
      for( int i = 0; i < 25; i++ )
      {
        if( canScore( agent, target[i], first_speed ) )
         {
             shootTarget = target[i];
             shooting2 = true;
             break;
        }
      }
    }

    if( !shooting1 && !shooting2 )
    {
         return false;
    }

//又是踢一脚
     if ( one_step_speed > first_speed * 0.6  && shooting1 ) // * 0.92
     {
        rcsc::Body_SmartKick( shootTarget,
                        one_step_speed,
                        one_step_speed - 0.0001,
                        1 ).execute( agent );
        agent->setNeckAction( new rcsc::Neck_TurnToGoalieOrScan( -1 ) );
        return true;
     }

//未知的 step shoot
     if (  shooting2 )
     {
        rcsc::Body_SmartKick( shootTarget,
                        first_speed,
                        first_speed*0.92,
                        1 ).execute( agent );
        agent->setNeckAction( new rcsc::Neck_TurnToGoalieOrScan( -1 ) );
        return true;
     }

     rcsc::Body_HoldBall().execute( agent );

     agent->setNeckAction( new rcsc::Neck_TurnToGoalieOrScan( -1 ) );
     return true;


}


bool Bhv_ZhShoot::canScore( rcsc::PlayerAgent * agent, const rcsc::Vector2D & shootTarget,
                            const double one_step_speed )
{

  const rcsc::WorldModel & wm = agent->world();

  rcsc::Vector2D ball = wm.ball().pos();

  if( agent->world().gameMode().type() != rcsc::GameMode::PenaltyTaken_ &&
      (ball.x < 25.0 || ball.absY() > 28.0) )
       return false;

  rcsc::AngleDeg shootDir = (shootTarget - ball).dir();
  bool catched = false;

  double ballSpeed = one_step_speed;

  rcsc::Vector2D nextBall = ball + rcsc::Vector2D::polar2vector( ballSpeed, (shootTarget - ball).dir() );

  const rcsc::PlayerObject * oppGoalie = wm.getOpponentGoalie();

  if( agent->world().gameMode().type() == rcsc::GameMode::PenaltyTaken_ &&
      (oppGoalie->posCount() > 2 || oppGoalie->distFromSelf() > 12.0 ) )
       return false;

//查找对手门将
  const rcsc::PlayerPtrCont::const_iterator o_end = wm.opponentsFromBall().end();
  for ( rcsc::PlayerPtrCont::const_iterator o = wm.opponentsFromBall().begin();
        o != o_end;
        ++o )
  {
     const rcsc::PlayerType * player_type = (*o)->playerTypePtr();

     bool isGoalie = false;

     double getableArea = player_type->kickableArea();

     if ( (*o)->goalie() )
     {
         isGoalie = true;
         getableArea = 1.25;
     }

     if( !isGoalie )
          continue;


        if( isGoalie && (*o)->posCount() > 2 )
        {
           catched = true;
           return false;
           break;
        }


     rcsc::Vector2D oppPos = (*o)->pos() + (*o)->vel();

     int time = 1;

     float weakness = 0.2;

     if( one_step_speed < 2.2 )
          weakness = 0.0;


     float goalX = 52.5;

       if( agent->world().gameMode().type() == rcsc::GameMode::PenaltyTaken_ &&
           ball.x < 0 )
            goalX *= -1.0;

     while( nextBall.x < goalX && ballSpeed > 0.5 )
     {
       if( time == 1 && oppPos.dist( nextBall ) < getableArea )
       {
                catched = true;
                return false;
                break;
       }
       else
       {
          float goalieX = oppPos.x;
          float goalieY = oppPos.y;

          float goalieXpower = getableArea + 0.3;

            if( time > 1 && ball.x > 48.0 && ball.absY() > 7.0 )
                goalieXpower += 0.5;
            if( time > 2 )
                goalieXpower += 0.5;
            if( time > 4 )
                goalieXpower += 0.7;
            if( time > 5 )
                goalieXpower += (time - 5)* 0.7;


          if( ( nextBall.y < goalieY + getableArea + (time - weakness) ) &&
              ( nextBall.y > goalieY - getableArea - (time - weakness) ) &&
              fabs( nextBall.x - goalieX ) < goalieXpower )
          {
                catched = true;
                return false;
                break;
          }


       }

      ballSpeed *= .94;
      nextBall += rcsc::Vector2D::polar2vector( ballSpeed, shootDir );
      time++;
     }




       if( nextBall.x < goalX )
            catched = true;

       if( time > 20 )
            catched = true;
  }


/// Other Opponents Check
  ballSpeed = one_step_speed;
  nextBall = ball + rcsc::Vector2D::polar2vector( ballSpeed, (shootTarget - ball).dir() );

  const rcsc::PlayerPtrCont::const_iterator opp_end = wm.opponentsFromBall().end();
  for ( rcsc::PlayerPtrCont::const_iterator opp = wm.opponentsFromBall().begin();
        opp != opp_end;
        ++opp )
  {
    const rcsc::PlayerType * player_type = (*opp)->playerTypePtr();
    double kickableArea = player_type->kickableArea();

    if( agent->world().gameMode().type() == rcsc::GameMode::PenaltyTaken_ )
        break;

    if ( (*opp)->goalie() )
        continue;
    else
    {

      if( (*opp)->posCount() > 3 )
         continue;

      int time = 1;

      float goalX = 52.5;

       if( agent->world().gameMode().type() == rcsc::GameMode::PenaltyTaken_ &&
           ball.x < 0 )
            goalX *= -1.0;

      while( nextBall.x < goalX && ballSpeed > 0.8 )
      {

         rcsc::Vector2D oPos = (*opp)->pos() + (*opp)->vel();
         if( time == 1 || time == 2 )
         {
           if( oPos.dist( nextBall ) < kickableArea )
              return false;
         }
         else if( time >= 3 )
         {
           if( oPos.dist( nextBall ) < kickableArea + (time - 1) )
              return false;
         }

       ballSpeed *= .94;
       nextBall += rcsc::Vector2D::polar2vector( ballSpeed, shootDir );
       time++;
      }

    }

  }



  if( catched )
     return false;

return true;
}
