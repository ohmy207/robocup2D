/*
 *Copyright (C) TJNU
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

//好吧,头文件
#include "bhv_ZhMark.h"
#include "bhv_set_play.h"
#include "strategy.h"
#include "bhv_set_play_free_kick.h"
#include "bhv_set_play_goal_kick.h"
#include "bhv_set_play_kick_in.h"
#include "bhv_set_play_kick_off.h"
#include "bhv_their_goal_kick_move.h"

#include <rcsc/action/basic_actions.h>
#include <rcsc/action/bhv_before_kick_off.h>
#include <rcsc/action/bhv_scan_field.h>
#include <rcsc/action/body_go_to_point.h>
#include <rcsc/action/neck_scan_field.h>
#include <rcsc/action/neck_turn_to_ball_or_scan.h>

#include <rcsc/player/audio_sensor.h>
#include <rcsc/player/player_agent.h>
#include <rcsc/player/debug_client.h>
#include <rcsc/common/audio_memory.h>

#include <rcsc/common/logger.h>
#include <rcsc/common/server_param.h>
#include <rcsc/geom/line_2d.h>

/*-------------------------------------------------------------------*/
/*!

*/
bool
Bhv_ZhMark::execute( rcsc::PlayerAgent * agent )
{
	//球员与target对应的过程
     static std::string msg = "";

     if( agent->world().gameMode().time().cycle() - agent->world().time().cycle() < 3 )
          msg = "";
     std::string temp = agent->audioSensor().freeformMessage();

     if( temp.size() > 0 && ((char *)(temp.c_str()))[0]=='M' )
          msg = temp;

     char x = ((char *)(msg.c_str()))[agent->world().self().unum()-1];
     int target_opp_number;

     switch( x )
     {
         case 'z': target_opp_number = 2;    break;
         case 'y': target_opp_number = 3;    break;
         case 'x': target_opp_number = 4;    break;
         case 'w': target_opp_number = 5;    break;
         case 'v': target_opp_number = 6;    break;
         case 'u': target_opp_number = 7;    break;
         case 't': target_opp_number = 8;    break;
         case 's': target_opp_number = 9;    break;
         case 'r': target_opp_number = 10;   break;
         case 'q': target_opp_number = 11;   break;
         default:
         {
            return false;
         }
     }

     rcsc::Vector2D target ;
     rcsc::Vector2D posAgent = agent->world().self().pos();
     rcsc::Vector2D posBall = agent->world().ball().pos();

     double dist_thr = 0.5;
     double MARK_DIST = 1.0;
     double maxDist = 17.0;
     //或以为可扩大,如取个数27.0到36.0之间
     if( agent->world().ball().pos().x > 36.0 )
            maxDist = 15.0;

    if( agent->world().self().unum() < 9 &&agent->world().gameMode().side() != agent->world().ourSide() &&agent->world().gameMode().type() == rcsc::GameMode::GoalieCatch_ )
         return false;

     if( agent->world().theirPlayer( target_opp_number ) )
     {
         target = agent->world().theirPlayer( target_opp_number )->pos();
         target += rcsc::Vector2D::polar2vector( MARK_DIST , ( posBall-target ).th() );

          if( target.dist( HOME_POS ) > maxDist )
                return false;

          if( agent->world().gameMode().type() == rcsc::GameMode::GoalKick_ && posBall.x > 36.0 &&
              target.x > 35.0 && target.absY() < 20.5 )
          {
              if( target.x < 41.0 && target.absY() < 14.0 )
                target = rcsc::Vector2D( 35.0, target.y );
              else if ( target.absY() > 14.0 )
                target = rcsc::Vector2D( target.x-1.0 , ( target.y > 0.0 ? 21.0 : -21.0 ) );
              else
                return false;
          }

          if( target.dist( posBall ) < 10.0 )
               target = posBall + rcsc::Vector2D::polar2vector( 10.0, (target-posBall).th() );

          if( target.x > 52.5 || target.x < -53.5 || target.absY() > 36.0 )
               return false;
      }
      else
      {
         agent->doTurn( 30 );
         agent->setNeckAction( new rcsc::Neck_ScanField() );
         target = posAgent;
         return true;
      }

      double dash_power = Bhv_SetPlay::get_set_play_dash_power( agent );

      if ( ! rcsc::Body_GoToPoint( target, dist_thr, dash_power ).execute( agent ) )
      {
         rcsc::AngleDeg body_angle = agent->world().ball().angleFromSelf();
         if ( body_angle.degree() < 0.0 )
                body_angle -= 90.0;
         else
                body_angle += 90.0;

         rcsc::Vector2D body_point = posAgent;
         body_point += rcsc::Vector2D::polar2vector( 10.0, body_angle );

         rcsc::Body_TurnToPoint( body_point ).execute( agent );
      }

  agent->setNeckAction( new rcsc::Neck_TurnToBall() );
  return true;
}
