// -*-c++-*-

/*
 *Copyright (C) TJNU
 */

/////////////////////////////////////////////////////////////////////

#ifndef BHV_SIDE_BACK_MOVE_H
#define BHV_SIDE_BACK_MOVE_H

#include <rcsc/geom/vector_2d.h>
#include <rcsc/player/soccer_action.h>
#include <rcsc/common/server_param.h>

#include <rcsc/action/body_intercept2009.h>
#include <rcsc/action/body_turn_to_point.h>
#include <rcsc/action/neck_turn_to_point.h>
#include <rcsc/action/body_go_to_point.h>

#include "bhv_basic_tackle.h"
#include "yu_modify/neck_offensive_intercept_neck.h"
#include "bhv_danger_area_move.h"
#include "body_forestall_block.h"



using namespace rcsc;

// added : 2009 / 07 / 19 , Hossein Mobasher
// works::
// 1. try to tackle
// 2. try to intercept
// 3. try to block ball owner ** later
// 4. try to fast positioning
// 5. try to normal moving

class Bhv_SideBackMove
    : public rcsc::SoccerBehavior {

private:
    const rcsc::Vector2D M_home_pos;
public:
    Bhv_SideBackMove( Vector2D home_pos )
       : M_home_pos( home_pos )
      { }

    bool execute( PlayerAgent * agent )
    {
      if( doTackle( agent ) )
      {
        //cout<<"TACKLE :: "<<agent->world().time().cycle()<<" : "<<agent->world().time().cycle()<<endl;
        return true;
      }

      if( doForestall( agent ) )
      {
        return true;
      }

      if( doAttackForTackle( agent ) && agent->world().ball().pos().x < -30.0 )
      {
        return true;
      }

      if( doIntercept( agent ) )
      {
        //cout<<"INTERCEPT :: "<<agent->world().time().cycle()<<" : "<<agent->world().time().cycle()<<endl;
        return true;
      }

      if( doEmergencyMove( agent ) )
      {
        //cout<<"EMERGENCY_MOVE :: "<<agent->world().time().cycle()<<" : "<<agent->world().time().cycle()<<endl;
        return true;
      }

      //cout<<"NORMAL_MOVE :: "<<agent->world().time().cycle()<<" : "<<agent->world().time().cycle()<<endl;
      doNormalMove( agent );
      return true;
    }

    bool doTackle( PlayerAgent * agent )
    {
      if( Bhv_BasicTackle( 0.85 , 90 ).execute( agent ) )
      {
        return true;
      }

      return false;
    }

    bool doAttackForTackle( PlayerAgent * agent )
    {
       const WorldModel & wm = agent->world();
       const PlayerObject * opp = wm.getOpponentNearestToBall(10);

       if( !opp )
       {
//          cout<<"no need for attack"<<" : "<<agent->world().time().cycle()<<endl;
         return false;
       }

       if( Bhv_BasicTackle( 0.9 , 90 ).execute( agent ) )
       {
//          cout<<"tackle behavior has acted"<<" : "<<agent->world().time().cycle()<<endl;
         return true;
       }


       Vector2D opp_pos = opp->pos();
       Vector2D opp_pos_next_cycle = opp_pos + opp->vel();

       if( opp_pos_next_cycle.absY() < opp_pos.absY() && 
           opp_pos.dist( wm.ball().pos() ) < agent->world().self().distFromBall() &&
           opp_pos.dist( wm.ball().pos() ) < 2.0 )
       {
         if( opp_pos.dist( wm.self().pos() ) < 8 )
         {
           if( doIntercept( agent ) )
           {
              return true;
           }

           if( Body_GoToPoint( wm.ball().pos(),
                               0.1,
                               ServerParam::i().maxPower(),
                               1,
                               false,
                               false,
                               18.0 ).execute( agent ) )
           {
//               cout<<"i go to point with 18 degree"<<" : "<<agent->world().time().cycle()<<endl;
              agent->setNeckAction( new Neck_TurnToPoint( wm.ball().pos() ) );
              return true;
           }
           else
           {
//               cout<<"i turn to point now"<<" : "<<agent->world().time().cycle()<<endl;
              Body_TurnToPoint( wm.ball().pos() ).execute( agent );
           }
         }
         else
         {
           WorldModelHighLevel WM;
           Vector2D home_pos = M_home_pos;
           home_pos.x = ( opp_pos_next_cycle.x + opp_pos.x ) / 2 ;
           if( home_pos.absY() > opp_pos_next_cycle.absY() &&
               home_pos.y*opp_pos_next_cycle.y > 0 )
           {
             home_pos.y = wm.ball().pos().y - 2 * WM.sign( home_pos.y );
           }

            if( Body_GoToPoint( home_pos,
                                0.1,
                                ServerParam::i().maxPower(),
                                1,
                                false,
                                false,
                                12.0 ).execute( agent ) )
            {
//                cout<<"i go to point with 12 degree"<<" : "<<agent->world().time().cycle()<<endl;
               agent->setNeckAction( new Neck_TurnToPoint( wm.ball().pos() ) );
               return true;
            }
            else
            {
//                cout<<"i turn to point now"<<" : "<<agent->world().time().cycle()<<endl;
               Body_TurnToPoint( home_pos ).execute( agent );
            }
         }
       }
       else
       {
//          cout<<"no need for attack"<<" : "<<agent->world().time().cycle()<<endl;
         return false;
       }
       return false;
    }

    bool doForestall( rcsc::PlayerAgent * agent )
    {
	const WorldModel & wm = agent->world();
	int self_min = wm.interceptTable()->selfReachCycle();
	int mate_min = wm.interceptTable()->teammateReachCycle();
	int opp_min = wm.interceptTable()->opponentReachCycle();
	
	if ( wm.self().stamina() > ServerParam::i().staminaMax() * 0.6
		&& opp_min < 3
		&& opp_min < mate_min - 2
		&& ( opp_min < self_min - 2
		|| opp_min == 0
		|| ( opp_min == 1 && self_min > 1 ) )
		&& wm.ball().pos().dist( M_home_pos ) < 10.0
		&& wm.ball().distFromSelf() < 15.0 )
	{
		if ( Body_ForestallBlock( M_home_pos ).execute( agent ) )
		{
			agent->setNeckAction( new Neck_TurnToBall() );
			return true;
		}
	}
	
	return false;
    }

    bool doIntercept( PlayerAgent * agent )
    {
       const WorldModel & wm = agent->world();
       int self_min = wm.interceptTable()->selfReachCycle();
       int mate_min = wm.interceptTable()->teammateReachCycle();

       if( ( ( mate_min >= 2
              && self_min <= 4 )
         || ( self_min <= mate_min + 1
              && mate_min >= 4 ) ) &&
          wm.ball().distFromSelf() < 3.0 )
       {
          Vector2D face_point( 52.5, wm.self().pos().y );
          if ( wm.self().pos().absY() < 10.0 )
          {
             face_point.y *= 0.8;
          }
          else if ( wm.self().pos().absY() < 20.0 )
          {
             face_point.y *= 0.9;
          }

          Body_Intercept2009( true , face_point ).execute( agent );
          agent->setNeckAction( new Neck_OffensiveInterceptNeck() );
          return true;
      }

      return false;
    }

    bool doEmergencyMove( PlayerAgent * agent )
    {
       const WorldModel & wm = agent->world();

       int self_min = wm.interceptTable()->selfReachCycle();
       int mate_min = wm.interceptTable()->teammateReachCycle();

       if( self_min > mate_min + 1 )
       {
         return false;
       }

       const PlayerObject * opp = wm.getOpponentNearestToBall( 10 );

       if( !opp )
       {
         return false;
       }

       int opp_min = wm.interceptTable()->opponentReachCycle();


       Vector2D ball_pos = wm.ball().pos();
       Vector2D opp_pos  = opp->pos();

       if( self_min < opp_min && self_min < mate_min )
       {
          Vector2D face_point( 52.5, wm.self().pos().y );
          if ( wm.self().pos().absY() < 10.0 )
          {
             face_point.y *= 0.8;
          }
          else if ( wm.self().pos().absY() < 20.0 )
          {
             face_point.y *= 0.9;
          }

          Body_Intercept2009( false , face_point ).execute( agent );
          return true;
       }

       bool save_recovery = true;
       Vector2D move_point = M_home_pos;
       WorldModelHighLevel WM;

       if( ball_pos.x < -30.0 && ball_pos.dist( wm.self().pos() ) < 15.0 )
       {
         save_recovery = false;
         if( ball_pos.absY() < M_home_pos.absY() && ball_pos.absY() < 22.0 )
         {
            move_point.y = ball_pos.y + 1*WM.sign( ball_pos.y );
            if( ball_pos.x < M_home_pos.x )
            {
              move_point.x = ball_pos.x - 2;
            }
         }
         else
         {
           move_point.y = ball_pos.y;
           if( ball_pos.x < M_home_pos.x )
           {
             move_point.x = ball_pos.x - 2;
           }
         }

         if( Body_GoToPoint( move_point , 0.1 , ServerParam::i().maxPower() , 1 , false , save_recovery , 18.0
           ).execute( agent ) )
         {
           agent->setNeckAction( new Neck_TurnToPoint( opp_pos ) );
           return true;
         }
         else
         {
           Body_TurnToPoint( move_point ).execute( agent );
         }
         return true;
       }

       return false;
    }

    void doNormalMove( PlayerAgent * agent )
    {
         Bhv_DefensiveMove( M_home_pos , true , true ).execute( agent );
    }

};

#endif
