// -*-c++-*-

/*
 *Copyright (C) TJNU
 */

/////////////////////////////////////////////////////////////////////

#ifndef BHV_DANGER_AREA_MOVE_H
#define BHV_DANGER_AREA_MOVE_H

#include <rcsc/geom/vector_2d.h>
#include <rcsc/player/soccer_action.h>

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif


#include <rcsc/common/server_param.h>
#include <rcsc/common/logger.h>
#include <rcsc/player/player_agent.h>
#include <rcsc/player/debug_client.h>
#include <rcsc/player/intercept_table.h>

#include <rcsc/action/basic_actions.h>
#include <rcsc/action/body_go_to_point.h>
#include <rcsc/action/neck_turn_to_ball_or_scan.h>

#include <rcsc/action/body_intercept2009.h>

#include "bhv_get_ball.h"
#include "bhv_danger_area_tackle.h"
#include "yu_modify/neck_check_ball_owner.h"
#include "yu_modify/neck_default_intercept_neck.h"
#include "yu_modify/neck_offensive_intercept_neck.h"

#include "strategy.h"

namespace rcsc {
class WorldModel;
}

class Bhv_DangerAreaMove
    : public rcsc::SoccerBehavior {
private:
    rcsc::Vector2D M_home_pos ;
public:

    Bhv_DangerAreaMove( rcsc::Vector2D home_pos ) : M_home_pos( home_pos )
    { }

    bool execute( rcsc::PlayerAgent * agent )
    {
	//
	// tackle
	//
	if ( Bhv_DangerAreaTackle().execute( agent ) )
	{
		return true;
	}
	
	//
	// intercept
	//
	if ( doIntercept( agent ) )
	{
		return true;
	}
	
	//
	// get ball
	//
	if ( doGetBall( agent ) )
	{
		return true;
	}
	
	//
	// normal move
	//
	doNormalMove( agent );
	
	return true;
    }

private:
    bool doIntercept( rcsc::PlayerAgent * agent )
    {
	const WorldModel & wm = agent->world();
	
	int self_min = wm.interceptTable()->selfReachCycle();
	int mate_min = wm.interceptTable()->teammateReachCycle();
	int opp_min = wm.interceptTable()->opponentReachCycle();
	
	if ( self_min <= opp_min + 5
		&& self_min <= mate_min + 2
		&& ! wm.existKickableTeammate() )
	{
		Body_Intercept2009().execute( agent );
		if ( opp_min >= self_min + 3 )
		{
			agent->setNeckAction( new Neck_OffensiveInterceptNeck() );
		}
		else
		{
			agent->setNeckAction( new Neck_DefaultInterceptNeck
					    ( new Neck_TurnToBallOrScan() ) );
		}
		return true;
	}

	return false;
    }
    bool doGetBall( rcsc::PlayerAgent * agent )
    {
	
	const WorldModel & wm = agent->world();
	
	int opp_min = wm.interceptTable()->opponentReachCycle();
	const PlayerObject * fastest_opp = wm.interceptTable()->fastestOpponent();
	Vector2D opp_trap_pos = wm.ball().inertiaPoint( opp_min );
	
	if ( wm.existKickableTeammate()
		|| ! fastest_opp
		|| opp_trap_pos.x > -30.0
		|| opp_trap_pos.dist( M_home_pos ) > 7.0
		|| opp_trap_pos.absY() > 13.0
		)
	{
		return false;
	}
	
	//
	// search other blocker
	//
	bool exist_blocker = false;
	
	const double my_dist = wm.self().pos().dist( opp_trap_pos );
	const PlayerPtrCont::const_iterator end = wm.teammatesFromBall().end();
	for ( PlayerPtrCont::const_iterator p = wm.teammatesFromBall().begin();
		p != end;
		++p )
	{
		if ( (*p)->goalie() ) continue;
		if ( (*p)->isGhost() ) continue;
		if ( (*p)->posCount() >= 3 ) continue;
		if ( (*p)->pos().x > fastest_opp->pos().x ) continue;
		if ( (*p)->pos().x > opp_trap_pos.x ) continue;
		if ( (*p)->pos().dist( opp_trap_pos ) > my_dist ) continue;
	
		exist_blocker = true;
		break;
	}
	
	//
	// try intercept
	//
	if ( exist_blocker )
	{
		int self_min = wm.interceptTable()->selfReachCycle();
		Vector2D self_trap_pos = wm.ball().inertiaPoint( self_min );
		if ( self_min <= 10
		&& self_trap_pos.dist( M_home_pos ) < 10.0 )
		{
			Body_Intercept2009().execute( agent );
			if ( opp_min >= self_min + 3 )
			{
				agent->setNeckAction( new Neck_OffensiveInterceptNeck() );
			}
			else
			{
				agent->setNeckAction( new Neck_DefaultInterceptNeck
						( new Neck_TurnToBallOrScan() ) );
			}
			return true;
		}
	
		return false;
	}
	
	//
	// try get ball
	//
	double max_x = -34.0;
	Rect2D bounding_rect( Vector2D( -60.0, M_home_pos.y - 6.0 ),
				Vector2D( max_x, M_home_pos.y + 6.0 ) );
	if ( Bhv_GetBall( bounding_rect ).execute( agent ) )
	{
		return true;
	}

	return false;
    }
    void doNormalMove( rcsc::PlayerAgent * agent )
    {

	const WorldModel & wm = agent->world();
	Vector2D target_point = M_home_pos;
	target_point.x = Strategy::get_defense_line_x( wm, target_point );
	
	double dash_power = Strategy::get_defender_dash_power( wm, target_point );
	double dist_thr = std::fabs( wm.ball().pos().x - wm.self().pos().x ) * 0.1;
	if ( dist_thr < 0.5 ) dist_thr = 0.5;

//         cout<<target_point<<endl;

	doGoToPoint( agent, target_point, dist_thr, dash_power + 10 , 12.0 );
	
	agent->setNeckAction( new Neck_CheckBallOwner() );
     }

     void doGoToPoint( rcsc::PlayerAgent * agent,
		       const rcsc::Vector2D & target_point,
		       const double & dist_thr,
		       const double & dash_power,
		       const double & dir_thr )
     {
	
	const WorldModel & wm = agent->world();
	
	
	if ( Body_GoToPoint( target_point, dist_thr, dash_power,
				1, // 1 step
				false, // no back dash
				true, // save recovery
				dir_thr
				).execute( agent ) )
	{
		return;
	}
	
	// already there
	
	Vector2D ball_next = wm.ball().pos() + wm.ball().vel();
	Vector2D my_final = wm.self().inertiaFinalPoint();
	AngleDeg ball_angle = ( ball_next - my_final ).th();
	
	AngleDeg target_angle;
	if ( ball_next.x < -30.0 )
	{
		target_angle = ball_angle + 90.0;
		if ( ball_next.x < -45.0 )
		{
		if ( target_angle.degree() < 0.0 )
		{
			target_angle += 180.0;
		}
		}
		else
		{
		if ( target_angle.degree() > 0.0 )
		{
			target_angle += 180.0;
		}
		}
	}
	else
	{
		target_angle = ball_angle + 90.0;
		if ( ball_next.x > my_final.x + 15.0 )
		{
		if ( target_angle.abs() > 90.0 )
		{
			target_angle += 180.0;
		}
		}
		else
		{
		if ( target_angle.abs() < 90.0 )
		{
			target_angle += 180.0;
		}
		}
	}
	
	Body_TurnToAngle( target_angle ).execute( agent );
	
    }
};

#endif
