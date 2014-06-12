// -*-c++-*-

/*
 *Copyright (C) TJNU
 */

/////////////////////////////////////////////////////////////////////

#include "bhv_defensive_move.h"

#include "bhv_basic_tackle.h"

#include <rcsc/action/basic_actions.h>
#include <rcsc/action/body_intercept2009.h>
#include <rcsc/action/body_go_to_point.h>
#include <rcsc/action/neck_turn_to_ball_or_scan.h>
#include <rcsc/action/neck_turn_to_low_conf_teammate.h>
#include <rcsc/action/bhv_go_to_point_look_ball.h>

#include <rcsc/player/player_agent.h>
#include <rcsc/player/debug_client.h>
#include <rcsc/player/intercept_table.h>

#include <rcsc/common/logger.h>
#include <rcsc/common/server_param.h>

#include "yu_modify/neck_offensive_intercept_neck.h"
#include "bhv_danger_area_tackle.h"

#include "strategy.h"


bool Bhv_DefensiveMove::execute( rcsc::PlayerAgent * agent )
{

// 	ofstream fout( "./file/Log_Def_Move.txt" , ios::app );
	const rcsc::WorldModel & wm = agent->world();
	/*--------------------------------------------------------*/
	// tackle

	if ( Bhv_BasicTackle( 0.95 , 90.0 ).execute( agent ) )
	{
		//cout<<"( Basic Tackle , "<<wm.time().cycle()<<" ) "<<endl;
		return true;
	}

	/*--------------------------------------------------------*/
	//intercept ball

	const int self_min = wm.interceptTable()->selfReachCycle    ();
	const int mate_min = wm.interceptTable()->teammateReachCycle();
	const int opp_min  = wm.interceptTable()->opponentReachCycle();

	if( wm.ball().pos().x < -10.0 &&
	    !wm.existKickableTeammate() &&
	    !wm.existKickableOpponent() &&
	    self_min < mate_min && self_min < opp_min + 6 )
	{
	        Vector2D face_point( -52.5, agent->world().self().pos().y * 0.9 );
		rcsc::Body_Intercept2009( false , face_point ).execute( agent );
		agent->setNeckAction( new Neck_OffensiveInterceptNeck() );
		return true;
	}

	if( !wm.existKickableTeammate() && 
	     wm.self().pos().x < wm.ball().pos().x - 2 &&
	     self_min < mate_min && self_min < opp_min + 1 && wm.ball().distFromSelf() < 8.0 &&
	     ( wm.self().unum() > 6 || wm.self().pos().x < -1 ) 
	     )
	{
	        Vector2D face_point( -52.5, agent->world().self().pos().y * 0.9 );
		rcsc::Body_Intercept2009( false , face_point ).execute( agent );
		agent->setNeckAction( new Neck_OffensiveInterceptNeck() );
		return true;
	}

	if(  wm.self().pos().x < -17 && !wm.existKickableTeammate() &&
	     wm.self().pos().x < wm.ball().pos().x - 2 &&
	     self_min < mate_min && wm.ball().distFromSelf() < 5.0 )
	{
	        Vector2D face_point(-52.5, agent->world().self().pos().y * 0.9 );
		Body_Intercept2009( false , face_point ).execute( agent );
		agent->setNeckAction( new Neck_OffensiveInterceptNeck() );
		return true;
	}

	/*--------------------------------------------------------*/
	//go to ball with maximum dash power

// 	bool save_stamina = true;
	double dist_thr = wm.ball().distFromSelf() * 0.1;
	if ( dist_thr < 0.5 ) dist_thr = 0.5;


	if ( !wm.existKickableTeammate() && !wm.existKickableOpponent() &&
	     //self_min < mate_min && 
	     self_min <= opp_min &&
	     wm.self().stamina() > ServerParam::i().staminaMax() * 0.5 &&
	    ( wm.self().unum() > 6 || wm.self().pos().x > 0 ) 
	    )
	{
		if ( ! rcsc::Body_GoToPoint( wm.ball().pos() , dist_thr, rcsc::ServerParam::i().maxPower(),
					     1 ).execute( agent ) )
		{
			if ( M_turn_at )
			{
				rcsc::Body_TurnToBall().execute( agent );
			}
			else
			{
					return false;
			}
		}

		if ( M_turn_neck )
		{
			if ( wm.existKickableOpponent()
			  && wm.ball().distFromSelf() < 18.0 )
			{
				agent->setNeckAction( new rcsc::Neck_TurnToBall() );
			}
			else
			{
				agent->setNeckAction( new rcsc::Neck_TurnToBallOrScan() );
			}
		}
		agent->setArmAction( new Arm_PointToPoint( wm.ball().pos() ) );
		return true;
	}
	/*--------------------------------------------------------*/
	// chase ball
	
	if ( ! wm.existKickableTeammate()
		&& ( self_min <= 3
		|| ( self_min < mate_min + 3
			&& self_min < opp_min + 4 )
		)
		)
	{
		Body_Intercept2009().execute( agent );

	
		if ( M_turn_neck )
		{
	#if 0
		if ( self_min == 4 && opp_min >= 2 )
		{
			agent->setViewAction( new rcsc::View_Wide() );
		}
		else if ( self_min == 3 && opp_min >= 2 )
		{
			agent->setViewAction( new rcsc::View_Normal() );
		}
		else if ( self_min > 10 )
		{
			agent->setViewAction( new rcsc::View_Normal() );
		}
	
		if ( wm.ball().distFromSelf()
			< rcsc::ServerParam::i().visibleDistance() )
		{
			agent->setNeckAction( new rcsc::Neck_TurnToLowConfTeammate() );
		}
		else
		{
			agent->setNeckAction( new rcsc::Neck_TurnToBallOrScan() );
		}
	#else
		agent->setNeckAction( new Neck_OffensiveInterceptNeck() );
	#endif
		}
		return true;
	}
	
	const double dash_power = getDashPower( agent, M_home_pos );

	agent->debugClient().addMessage( "BasicMove%.0f", dash_power );
	agent->debugClient().setTarget( M_home_pos );
	agent->debugClient().addCircle( M_home_pos, dist_thr );
	
	
	Vector2D move_pos = M_home_pos;
	double   dash__power = dash_power;

	if ( wm.ball().pos().x < -20.0 && wm.ball().distFromSelf() <= 3 )//intercept ball that near to me
	{
		//cout<<"( Intercept Ball , "<<wm.time().cycle()<<" ) "<<endl;
	        Vector2D face_point(-52.5, agent->world().self().pos().y * 0.9 );
		Body_Intercept2009( false , face_point ).execute( agent );

		agent->setNeckAction( new Neck_OffensiveInterceptNeck() );
		return true;
	}

	if ( wm.ball().pos().x  > wm.self().pos().x + 22.0 )//increase dash power to recovery
	{
// 		cout<<"This condition is action now !!!"<<endl;
		dash__power *= 0.6;
	}



	if ( ! rcsc::Body_GoToPoint( move_pos , dist_thr, dash__power , 1
					).execute( agent ) )
	{
		//cout<<"( Go To Point - 2 , "<<wm.time().cycle()<<" ) "<<endl;
		if ( M_turn_at )
		{
			rcsc::Body_TurnToBall().execute( agent );
		}
		else
		{
			return false;
		}
	}

	if ( M_turn_neck )
	{
		if(  wm.existKickableOpponent()
		  && wm.ball().distFromSelf() < 18.0 )
		{
			agent->setNeckAction( new rcsc::Neck_TurnToBall() );
		}
		else
		{
			agent->setNeckAction( new rcsc::Neck_TurnToBallOrScan() );
		}
	}
	agent->setArmAction( new Arm_PointToPoint( move_pos ) );
	return true;
}



double Bhv_DefensiveMove::getDashPower( const rcsc::PlayerAgent * agent,
                     const rcsc::Vector2D & target_point )
{
	static bool s_recover_mode = false;
	
	const rcsc::WorldModel & wm = agent->world();

	const PlayerObject * tmm_to_ball = wm.getTeammateNearestToBall( 10 );	

	const int self_min = wm.interceptTable()->selfReachCycle();
	const int mate_min = wm.interceptTable()->teammateReachCycle();
	const int opp_min = wm.interceptTable()->opponentReachCycle();
	
	// check recover
	if ( wm.self().stamina() < rcsc::ServerParam::i().staminaMax() * 0.375 )
	{
		s_recover_mode = true;
	}
	else if ( wm.self().stamina() > rcsc::ServerParam::i().staminaMax() * 0.6 )
	{
		s_recover_mode = false;
	}
	
	/*--------------------------------------------------------*/
	double dash_power = rcsc::ServerParam::i().maxPower();
	const double my_inc
		= wm.self().playerType().staminaIncMax()
		* wm.self().recovery();

// 	const PlayerObject * near_opp = wm.getOpponentNearestToBall(10);

	if ( wm.self().pos().dist( wm.ball().pos() ) > 20 && wm.self().stamina() < 7000 &&
	     wm.ball().pos().x - wm.self().pos().x > 15.0 )
	{
		dash_power = rcsc::ServerParam::i().maxPower() * 0.6;
	}
	else if (  wm.defenseLineX() < wm.self().pos().x
		&& wm.ball().pos().x < wm.defenseLineX() + 15.0 
		&& M_home_pos.x < 0.0 )
	{
		// keep max power
		dash_power = rcsc::ServerParam::i().maxPower();
	}
	else if ( s_recover_mode )
	{
		dash_power = my_inc - 25.0; // preffered recover value
		if ( dash_power < 0.0 ) dash_power = 0.0;
	}
	// exist kickable teammate
	else if ( wm.existKickableTeammate()
		&& wm.ball().distFromSelf() < 20.0 )
	{
		dash_power = std::min( my_inc * 1.1,
				rcsc::ServerParam::i().maxPower() );
	}
	// exist kickable opponent
	else if ( (wm.existKickableOpponent()
		   && wm.ball().distFromSelf() < 15 ) ||
		   wm.ball().pos().x - wm.self().pos().x < 15 )
	{
		dash_power = rcsc::ServerParam::i().maxPower();
	}
	// not exist kickable opponents and teammates
	else if ( tmm_to_ball && tmm_to_ball->pos().dist( wm.self().pos() ) <= 1.0
		  &&!wm.existKickableOpponent() && !wm.existKickableTeammate()
		  && wm.ball().distFromSelf() < 3.0 )
	{
		dash_power = rcsc::ServerParam::i().maxPower();
	}
	// in offside area
	else if ( wm.self().pos().x > wm.offsideLineX() )
	{
		dash_power = rcsc::ServerParam::i().maxPower();
	}
	else if ( wm.ball().pos().x > 25.0
		&& wm.ball().pos().x >wm.self().pos().x && wm.self().pos().x > 10.0
		&& self_min < opp_min - 6
		&& mate_min < opp_min - 6 )
	{
		dash_power = rcsc::bound( rcsc::ServerParam::i().maxPower() * 0.1,
					my_inc * 0.5,
					rcsc::ServerParam::i().maxPower() );
	}
	// normal
	else
	{
		dash_power = std::min( my_inc ,
				rcsc::ServerParam::i().maxPower() );
	}
	
	return dash_power;
}
