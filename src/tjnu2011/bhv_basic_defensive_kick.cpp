// -*-c++-*-

/*
 *Copyright (C) TJNU
 */

/////////////////////////////////////////////////////////////////////

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "bhv_basic_defensive_kick.h"

#include <rcsc/action/body_advance_ball.h>
#include "body_clear_ball2008.h"
#include <rcsc/action/body_pass.h>
#include <rcsc/action/body_dribble2008.h>
#include <rcsc/action/body_hold_ball2008.h>

#include <rcsc/action/neck_scan_field.h>
#include <rcsc/action/basic_actions.h>

#include <rcsc/player/player_agent.h>
#include <rcsc/player/debug_client.h>

#include <rcsc/common/logger.h>
#include <rcsc/common/server_param.h>

#include "yu_modify/bhv_dribble.h"
#include "yu_modify/bhv_dribble2009.h"


bool Bhv_BasicDefensiveKick::execute( rcsc::PlayerAgent * agent )
{

	const rcsc::WorldModel & wm = agent->world();
	
	const rcsc::PlayerObject * nearest_opp = wm.getOpponentNearestToSelf( 7 );
	const double nearest_opp_dist = ( nearest_opp
					? nearest_opp->distFromSelf()
					: 1000.0 );

//
// pass
//

	Vector2D pass_point( 0.0 , 0.0 );

	if( Body_Pass::get_best_pass( wm , &pass_point , NULL , NULL ) )
	{
		if( pass_point.x > wm.self().pos().x + 18 )
		{
			if( Body_Pass().execute( agent ) )
			{
				return true;
			}
		}

		if( pass_point.x > wm.self().pos().x + 7.0 )
		{
			double opp_dist = 1000.0;
			const rcsc::PlayerObject * opp
				= wm.getOpponentNearestTo( pass_point,
							   10,
							   &opp_dist );

			if( opp && opp_dist > 7.0 )
			{
				if ( rcsc::Body_Pass().execute( agent ) )
				{
					return true;
				}
				else
				{
					Body_Dribble2008( pass_point ,
							  1.0,
							  rcsc::ServerParam::i().maxPower() * 0.8,
							  1,
							  false ).execute( agent );
				}
			}
		}
	}
//
//Dribble
//

	if ( nearest_opp_dist > 12.0 )
	{
		if ( Bhv_Dribble().execute(agent) )
			;
		else
		{
			Bhv_Dribble2009().execute( agent );
		}
		agent->setNeckAction( new rcsc::Neck_ScanField() );

		return true;
	}
	
	if ( nearest_opp_dist > 8.0
		&& wm.self().pos().x > -30.0 )
	{
		if ( Bhv_Dribble().execute(agent) )
			;
		else
		{
			Bhv_Dribble2009().execute( agent );
		}
		agent->setNeckAction( new rcsc::Neck_ScanField() );

		return true;
	}
	
	if ( rcsc::Body_Pass().execute( agent ) )
	{
		return true;
	}
	
	if ( nearest_opp_dist > 12.0
	     && wm.self().pos().absY() > 20.0 )
	{
		if( !nearest_opp )
		{
			if ( Bhv_Dribble().execute(agent) )
				;
			else
			{
				Bhv_Dribble2009().execute( agent );
			}
			agent->setNeckAction( new rcsc::Neck_ScanField() );

			return true;
		}
		else
		{
			if( wm.self().pos().y > nearest_opp->pos().y )
			{
				rcsc::Body_Dribble2008( wm.self().pos() + Vector2D( 10 , 10 ),
						  1.0,
						  rcsc::ServerParam::i().maxPower() * 0.8,
						  1,
						  false ).execute( agent );
				return true;
			}
			else
			{
				rcsc::Body_Dribble2008( wm.self().pos() + Vector2D( 10 , -10 ),
						  1.0,
						  rcsc::ServerParam::i().maxPower() * 0.8,
						  1,
						  false ).execute( agent );
				return true;
			}
		}
	}
	
	if ( nearest_opp_dist > 5.0
	     || ( nearest_opp_dist > 3.0
	     && nearest_opp
	     && nearest_opp->pos().x < wm.ball().pos().x - 0.5 )
	   )
	{
		rcsc::Vector2D target( 36.0, wm.self().pos().y * 1.1 );
		if ( target.y > 30.0 ) target.y = 30.0;
		if ( target.y < -30.0 ) target.y = -30.0;
		rcsc::Body_Dribble2008( target,
					1.0,
					rcsc::ServerParam::i().maxPower() * 0.8,
					1,
					false // never avoid
					).execute( agent );
		agent->setNeckAction( new rcsc::Neck_ScanField() );
		return true;
	}
	
	if ( nearest_opp_dist > 4.0 )
	{
		agent->debugClient().addMessage( "DefHold" );
	
		rcsc::Vector2D face_point( 100.0, 0.0 );
		rcsc::Body_HoldBall2008( true, face_point ).execute( agent );
		agent->setNeckAction( new rcsc::Neck_ScanField() );
		return true;
	}
	
	if ( wm.self().pos().x < -35.0
		&& wm.self().pos().absY() < 16.0 )
	{

		Body_ClearBall2008().execute( agent );
		agent->setNeckAction( new rcsc::Neck_ScanField() );
		return true;
	}

	#if 1
		rcsc::Body_AdvanceBall().execute( agent );
		agent->setNeckAction( new rcsc::Neck_ScanField() );
	#else
		rcsc::Body_Pass().execute( agent );
		agent->setNeckAction( new rcsc::Neck_ScanField() );
	#endif
	
	return true;
}
