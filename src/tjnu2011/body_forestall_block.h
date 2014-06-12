// -*-c++-*-

/*
 *Copyright:

 Copyright (C) Hidehisa AKIYAMA

 This code is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2, or (at your option)
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

/*
 *Modified copyright (C) TJNU
 */

/////////////////////////////////////////////////////////////////////

#ifndef BODY_FORESTALL_BLOCK_H
#define BODY_FORESTALL_BLOCK_H

#include <rcsc/geom/vector_2d.h>
#include <rcsc/player/soccer_action.h>

#include <rcsc/action/body_intercept2009.h>

#include <rcsc/action/body_go_to_point.h>
#include <rcsc/action/body_turn_to_angle.h>

#include <rcsc/player/player_agent.h>
#include <rcsc/player/intercept_table.h>
#include <rcsc/player/debug_client.h>

#include <rcsc/common/logger.h>
#include <rcsc/common/server_param.h>
#include <rcsc/soccer_math.h>


namespace rcsc {
class WorldModel;
}

class Body_ForestallBlock
    : public rcsc::BodyAction {
private:
    static int S_count_no_move;
    const rcsc::Vector2D M_home_position;

public:
    explicit
    Body_ForestallBlock( const rcsc::Vector2D & home_pos )
        : M_home_position( home_pos )
      { }

    bool execute( rcsc::PlayerAgent * agent )
    {

	static int S_count_no_move = 0;
	rcsc::dlog.addText( rcsc::Logger::TEAM,
				__FILE__":Body_ForestallBlock. no_move_count=%d",
				S_count_no_move );
	
	const rcsc::WorldModel & wm = agent->world();
	
	rcsc::Vector2D block_point = getTargetPointSimple( wm );
	
	if ( ! block_point.valid() )
	{
		return false;
	}
	
	double dist_thr = 1.0;
	
	agent->debugClient().addMessage( "forestall" );
	agent->debugClient().setTarget( block_point );
	agent->debugClient().addCircle( block_point, dist_thr );
	
	double dash_power = wm.self().getSafetyDashPower( rcsc::ServerParam::i().maxPower() );
	
	if ( rcsc::Body_GoToPoint( block_point, dist_thr, dash_power,
				5, // cycle
				false, // no back dash
				true, // save recovery
				40.0 // dir_thr
				).execute( agent ) )
	{
		S_count_no_move = 0;
		return true;
	}
	
	#if 1
	++S_count_no_move;
	
	if ( S_count_no_move >= 10 )
	{
		rcsc::dlog.addText( rcsc::Logger::TEAM,
				__FILE__": forestall. long no move. attack to the ball" );
		agent->debugClient().addMessage( "ForestallAttack" );
		rcsc::Body_Intercept2009( true ).execute( agent );
		return true;
	}
	#endif
	
	rcsc::AngleDeg body_angle = wm.ball().angleFromSelf() + 90.0;
	if ( body_angle.abs() < 90.0 )
	{
		body_angle += 180.0;
	}
	
	rcsc::Body_TurnToAngle( body_angle ).execute( agent );
	
	return true;
    }

private:

    rcsc::Vector2D getTargetPointSimple( const rcsc::WorldModel & wm )
    {
	const rcsc::PlayerObject * opp = wm.interceptTable()->fastestOpponent();
	
	if ( ! opp )
	{
		rcsc::dlog.addText( rcsc::Logger::ROLE,
				__FILE__": execute(). no opponent" );
		return rcsc::Vector2D::INVALIDATED;
	}
	
	rcsc::Vector2D block_point = opp->pos();
	
	if ( opp->pos().x > -36.0 )
	{
		if ( wm.self().pos().x > opp->pos().x )
		{
		block_point.x -= 5.0;
		}
		else
		{
		block_point.x -= 2.0;
		}
	
		if ( block_point.y > 0.0 ) block_point.y -= 0.5;
		else block_point.y += 0.5;
	}
	else if ( opp->pos().absY() > 7.0 )
	{
		if ( block_point.y > 0.0 ) block_point.y -= 1.0;
		else block_point.y += 1.0;
	}
	else
	{
		block_point.x -= 2.0;
	}
	
	
	rcsc::dlog.addText( rcsc::Logger::TEAM,
				__FILE__": target opp=%d(%,2f, %,2f) block_point=(%.1f %.1f)",
				opp->unum(), opp->pos().x, opp->pos().y,
				block_point.x, block_point.y );
	
	return block_point;
    }
    rcsc::Vector2D getTargetPoint( const rcsc::WorldModel & wm )
    {
	const rcsc::PlayerObject * opp = wm.interceptTable()->fastestOpponent();
	
	if ( ! opp )
	{
		rcsc::dlog.addText( rcsc::Logger::ROLE,
				__FILE__": execute(). no opponent" );
		return rcsc::Vector2D::INVALIDATED;
	}
	
	rcsc::Vector2D block_point = opp->pos();
	block_point.x -= 7.0;
	
	// already more backward position than opponent
	if ( opp->pos().x > wm.self().pos().x )
	{
		rcsc::Line2D perpend_line
		= rcsc::Line2D::perpendicular_bisector( wm.self().pos(),
							opp->pos() );
		rcsc::Line2D opp_line( opp->pos(),
				opp->pos() + rcsc::Vector2D( -5.0, 0.0 ) );
	
	
		rcsc::Vector2D intersection = perpend_line.intersection( opp_line );
	
		if ( ! intersection.valid() )
		{
		block_point.x = wm.self().pos().x;
		block_point.y = opp->pos().y;
		}
		else if ( intersection.x > opp->pos().x )
		{
		if ( wm.self().pos().x < opp->pos().x )
		{
			block_point = opp->pos();
			block_point.x -= 7.0;
		}
		}
	}
	
	/*
	rcsc::Ray2D my_body_ray( wm.self().pos(),
				wm.self().body() );
	rcsc::Vector2D
	*/
	
	
	return block_point;
    }

};

#endif
