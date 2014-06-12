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

// -*-c++-*-

/*
 *Modified copyright (C) TJNU
 */

/////////////////////////////////////////////////////////////////////

#include "bhv_get_ball.h"

using namespace rcsc;

bool Bhv_GetBall::execute( rcsc::PlayerAgent * agent )
{

	const WorldModel & wm = agent->world();
	
	int self_min = wm.interceptTable()->selfReachCycle();
	int mate_min = wm.interceptTable()->teammateReachCycle();
	int opp_min = wm.interceptTable()->opponentReachCycle();
	
	//
	// check ball owner
	//
	if ( //mate_min <= self_min - 3
		//|| ( mate_min == 1 && self_min >= 3 )
		mate_min == 0
		|| wm.existKickableTeammate() )
	{
		return false;
	}
	
	//
	// tackle
	//
	if ( wm.ball().pos().x < -35.0
		&& wm.ball().pos().absY() < 20.0 )
	{
		if ( Bhv_DangerAreaTackle().execute( agent ) )
		{
			return true;
		}
	}
	
	if ( Bhv_BasicTackle( 0.9, 80.0 ).execute( agent ) )
	{
		return true;
	}
	
	//
	// intercept
	//
	if ( ! wm.existKickableTeammate()
		&& self_min <= mate_min + 1
		&& self_min <= opp_min )
	{
		Body_Intercept2009().execute( agent );
		agent->setNeckAction( new Neck_TurnToBall() );
		return true;
	}
	
	
	Vector2D opp_trap_pos = wm.ball().inertiaPoint( opp_min );
	
	//
	// searth the best point
	//
	Vector2D center_pos( -44.0, 0.0 );
	if ( opp_trap_pos.x < -38.0
		&& opp_trap_pos.absY() < 7.0 )
	{
		center_pos.x = -52.5;
		center_pos.y = opp_trap_pos.y * 0.6;
	}
	else if ( opp_trap_pos.x < -38.0
		&& opp_trap_pos.absY() < 12.0 )
	{
		center_pos.x = -50.0;
		//center_pos.y = 2.5 * sign( opp_trap_pos.y );
		center_pos.y = 6.5 * sign( opp_trap_pos.y );
	}
	else if ( opp_trap_pos.x < -30.0
		&& 2.0 < opp_trap_pos.absY()
		&& opp_trap_pos.absY() < 8.0 )
	{
		center_pos.x = -50.0;
		center_pos.y = opp_trap_pos.y * 0.9;
	}
	else if ( opp_trap_pos.absY() > 25.0 )
	{
		center_pos.x = -44.0;
		center_pos.y = 20.0 * sign( opp_trap_pos.y );
	}
	else if ( opp_trap_pos.absY() > 20.0 )
	{
		center_pos.x = -44.0;
		center_pos.y = 15.0 * sign( opp_trap_pos.y );
	}
	else if ( opp_trap_pos.absY() > 15.0 )
	{
		center_pos.x = -44.0;
		center_pos.y = 10.0 * sign( opp_trap_pos.y );
	}
	else
	{

	}

	double dist_thr = wm.self().playerType().kickableArea() - 0.2;
	bool save_recovery = true;
	
	Param param;
	simulate( wm, center_pos, M_bounding_rect, dist_thr, save_recovery,
		&param );
	
	agent->debugClient().addLine( opp_trap_pos, center_pos );
	agent->debugClient().addRectangle( M_bounding_rect );
	
	if ( ! param.point_.valid() )
	{
		return false;
	}
	
	//
	// execute action
	//
	if ( Body_GoToPoint( param.point_,
				dist_thr,
				ServerParam::i().maxPower(),
				param.cycle_,
				false, // no back mode
				save_recovery,
				15.0 // dir threshold
				).execute( agent ) )
	{
		agent->debugClient().setTarget( param.point_ );
		agent->debugClient().addCircle( param.point_, dist_thr );
		agent->debugClient().addMessage( "GetBallGoTo" );
	}
	else
	{
	
		AngleDeg body_angle = wm.ball().angleFromSelf() + 90.0;
		if ( body_angle.abs() < 90.0 )
		{
		body_angle += 180.0;
		}
	
		Body_TurnToAngle( body_angle ).execute( agent );
	
		agent->debugClient().setTarget( param.point_ );
		agent->debugClient().addCircle( param.point_, dist_thr );
		agent->debugClient().addMessage( "GetBallTurnTo%.0f",
						body_angle.degree() );
	}
	
	agent->setNeckAction( new Neck_CheckBallOwner() );
	
	return true;
}

void Bhv_GetBall::simulate( const rcsc::WorldModel & wm,
                            const rcsc::Vector2D & center_pos,
                            const rcsc::Rect2D & bounding_rect,
                            const double & dist_thr,
                            const bool save_recovery,
                            Param * param )
{
	const PlayerObject * opp = wm.interceptTable()->fastestOpponent();
	
	if ( ! opp )
	{
		return;
	}
	
	MSecTimer timer;

	const double pitch_half_length = ServerParam::i().pitchHalfLength() - 1.0;
	const double pitch_half_width = ServerParam::i().pitchHalfWidth() - 1.0;
	
	const int opp_min = wm.interceptTable()->opponentReachCycle();
	const Vector2D trap_pos = ( opp_min == 0
					? opp->inertiaPoint( 10 )
					: wm.ball().inertiaPoint( opp_min ) );
	const Vector2D unit_vec = ( center_pos - trap_pos ).setLengthVector( 1.0 );
	
	const int opp_penalty_step = 1;
	
	//
	// binary search
	//
	
	Param best;
	
	const double max_length = std::min( 20.0, trap_pos.dist( center_pos ) ) + 1.0;
	
	int count = 0;
	for ( double len = 0.3; len < max_length; len += 1.0, ++count )
	{
		Vector2D target_point = trap_pos + unit_vec * len;
		//
		// region check
		//
		if ( ! bounding_rect.contains( target_point ) )
		{
			continue;
		}
	
		if ( target_point.absX() > pitch_half_length
		|| target_point.absY() > pitch_half_width )
		{
			continue;
		}
	
		//
		// predict self cycle
		//
		double stamina = 0.0;
		int self_cycle = predictSelfReachCycle( wm,
							target_point,
							dist_thr,
							save_recovery,
							&stamina );
		if ( self_cycle > 100 )
		{
		continue;
		}
	
		double opp_dist = trap_pos.dist( target_point );
		int opp_cycle
		= opp_min
		+ opp_penalty_step
		+ opp->playerTypePtr()->cyclesToReachDistance( opp_dist );
	
	
	//         if ( self_cycle < opp_cycle - 1 )
		if ( self_cycle < opp_cycle - 1
		//|| self_cycle <= 2 && self_cycle < opp_cycle )
		|| self_cycle <= 1 && self_cycle < opp_cycle
		)
		{
		best.point_ = target_point;
		best.cycle_ = self_cycle;
		best.stamina_ = stamina;
		break;
		}
	}
	
	
	if ( best.point_.valid() )
	{
		*param = best;
	}
	else
	{

 	}
}


int Bhv_GetBall::predictSelfReachCycle( const rcsc::WorldModel & wm,
                                        const rcsc::Vector2D & target_point,
                                        const double & dist_thr,
                                        const bool save_recovery,
                                        double * stamina )
{

	const ServerParam & param = ServerParam::i();
	const PlayerType & self_type = wm.self().playerType();
	const double max_moment = param.maxMoment();
	const double recover_dec_thr = param.staminaMax() * param.recoverDecThr();
	
	const double first_my_speed = wm.self().vel().r();
	
	for ( int cycle = 0; cycle < 25; ++cycle )
	{
		Vector2D my_pos = wm.self().inertiaPoint( cycle );
		double target_dist = ( target_point - my_pos ).r();
		if ( target_dist - dist_thr > self_type.realSpeedMax() * cycle )
		{
		continue;
		}
	
		int n_turn = 0;
		int n_dash = 0;
	
		AngleDeg target_angle = ( target_point - my_pos ).th();
		double my_speed = first_my_speed;
	
		//
		// turn
		//
		double angle_diff = ( target_angle - wm.self().body() ).abs();
		double turn_margin = 180.0;
		if ( dist_thr < target_dist )
		{
		turn_margin = std::max( 15.0,
					AngleDeg::asin_deg( dist_thr / target_dist ) );
		}
	
		while ( angle_diff > turn_margin )
		{
		angle_diff -= self_type.effectiveTurn( max_moment, my_speed );
		my_speed *= self_type.playerDecay();
		++n_turn;
		}
	
		double my_stamina = wm.self().stamina();
		double my_effort = wm.self().effort();
		double my_recovery = wm.self().recovery();
	
	#if 0
		// TODO: stop dash
		if ( n_turn >= 3 )
		{
		Vector2D vel = wm.self().vel();
		vel.rotate( - wm.self().body() );
		Vector2D stop_accel( -vel.x, 0.0 );
	
		double dash_power
			= stop_accel.x / ( self_type.dashPowerRate() * my_effort );
		double stop_stamina = stop_dash_power;
		if ( stop_dash_power < 0.0 )
		{
			stop_stamina *= -2.0;
		}
	
		if ( save_recovery )
		{
			if ( stop_stamina > my_stamina - recover_dec_thr )
			{
			stop_stamina = std::max( 0.0, my_stamina - recover_dec_thr );
			}
		}
		else if ( stop_stamina > my_stamina )
		{
			stop_stamina = my_stamina;
		}
	
		if ( stop_dash_power < 0.0 )
		{
			dash_power = -stop_stamina * 0.5;
		}
		else
		{
			dash_power = stop_stamina;
		}
	
		stop_accel.x = dash_power * self_type.dashPowerRate() * my_effort;
		vel += stop_accel;
		my_speed = vel.r();
		}
	#endif
	
		AngleDeg dash_angle = wm.self().body();
		if ( n_turn > 0 )
		{
		angle_diff = std::max( 0.0, angle_diff );
		dash_angle = target_angle;
		if ( ( target_angle - wm.self().body() ).degree() > 0.0 )
		{
			dash_angle += angle_diff;
		}
		else
		{
			dash_angle += angle_diff;
		}
	
		self_type.predictStaminaAfterWait( param,
						n_turn,
						&my_stamina,
						&my_effort,
						my_recovery );
		}
	
		//
		// dash
		//
	
		Vector2D vel = wm.self().vel() * std::pow( self_type.playerDecay(), n_turn );
		while ( n_turn + n_dash < cycle
			&& target_dist > dist_thr )
		{
		double dash_power = std::min( param.maxPower(), my_stamina );
		if ( save_recovery
			&& my_stamina - dash_power < recover_dec_thr )
		{
			dash_power = std::max( 0.0, my_stamina - recover_dec_thr );
		}
	
		Vector2D accel = Vector2D::polar2vector( dash_power
							* self_type.dashPowerRate()
							* my_effort,
							dash_angle );
		vel += accel;
		double speed = vel.r();
		if ( speed > self_type.playerSpeedMax() )
		{
			vel *= self_type.playerSpeedMax() / speed;
		}
	
		my_pos += vel;
		vel *= self_type.playerDecay();
	
		self_type.predictStaminaAfterOneDash( param,
							param.maxPower(),
							&my_stamina,
							&my_effort,
							&my_recovery );
	
		target_dist = my_pos.dist( target_point );
		++n_dash;
		}
		if ( target_dist <= dist_thr )
		{
		if ( stamina )
		{
			*stamina = my_stamina;
		}

		return n_turn + n_dash;
		}
	}
	
	return 1000;
}
