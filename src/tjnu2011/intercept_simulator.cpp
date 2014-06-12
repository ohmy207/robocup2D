// -*-c++-*-

/*
 *Copyright (C) TJNU
 */

/////////////////////////////////////////////////////////////////////

#include "intercept_simulator.h"

long InterceptSimulator::getAccessStepToBallPredictPoint( const rcsc::WorldModel & wm,
                                                          const rcsc::Vector2D & ball_pos,
                                                          const rcsc::Vector2D & ball_vel,
                                                          const long ball_vel_change_step,
                                                          const rcsc::AbstractPlayerObject & pl,
                                                          const long step ) const
{
	assert( step >= 0 );
	assert( pl.playerTypePtr() != static_cast< rcsc::PlayerType * >( 0 ) );
	
	const rcsc::Vector2D & current_ball_pos = wm.ball().pos();
	
	const double ball_controllable_distance
		= pl.playerTypePtr()->kickableArea();
	
	rcsc::Vector2D ball_predict_point;
	
	if ( ball_vel_change_step != 0
		&& step <= ball_vel_change_step )
	{
		// XXX
		ball_predict_point = current_ball_pos
		+ ( ball_pos - current_ball_pos )
		* ( static_cast< double >( step )
			/ static_cast< double >( ball_vel_change_step ) );
	}
	else
	{
		ball_predict_point = inertia_n_step_point( ball_pos,
							ball_vel,
							step - ball_vel_change_step,
							rcsc::ServerParam::i().ballDecay() );
	}
	
	rcsc::Vector2D rel = ( ball_predict_point
				- pl.playerTypePtr()->inertiaPoint( pl.pos(), pl.vel(), step ) );
	rel.rotate( - pl.body() );
	
	long spend_time = 0;
	
	#if 0
	double diff_angle = rel.th().abs();
	#endif
	double cone_angle = std::fabs( rcsc::AngleDeg::atan2_deg
					( ball_controllable_distance,
					rel.r() ) );
	
	if ( cone_angle < 2.0 )
	{
		cone_angle = 2.0;
	}
	
	#if 0
	if ( pl.bodyCount() == 0 )
	{
		if ( diff_angle >= cone_angle
		&& rel.r() > ball_controllable_distance )
		{
		spend_time += pl.playerType().n_step_to_turn_body
			( diff_angle, pl.velocity().r() );
		}
	}
	else
	#endif
	{
		if ( pl.side() == wm.self().side() )
		{
		spend_time += 2;
		}
		else
		{
		spend_time += 0;
		}
	}
	
	
	double dist = rel.r() - ball_controllable_distance;
	
	if ( dist > 0.0 )
	{
		spend_time += pl.playerTypePtr()->cyclesToReachDistance( dist );
	}
	
	if ( pl.side() != wm.self().side() )
	{
		spend_time += this->getOpponentPenaltyTime( pl.posCount() );
	
		if ( spend_time < 0 )
		{
		spend_time = 0;
		}
	}
	
	if ( pl.side() == wm.self().side() )
	{
		spend_time += this->getTeammatePenaltyTime( pl.posCount() );
	}
	
	
	#ifdef SIMULATOR_DEBUG_PRINT
	if ( pl.side() == wm.self().side() )
	{
		rcsc::dlog.addText( rcsc::Logger::TEAM,
				"step = %ld, teammate %d, spend_time %ld",
				step, pl.unum(), spend_time );
	}
	else if ( pl.side() != rcsc::NEUTRAL )
	{
		rcsc::dlog.addText( rcsc::Logger::TEAM,
				"step = %ld, opponent %d, spend_time %ld",
				step, pl.unum(), spend_time );
	}
	else
	{
		rcsc::dlog.addText( rcsc::Logger::TEAM,
				"step = %ld, unknown, spend_time %ld",
				step, spend_time );
	}
	#endif
	
	return spend_time;
}

long InterceptSimulator::playerBallAccessStep( const rcsc::WorldModel & wm,
                                               const rcsc::Vector2D & ball_pos,
                                               const rcsc::Vector2D & ball_vel,
                                               const long ball_vel_change_step,
                                               const long penalty_step,
                                               const rcsc::AbstractPlayerObject & pl,
                                               const long max_step ) const
{
	const long STRICT_CALCULATE_STEP_THRESHOLD = 5;
	
	// XXX: consider catching
	const double ball_controllable_distance
		= pl.playerTypePtr()->kickableArea();
	
	// XXX
	const double player_speed_max = pl.playerTypePtr()->realSpeedMax() * 0.9;
	
	const rcsc::Vector2D relative_ball_pos = wm.ball().pos() - pl.pos();
	
	//
	// if can't access in max_step, return max_step + 1
	//
	if ( max_step != -1
		&& relative_ball_pos.r() > ( ( player_speed_max + rcsc::ServerParam::i().ballSpeedMax() )
					* ( max_step - penalty_step )
					+ ball_controllable_distance )
		)
	{
		return max_step + 1;
	}
	
	
	long t = 0;
	
	//
	// for few steps, calculate strictly
	//
	for ( t = 0;
		( t <= max_step || max_step == -1 )
		&& ( t <= STRICT_CALCULATE_STEP_THRESHOLD
			|| penalty_step != 0 );
		++t )
	{
		if ( getAccessStepToBallPredictPoint( wm,
						ball_pos, ball_vel,
						ball_vel_change_step,
						pl, t )
		+ penalty_step <= t )
		{
		return t;
		}
	}
	
	if ( penalty_step != 0 )
	{
		if ( max_step == -1 )
		{
		return VERY_LONG_STEPS + 1;
		}
		else
		{
		return max_step + 1;
		}
	}
	
	
	//
	// calculate step count or set t_min & max
	//
	long t_min = STRICT_CALCULATE_STEP_THRESHOLD + 1;
	long t_max = 200;
	
	// if ball is getting far away
	if ( ( ball_vel.th() - ( pl.pos() - ball_pos ).th() ).abs() >= 90.0 )
	{
		t_min = std::max( static_cast< long >( ( relative_ball_pos.r() - ball_controllable_distance )
						/ player_speed_max
						- 1 ),
				STRICT_CALCULATE_STEP_THRESHOLD + 1 );
		// XXX
		t_max = std::max( static_cast< long >( 200 ), t_min );
	}
	else
	{
		const rcsc::Line2D ball_line( ball_pos, ball_vel.th() );
	
		const double ball_project_point_dist
		= ( ball_pos - ball_line.projection( pl.pos() ) ).r();
	
		const double project_ball_reach_step
		= get_ball_travel_step( ball_project_point_dist, ball_vel.r() );
	
		if ( project_ball_reach_step > static_cast< double >( LONG_MAX - 1 ) )
		{
		for ( ;
			( max_step == -1 ) || ( t <= max_step );
			++t )
		{
			if ( getAccessStepToBallPredictPoint( wm,
							ball_pos, ball_vel,
							ball_vel_change_step,
							pl, t )
			<= t )
			{
			return t;
			}
		}
	
		return t;
		}
	
		// if ball can't reach to project point
		if ( project_ball_reach_step < 0.0 )
		{
		t_min = std::max( static_cast< long >( ( wm.ball().pos().dist( pl.pos() )
							- ball_controllable_distance )
							/ ( player_speed_max + rcsc::ServerParam::i().ballSpeedMax() )
							- 1 ),
				STRICT_CALCULATE_STEP_THRESHOLD + 1 );
		// XXX
		t_max = std::max( static_cast< long >( 200 ), t_min );
		}
		else
		{
		long proj_ball_reach_step_min = static_cast< long >( project_ball_reach_step );
		long proj_ball_reach_step_max = proj_ball_reach_step_min + 1;
	
		long proj_access_step_min = getAccessStepToBallPredictPoint( wm,
										ball_pos,
										ball_vel,
										ball_vel_change_step,
										pl,
										proj_ball_reach_step_min );
	
		long proj_access_step_max = getAccessStepToBallPredictPoint( wm,
										ball_pos,
										ball_vel,
										ball_vel_change_step,
										pl,
										proj_ball_reach_step_max );
	
		if ( proj_access_step_min <= proj_ball_reach_step_min )
		{
			t_min = STRICT_CALCULATE_STEP_THRESHOLD + 1;
			t_max = std::max( proj_access_step_min, t_min );
		}
		else
		{
			if ( proj_access_step_max <= proj_ball_reach_step_max )
			{
			return proj_ball_reach_step_max;
			}
			else
			{
			t_min = std::max( proj_ball_reach_step_max + 1,
					STRICT_CALCULATE_STEP_THRESHOLD + 1 );
	
			if ( max_step == -1 )
			{
				t_max = proj_ball_reach_step_max + 50;
	
				while ( getAccessStepToBallPredictPoint( wm,
									ball_pos, ball_vel,
									ball_vel_change_step,
									pl,
									t_max )
					> t_max )
				{
				t_max += 50;
				}
			}
			else
			{
				t_max = max_step + 1;
			}
	
			t_max = std::max( STRICT_CALCULATE_STEP_THRESHOLD + 1,
					t_max );
	
			t_max = std::max( t_max, t_min );
			}
		}
		}
	}
	
	
	assert( t_min >= STRICT_CALCULATE_STEP_THRESHOLD + 1 );
	assert( t_max >= t_min );
	
	for ( ; ; )
	{
		long medium = ( t_min + t_max ) / 2;
	
		if ( medium == t_min )
		{
		if ( getAccessStepToBallPredictPoint( wm,
							ball_pos, ball_vel,
							ball_vel_change_step,
							pl, medium )
			<= medium )
		{
			return t_min;
		}
		else
		{
			return t_max;
		}
		}
	
		if ( getAccessStepToBallPredictPoint( wm,
						ball_pos, ball_vel,
						ball_vel_change_step,
						pl, medium )
		<= medium )
		{
		t_max = medium;
		}
		else
		{
		t_min = medium;
		}
	}
}

void InterceptSimulator::simulate( const rcsc::WorldModel & wm,
                                   const rcsc::AbstractPlayerCont & players,
                                   const rcsc::Vector2D & ball_pos,
                                   const rcsc::Vector2D & ball_vel,
                                   const long ball_vel_change_step,
                                   const long self_penalty_step,
                                   const long teammate_penalty_step,
                                   const long opponent_or_unknown_penalty_step,
                                   long * self_access_step,
                                   long * teammate_access_step,
                                   long * opponent_access_step,
                                   long * unknown_access_step,
                                   int * first_access_teammate,
                                   int * first_access_opponent,
                                   long max_step ) const
     {
	#ifdef SIMULATOR_DEBUG_PRINT
	rcsc::dlog.addText( rcsc::Logger::TEAM,
				__FILE__ ": simulate()" );
	
	rcsc::dlog.addText( rcsc::Logger::TEAM,
				"ball pos = [%lf,%lf]",
				ball_pos.x, ball_pos.y );
	
	rcsc::dlog.addText( rcsc::Logger::TEAM,
				"ball vel(x,y) = [%lf,%lf], vel(r,th) = [%lf,%lf]",
				ball_vel.x, ball_vel.y,
				ball_vel.r(), ball_vel.th().degree() );
	
	rcsc::dlog.addText( rcsc::Logger::TEAM,
				"ball vel change step = %ld",
				ball_vel_change_step );
	
	rcsc::dlog.addText( rcsc::Logger::TEAM,
				"self penalty step = %ld",
				self_penalty_step );
	#endif
	
	if ( max_step < 0 )
	{
		max_step = VERY_LONG_STEPS;
	}
	
	if ( self_access_step )
	{
		*self_access_step = max_step + 1;
	}
	
	if ( teammate_access_step )
	{
		*teammate_access_step = max_step + 1;
	}
	
	if ( opponent_access_step )
	{
		*opponent_access_step = max_step + 1;
	}
	
	if ( unknown_access_step )
	{
		*unknown_access_step = max_step + 1;
	}
	
	if ( first_access_teammate )
	{
		*first_access_teammate = rcsc::Unum_Unknown;
	}
	
	if ( first_access_opponent )
	{
		*first_access_opponent = rcsc::Unum_Unknown;
	}
	
	
	const rcsc::AbstractPlayerCont::const_iterator p_end = players.end();
	for ( rcsc::AbstractPlayerCont::const_iterator it = players.begin();
		it != p_end;
		++it )
	{
		if ( (**it).isSelf() )
		{
		if ( ! self_access_step
			|| rcsc::OffsidePositionPlayerPredicate( wm )( **it ) )
		{
			continue;
		}
	
		*self_access_step = playerBallAccessStep( wm,
							ball_pos,
							ball_vel,
							ball_vel_change_step,
							self_penalty_step,
							**it,
							max_step );
	
	#ifdef SIMULATOR_DEBUG_PRINT
		rcsc::dlog.addText( rcsc::Logger::TEAM,
					"self = %ld", *self_access_step );
	#endif
		}
		else if ( (**it).side() == wm.self().side() )
		{
		if ( ! teammate_access_step
			|| *teammate_access_step == 0
			|| rcsc::OffsidePositionPlayerPredicate( wm )( **it ) )
		{
			continue;
		}
	
		long step = playerBallAccessStep( wm,
						ball_pos,
						ball_vel,
						ball_vel_change_step,
						teammate_penalty_step,
						**it,
						*teammate_access_step - 1 );
	
	#ifdef SIMULATOR_DEBUG_PRINT
		rcsc::dlog.addText( rcsc::Logger::TEAM,
					"teammate %d = %ld (max %ld)",
					(**it).unum(),
					step, *teammate_access_step - 1 );
	#endif
	
		if ( step < *teammate_access_step )
		{
			*teammate_access_step = step;
			if ( first_access_teammate )
			{
			*first_access_teammate = (**it).unum();
			}
		}
		}
		else if ( (**it).side() != wm.self().side()
			&& (**it).side() != rcsc::NEUTRAL )
		{
		if ( ! opponent_access_step
			|| *opponent_access_step == 0
			|| rcsc::OffsidePositionPlayerPredicate(wm)(**it) )
		{
			continue;
		}
	
		long step = playerBallAccessStep( wm,
						ball_pos,
						ball_vel,
						ball_vel_change_step,
						opponent_or_unknown_penalty_step,
						**it,
						*opponent_access_step - 1 );
	
	#ifdef SIMULATOR_DEBUG_PRINT
		rcsc::dlog.addText( rcsc::Logger::TEAM,
					"opponent %d = %ld (max %ld)",
					(**it).unum(),
					step, *opponent_access_step - 1 );
	#endif
	
		if ( *opponent_access_step == -1
			|| step < *opponent_access_step )
		{
			*opponent_access_step = step;
	
			if ( first_access_opponent )
			{
			*first_access_opponent = (**it).unum();
			}
		}
		}
		else // side == UNKNOWN
		{
		if ( ! unknown_access_step
			|| *unknown_access_step == 0 )
		{
			continue;
		}
	
		long step = playerBallAccessStep( wm,
						ball_pos,
						ball_vel,
						ball_vel_change_step,
						opponent_or_unknown_penalty_step,
						**it,
						*unknown_access_step - 1 );
	
	#ifdef SIMULATOR_DEBUG_PRINT
		rcsc::dlog.addText( rcsc::Logger::TEAM,
					"unknown %d = %ld (max %ld)",
					(**it).unum(),
					step, *unknown_access_step - 1 );
	#endif
	
		if ( step < *unknown_access_step )
		{
			*unknown_access_step = step;
		}
		}
	}
}
