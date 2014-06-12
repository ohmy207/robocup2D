// -*-c++-*-

/*
 *Copyright (C) TJNU
 */

/////////////////////////////////////////////////////////////////////

#ifndef BODY_CLEAR_BALL_2008_H
#define BODY_CLEAR_BALL_2008_H

#include <rcsc/player/soccer_action.h>

#include "intercept_simulator.h"
#include "player_evaluator.h"

#include <rcsc/action/body_smart_kick.h>
#include <rcsc/action/body_kick_one_step.h>

#include <rcsc/player/player_agent.h>
#include <rcsc/player/player_predicate.h>
#include <rcsc/common/logger.h>
#include <rcsc/common/server_param.h>
#include <rcsc/math_util.h>
#include <rcsc/timer.h>

#include <cmath>
#include <limits>

namespace {

inline
double
AbstractPlayerCont_getMinimumEvaluation( const rcsc::AbstractPlayerCont & container,
                                         const PlayerEvaluator * evaluator )
{
    //double min_value = +DBL_MAX;
    double min_value = std::numeric_limits< double >::max();

    const rcsc::AbstractPlayerCont::const_iterator p_end = container.end();
    for ( rcsc::AbstractPlayerCont::const_iterator it = container.begin();
          it != p_end;
          ++it )
    {
        double value = (*evaluator)( **it );

        if ( value < min_value )
        {
            min_value = value;
        }
    }

    delete evaluator;
    return min_value;
}

}

rcsc::AngleDeg
getClearCourse( const rcsc::WorldModel & wm,
                double safe_angle,
                int recursive_count );
//std::vector< ClearCourse > * courses );


double
getFreeAngle( const rcsc::WorldModel & wm, const rcsc::AngleDeg & angle );


class Body_ClearBall2008
    : public rcsc::BodyAction {

public:
    /*!
      \brief constructor
     */
    Body_ClearBall2008()
    {
    }

    /*!
      \brief do clear ball
      \param agent agent pointer to the agent itself
      \return true with action, false if can't do clear
     */
    bool execute( rcsc::PlayerAgent * agent )
    {
	// TODO:
	// . if result free cycles will be same steps, choice by free angle width
	// . check tackle
	
	const rcsc::WorldModel & wm = agent->world();
	const rcsc::ServerParam & param = rcsc::ServerParam::i();
	
	
	// enforce kick
	if ( wm.existKickableOpponent() )
	{
		agent->debugClient().addMessage( "ClearEnforce" );
		rcsc::dlog.addText( rcsc::Logger::CLEAR,
				__FILE__": exist kickable opponent" );
		return rcsc::Body_KickOneStep( wm.ball().pos() + rcsc::Vector2D( 10.0, 0.0 ),
					param.ballSpeedMax() ).execute( agent );
	}
	
	rcsc::MSecTimer timer;
	rcsc::AngleDeg clear_angle = getClearCourse( wm,
							15.0, /* safe angle */
							3 /* recursive count */ );
	rcsc::dlog.addText( rcsc::Logger::CLEAR,
				__FILE__": getClearCourse() elapsed %.3f [ms]",
				timer.elapsedReal() );
	
	const rcsc::Vector2D kick_target = wm.ball().pos()
		+ rcsc::Vector2D::polar2vector( 30.0, clear_angle );
	
	agent->debugClient().addLine( wm.ball().pos(), kick_target );
	
	if ( wm.gameMode().type() != rcsc::GameMode::PlayOn
		|| AbstractPlayerCont_getMinimumEvaluation( wm.getPlayerCont( new rcsc::OpponentOrUnknownPlayerPredicate( wm ) ),
							new DistFromPosPlayerEvaluator( wm.ball().pos() ) ) < 3.0 )
	{
		agent->debugClient().addMessage( "Clear1" );
		rcsc::dlog.addText( rcsc::Logger::CLEAR,
				__FILE__": Clear 1 step kicl. target=(%.1f %.1f)",
				kick_target.x, kick_target.y );
		return rcsc::Body_KickOneStep( kick_target,
					param.ballSpeedMax() ).execute( agent );
	}
	else
	{
		agent->debugClient().addMessage( "ClearS" );
		rcsc::dlog.addText( rcsc::Logger::CLEAR,
				__FILE__": Clear 1 step kicl. target=(%.1f %.1f)",
				kick_target.x, kick_target.y );
		return rcsc::Body_SmartKick( kick_target,
					param.ballSpeedMax(),
					std::max( 2.5, param.ballSpeedMax() * 0.85 ),
					2 ).execute( agent );
	}
    }

static
double
getFreeAngle( const rcsc::WorldModel & wm,
              const rcsc::AngleDeg & angle )
{
    return std::min( AbstractPlayerCont_getMinimumEvaluation
                     ( wm.getPlayerCont( new rcsc::OpponentOrUnknownPlayerPredicate( wm ) ),
                       new AbsAngleDiffPlayerEvaluator( wm.ball().pos(), angle ) ),
                     360.0 );
}


static
rcsc::AngleDeg
getClearCourse( const rcsc::WorldModel & wm,
                double safe_angle,
                int recursive_count )

{
    const rcsc::ServerParam & param = rcsc::ServerParam::i();

    const double sign = rcsc::sign( wm.ball().pos().y );

    rcsc::Vector2D clear_point;
    rcsc::AngleDeg clear_dir;

    for ( double x = 50.0; x > 11.0; x -= 10.0 )
    {
        clear_point.assign( wm.self().pos().x + x,
                            sign * ( param.pitchHalfWidth() - 2.0 ) );

        if ( wm.self().pos().dist2( clear_point ) >= 10.0 * 10.0 )
        {
            clear_dir = ( clear_point - wm.ball().pos() ).th();

            if ( getFreeAngle( wm, clear_dir ) >= safe_angle )
            {
                rcsc::dlog.addText( rcsc::Logger::CLEAR,
                                    __FILE__": Clear recursive %d safe_angle=%.1f point=(%.1f %.1f).angle=%.1f",
                                    recursive_count,
                                    safe_angle,
                                    clear_point.x, clear_point.y,
                                    clear_dir.degree() );
                return clear_dir;
            }
        }
    }

    // search from 30 degree to 75 degree
    for ( double dir = 30.0; dir <= 75.0 + rcsc::EPS; dir += 1.0 )
    {
        if ( getFreeAngle( wm, sign * dir ) >= safe_angle )
        {
            rcsc::dlog.addText( rcsc::Logger::CLEAR,
                                __FILE__": Clear recursive %d safe_angle=%.1f angle=%.1f",
                                recursive_count,
                                safe_angle,
                                sign * sign );
            return sign * dir;
        }
    }

    // search from 30 degree to -30 degree
    for ( double dir = 30.0; dir >= -30.0 - rcsc::EPS; dir -= 1.0 )
    {
        if ( getFreeAngle( wm, sign * dir ) >= safe_angle )
        {
            rcsc::dlog.addText( rcsc::Logger::CLEAR,
                                __FILE__": Clear recursive %d safe_angle=%.1f angle=%.1f",
                                recursive_count,
                                safe_angle,
                                sign * sign );
            return sign * dir;
        }
    }

    if ( recursive_count > 0 )
    {
        rcsc::dlog.addText( rcsc::Logger::CLEAR,
                            __FILE__": recursive: %d",
                            recursive_count );

        return getClearCourse( wm,
                               safe_angle * 0.7,
                               recursive_count - 1 );
    }
    else
    {
        rcsc::AngleDeg goal_away = (wm.ball().pos()
                                    - param.ourTeamGoalPos()).th();

        if ( std::fabs( wm.ball().pos().y + goal_away.sin() * 30.0 )
             < param.pitchHalfWidth() )
        {
            // goal away
            rcsc::dlog.addText( rcsc::Logger::CLEAR,
                                __FILE__": goal_away" );
            return goal_away;
        }
        else
        {
            // beside line
            rcsc::dlog.addText( rcsc::Logger::CLEAR,
                                __FILE__": beside line" );

            if ( wm.self().pos().absY() <= 25.0 )
            {
                return + sign * 3.0;
            }
            else
            {
                return - sign * 3.0;
            }
        }
    }

    return 0.0;
}


};

#endif
