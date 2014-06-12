// -*-c++-*-

/*
 *Copyright:

 Copyright (C) Hidehisa AKIYAMA

 This code is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 3, or (at your option)
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

/////////////////////////////////////////////////////////////////////
#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "role_goalie.h"
#include "zh2012/bhv_ZhPass.h"
//#include "zh2012/bhv_goalie_one_to_one.h"

#include "bhv_basic_tackle.h"
#include "bhv_goalie_basic_move.h"
#include "bhv_goalie_chase_ball.h"
#include "bhv_goalie_free_kick.h"

#include <rcsc/action/basic_actions.h>
#include <rcsc/action/neck_scan_field.h>
#include <rcsc/action/neck_turn_to_low_conf_teammate.h>
#include <rcsc/action/body_clear_ball.h>
#include <rcsc/action/body_dribble.h>

#include <rcsc/player/player_agent.h>
#include <rcsc/player/debug_client.h>
#include <rcsc/player/world_model.h>

#include <rcsc/common/logger.h>
#include <rcsc/common/server_param.h>
#include <rcsc/geom/rect_2d.h>

using namespace rcsc;

const std::string RoleGoalie::NAME("Goalie");

/*-------------------------------------------------------------------*/
/*!

 */
namespace {
rcss::RegHolder role = SoccerRole::creators().autoReg(&RoleGoalie::create,
		RoleGoalie::name());
}

/*-------------------------------------------------------------------*/
/*!

 */
bool RoleGoalie::execute(PlayerAgent * agent) {
	const WorldModel & wm = agent->world();

	static const Rect2D our_penalty(
			Vector2D(-ServerParam::i().pitchHalfLength(),
					-ServerParam::i().penaltyAreaHalfWidth() + 1.0),
			Size2D(ServerParam::i().penaltyAreaLength() - 1.0,
					ServerParam::i().penaltyAreaWidth() - 2.0));

	const PlayerPtrCont & opps = wm.opponentsFromSelf();
	const PlayerObject * nearest_opp = (
			opps.empty() ? static_cast<PlayerObject *>(0) : opps.front());
	const double nearest_opp_dist = (
			nearest_opp ? nearest_opp->distFromSelf() : 1000.0);
	const Vector2D nearest_opp_pos = (
			nearest_opp ? nearest_opp->pos() : Vector2D(-1000.0, 0.0));

	const rcsc::PlayerType * player_type = wm.self().playerTypePtr();
	float kickableArea = player_type->kickableArea();

	static bool kicking = false;

	//////////////////////////////////////////////////////////////
	// play_on play

	static bool isDanger = false;
	static int dangerCycle = 0;

	rcsc::Vector2D ball = wm.ball().pos();
//     rcsc::Vector2D prevVel = (wm.,ball().vel() * 100.0) / 94.0;
	rcsc::Vector2D prevBall = ball + wm.ball().rposPrev();
	rcsc::Vector2D nextBall = ball + wm.ball().vel();

	const rcsc::PlayerPtrCont::const_iterator t_end =
			wm.teammatesFromBall().end();
	for (rcsc::PlayerPtrCont::const_iterator t = wm.teammatesFromBall().begin();
			t != t_end; ++t) {
		const rcsc::PlayerType * player_type = (*t)->playerTypePtr();
		float kickable = player_type->kickableArea();

		rcsc::Vector2D tmm = (*t)->pos();

		if ((*t)->posCount() > 3)
			continue;

		if (prevBall.dist(tmm) < kickable + 0.4) {
			isDanger = true;
			dangerCycle = wm.time().cycle();
			break;
		}
	}

	if (isDanger) {
		if (wm.gameMode().type() != rcsc::GameMode::PlayOn)
			isDanger = false;

		if (wm.time().cycle() - dangerCycle > 33)
			isDanger = false;

		if (wm.existKickableOpponent() || wm.existKickableTeammate())
			isDanger = false;

//         if( ball.x > -10.0 )
//            isDanger = false;

//         if( wm.ball().inertiaPoint(2).x < -52.5 && wm.ball().inertiaPoint(2).absY() < 8.0 &&
//             nextBall.dist(wm.self().pos()) > kickableArea &&
//             ball.dist(wm.self().pos()) > kickableArea )
//         {
//             if( Bhv_BasicTackle( 0.3, 80.0 ).execute( agent ) )
//                return true;
//             else
//                isDanger = false;
//         }
	}

	// catchable
	if (agent->world().time().cycle()
			> agent->world().self().catchTime().cycle()
					+ ServerParam::i().catchBanCycle()
			&& agent->world().ball().distFromSelf()
					< ServerParam::i().catchableArea() - 0.05
			&& our_penalty.contains(agent->world().ball().pos())) {
		/*      if( ((!wm.self().isKickable() || wm.ball().vel().absX() > 1.7 || wm.ball().vel().r() > 2.0) &&
		 nearest_opp_dist > 11 ) ||
		 (nearest_opp_dist < 11 && wm.ball().vel().r() > 1.5 ) ||
		 wm.countOpponentsIn(our_penalty, 5, true) >= 2 ||  wm.countTeammatesIn(our_penalty, 5, true) >= 3 )*/

		Vector2D nearestTmmPos = wm.teammatesFromSelf().front()->pos();

//      if( isDanger )
//      {
//        std::cout<<"\nCycle: "<<wm.time().cycle()<<" isDanger == true **********************************\n";
//      }

		if (our_penalty.contains(agent->world().ball().pos())
				&& (nearestTmmPos.dist(wm.ball().pos()) > 2.5
						|| wm.existKickableOpponent()) && !kicking
				&& (!isDanger
						|| (wm.ball().vel().r() > 2.5
								&& ball.dist(wm.self().pos()) > 0.45))) {
			kicking = false;
			isDanger = false;
			agent->doCatch();
			agent->setNeckAction(new Neck_TurnToBall());
		} else {
			kicking = true;
			isDanger = false;
			doKick(agent);
		}
	} else if (wm.self().isKickable()) {
		kicking = true;
		isDanger = false;
		doKick(agent);
	} else {
		kicking = false;
		doMove(agent);
	}


	return true;
}

/*-------------------------------------------------------------------*/
/*!

 */
void RoleGoalie::doKick(PlayerAgent * agent) {
	const WorldModel & wm = agent->world();

	const PlayerPtrCont & opps = wm.opponentsFromSelf();
	const PlayerObject * nearest_opp = (
			opps.empty() ? static_cast<PlayerObject *>(0) : opps.front());
	const double nearest_opp_dist = (
			nearest_opp ? nearest_opp->distFromSelf() : 1000.0);
	const Vector2D nearest_opp_pos = (
			nearest_opp ? nearest_opp->pos() : Vector2D(-1000.0, 0.0));

	Vector2D dribbleTarget = Vector2D(0, sign(wm.ball().pos().y) * 20.0);

	Vector2D nearestTmmPos = wm.teammatesFromSelf().front()->pos();

	if (nearestTmmPos.dist(wm.ball().pos()) < 2.5) {
		Body_ClearBall().execute(agent);
		agent->setNeckAction(new Neck_ScanField());
	} else if (nearest_opp_dist > 9.0
			&& wm.self().stamina() > ServerParam::i().staminaMax() * 0.62) {
		Body_Dribble(dribbleTarget, 1.0, ServerParam::i().maxDashPower(), 2).execute(
				agent);
		agent->setNeckAction(new Neck_TurnToLowConfTeammate());
	} else if (Bhv_ZhPass().execute(agent))
		agent->setNeckAction(new Neck_TurnToLowConfTeammate());
	else {
		Body_ClearBall().execute(agent);
		agent->setNeckAction(new rcsc::Neck_ScanField());
	}
}

/*-------------------------------------------------------------------*/
/*!

 */
void RoleGoalie::doMove(PlayerAgent * agent) {
	if (Bhv_BasicTackle(0.5, 80.0).execute(agent)) {
		return;
	} else if (Bhv_GoalieChaseBall::is_ball_chase_situation(agent)) {
		Bhv_GoalieChaseBall().execute(agent);
	} else {
		Bhv_GoalieBasicMove().execute(agent);
	}
}
