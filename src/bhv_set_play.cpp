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

#include "bhv_set_play.h"
#include "zh2012/neck_for_mark.h"

#include "strategy.h"

#include "bhv_goalie_free_kick.h"
#include "bhv_set_play_free_kick.h"
#include "bhv_set_play_goal_kick.h"
#include "bhv_set_play_kick_off.h"
#include "bhv_set_play_kick_in.h"
#include "bhv_set_play_indirect_free_kick.h"
#include "bhv_their_goal_kick_move.h"

#include <rcsc/action/basic_actions.h>
#include <rcsc/action/bhv_before_kick_off.h>
#include <rcsc/action/bhv_scan_field.h>
#include <rcsc/action/body_go_to_point.h>
#include <rcsc/action/neck_scan_field.h>
#include <rcsc/action/neck_turn_to_ball_or_scan.h>

#include <rcsc/player/player_agent.h>
#include <rcsc/player/debug_client.h>

#include <rcsc/common/audio_memory.h>
#include <rcsc/common/logger.h>
#include <rcsc/common/server_param.h>
#include <rcsc/geom/circle_2d.h>
#include <rcsc/geom/line_2d.h>
#include <rcsc/geom/segment_2d.h>

#include <vector>
#include <algorithm>
#include <limits>
#include <cstdio>

// #define DEBUG_PRINT

using namespace rcsc;

/*-------------------------------------------------------------------*/
/*!

 */
bool Bhv_SetPlay::execute(PlayerAgent * agent) {
	dlog.addText(Logger::TEAM, __FILE__": Bhv_SetPlay");

	const WorldModel & wm = agent->world();

#ifdef DEBUG_PRINT
	Vector2D home_pos = Strategy::i().getPosition( wm.self().unum() );
	if ( ! home_pos.isValid() )
	{
		std::cerr << agent->config().teamName() << ": "
		<< wm.self().unum()
		<< " ***ERROR*** illegal home position."
		<< std::endl;
		home_pos.assign( 0.0, 0.0 );
	}

	dlog.addCircle( Logger::TEAM,
			home_pos, 0.5, "#0000ff", true );
#endif

	if (wm.self().goalie()) {
		if (wm.gameMode().type() != GameMode::BackPass_
				&& wm.gameMode().type() != GameMode::IndFreeKick_) {
			Bhv_GoalieFreeKick().execute(agent);
		} else {
			Bhv_SetPlayIndirectFreeKick().execute(agent);
		}

		return true;
	}

	switch (wm.gameMode().type()) {
	case GameMode::KickOff_:
		if (wm.gameMode().side() == wm.ourSide()) {
			return Bhv_SetPlayKickOff().execute(agent);
		} else {
			doBasicTheirSetPlayMove(agent);
			return true;
		}
		break;
	case GameMode::KickIn_:
	case GameMode::CornerKick_:
		if (wm.gameMode().side() == wm.ourSide()) {
			return Bhv_SetPlayKickIn().execute(agent);
		} else {
			doBasicTheirSetPlayMove(agent);
			return true;
		}
		break;
	case GameMode::GoalKick_:
		if (wm.gameMode().side() == wm.ourSide()) {
			return Bhv_SetPlayGoalKick().execute(agent);
		} else {
			return Bhv_TheirGoalKickMove().execute(agent);
		}
		break;
	case GameMode::BackPass_:
	case GameMode::IndFreeKick_:
		return Bhv_SetPlayIndirectFreeKick().execute(agent);
		break;
	case GameMode::FoulCharge_:
	case GameMode::FoulPush_:
		if (wm.ball().pos().x < ServerParam::i().ourPenaltyAreaLineX() + 1.0
				&& wm.ball().pos().absY()
						< ServerParam::i().penaltyAreaHalfWidth() + 1.0) {
			return Bhv_SetPlayIndirectFreeKick().execute(agent);
		} else if (wm.ball().pos().x
				> ServerParam::i().theirPenaltyAreaLineX() - 1.0
				&& wm.ball().pos().absY()
						< ServerParam::i().penaltyAreaHalfWidth() + 1.0) {
			return Bhv_SetPlayIndirectFreeKick().execute(agent);
		}
		break;
#if 0
		case GameMode::FreeKick_:
		case GameMode::CornerKick_:
		case GameMode::GoalieCatch_: // after catch
		case GameMode::Offside_:
		case GameMode::FreeKickFault_:
		case GameMode::CatchFault_:
#endif
	default:
		break;
	}

	if (wm.gameMode().isOurSetPlay(wm.ourSide())) {
		dlog.addText(Logger::TEAM, __FILE__": our set play");
		return Bhv_SetPlayFreeKick().execute(agent);
	} else {
		doBasicTheirSetPlayMove(agent);
		return true;
	}

	return false;
}

/*-------------------------------------------------------------------*/
/*!

 */
double Bhv_SetPlay::get_set_play_dash_power(const PlayerAgent * agent) {
#if 1
	const WorldModel & wm = agent->world();

	if (!wm.gameMode().isOurSetPlay(wm.ourSide())) {
		Vector2D target_point = Strategy::i().getPosition(wm.self().unum());
		if (target_point.x > wm.self().pos().x) {
			if (wm.ball().pos().x < -30.0
					&& target_point.x < wm.ball().pos().x) {
				return wm.self().getSafetyDashPower(
						ServerParam::i().maxDashPower());
			}

			double rate = 0.0;
			if (wm.self().stamina() > ServerParam::i().staminaMax() * 0.8) {
				rate = 1.5 * wm.self().stamina()
						/ ServerParam::i().staminaMax();
			} else {
				rate = 0.9
						* (wm.self().stamina()
								- ServerParam::i().recoverDecThrValue())
						/ ServerParam::i().staminaMax();
				rate = std::max(0.0, rate);
			}

			return (wm.self().playerType().staminaIncMax()
					* wm.self().recovery() * rate);
		}
	}

	return wm.self().getSafetyDashPower(ServerParam::i().maxDashPower());
#else
	if ( agent->world().gameMode().type() == GameMode::PenaltySetup_ )
	{
		return agent->world().self().getSafetyDashPower( ServerParam::i().maxDashPower() );
	}

	double rate;
	if ( agent->world().self().stamina() > ServerParam::i().staminaMax() * 0.8 )
	{
		rate = 1.5
		* agent->world().self().stamina()
		/ ServerParam::i().staminaMax();
	}
	else
	{
		rate = 0.9
		* ( agent->world().self().stamina()
				- ServerParam::i().recoverDecThrValue() )
		/ ServerParam::i().staminaMax();
		rate = std::max( 0.0, rate );
	}

	return ( agent->world().self().playerType().staminaIncMax()
			* agent->world().self().recovery()
			* rate );
#endif
}

namespace {

bool can_go_to(const int count, const WorldModel & wm,
		const Circle2D & ball_circle, const Vector2D & target_point) {
	Segment2D move_line(wm.self().pos(), target_point);

	int n_intersection = ball_circle.intersection(move_line, NULL, NULL);

	dlog.addText(Logger::TEAM,
			"%d: (can_go_to) check target=(%.2f %.2f) intersection=%d", count,
			target_point.x, target_point.y, n_intersection);
	dlog.addLine(Logger::TEAM, wm.self().pos(), target_point, "#0000ff");
	char num[8];
	snprintf(num, 8, "%d", count);
	dlog.addMessage(Logger::TEAM, target_point, num, "#0000ff");

	if (n_intersection == 0) {
		dlog.addText(Logger::TEAM, "%d: (can_go_to) ok(1)", count);
		return true;
	}

	if (n_intersection == 1) {
		AngleDeg angle = (target_point - wm.self().pos()).th();

		dlog.addText(Logger::TEAM,
				"%d: (can_go_to) intersection=1 angle_diff=%.1f", count,
				(angle - wm.ball().angleFromSelf()).abs());
		if ((angle - wm.ball().angleFromSelf()).abs() > 80.0) {
			dlog.addText(Logger::TEAM, "%d: (can_go_to) ok(2)", count);
			return true;
		}
	}

	return false;
}

} // end noname namespace

Vector2D Bhv_SetPlay::get_avoid_circle_point(const WorldModel & wm,
		const Vector2D & target_point) {
	const ServerParam & SP = ServerParam::i();

	const double avoid_radius = SP.centerCircleR()
			+ wm.self().playerType().playerSize();
	const Circle2D ball_circle(wm.ball().pos(), avoid_radius);

#ifdef DEBUG_PRINT
	dlog.addText( Logger::TEAM,
			__FILE__": (get_avoid_circle_point) first_target=(%.2f %.2f)",
			target_point.x, target_point.y );
	dlog.addCircle( Logger::TEAM,
			wm.ball().pos(), avoid_radius,
			"#ffffff" );
#endif

	if (can_go_to(-1, wm, ball_circle, target_point)) {
		dlog.addText(Logger::TEAM,
				__FILE__": (get_avoid_circle_point) ok, first point");
		return target_point;
	}

	AngleDeg target_angle = (target_point - wm.self().pos()).th();
	AngleDeg ball_target_angle = (target_point - wm.ball().pos()).th();
	bool ball_is_left = wm.ball().angleFromSelf().isLeftOf(target_angle);

	const int ANGLE_DIVS = 6;
	std::vector<Vector2D> subtargets;
	subtargets.reserve(ANGLE_DIVS);

	const int angle_step = (ball_is_left ? 1 : -1);

	int count = 0;
	int a = angle_step;

#ifdef DEBUG_PRINT
	dlog.addText( Logger::TEAM,
			__FILE__": (get_avoid_circle_point) ball_target_angle=%.1f angle_step=%d",
			ball_target_angle.degree(), angle_step );
#endif

	for (int i = 1; i < ANGLE_DIVS; ++i, a += angle_step, ++count) {
		AngleDeg angle = ball_target_angle + (180.0 / ANGLE_DIVS) * a;
		Vector2D new_target = wm.ball().pos()
				+ Vector2D::from_polar(avoid_radius + 1.0, angle);

		dlog.addText(Logger::TEAM, "%d: a=%d angle=%.1f (%.2f %.2f)", count, a,
				angle.degree(), new_target.x, new_target.y);

		if (new_target.absX() > SP.pitchHalfLength() + SP.pitchMargin() - 1.0
				|| new_target.absY()
						> SP.pitchHalfWidth() + SP.pitchMargin() - 1.0) {
			dlog.addText(Logger::TEAM, "%d: out of field", count);
			break;
		}

		if (can_go_to(count, wm, ball_circle, new_target)) {
			return new_target;
		}
	}

	a = -angle_step;
	for (int i = 1; i < ANGLE_DIVS * 2; ++i, a -= angle_step, ++count) {
		AngleDeg angle = ball_target_angle + (180.0 / ANGLE_DIVS) * a;
		Vector2D new_target = wm.ball().pos()
				+ Vector2D::from_polar(avoid_radius + 1.0, angle);

#ifdef DEBUG_PRINT
		dlog.addText( Logger::TEAM,
				"%d: a=%d angle=%.1f (%.2f %.2f)",
				count, a, angle.degree(),
				new_target.x, new_target.y );
#endif
		if (new_target.absX() > SP.pitchHalfLength() + SP.pitchMargin() - 1.0
				|| new_target.absY()
						> SP.pitchHalfWidth() + SP.pitchMargin() - 1.0) {
#ifdef DEBUG_PRINT
			dlog.addText( Logger::TEAM,
					"%d: out of field",
					count );
#endif
			break;
		}

		if (can_go_to(count, wm, ball_circle, new_target)) {
			return new_target;
		}
	}

	return target_point;
}

/*-------------------------------------------------------------------*/
/*!

 */
bool Bhv_SetPlay::is_kicker(const PlayerAgent * agent) {
	const WorldModel & wm = agent->world();

	//if ( setplayCount() < 1 )
	//     if ( wm.lastSetPlayStartTime().cycle() > wm.time().cycle() - 2 )
	//     {
	//         return false;
	//     }

	if (wm.gameMode().type() == GameMode::GoalieCatch_
			&& wm.gameMode().side() == wm.ourSide() && !wm.self().goalie()) {
		dlog.addText(Logger::TEAM, __FILE__": (is_kicker) goalie free kick");
		return false;
	}

	int kicker_unum = 0;
	double min_dist2 = std::numeric_limits<double>::max();
	int second_kicker_unum = 0;
	double second_min_dist2 = std::numeric_limits<double>::max();
	for (int unum = 1; unum <= 11; ++unum) {
		if (unum == wm.ourGoalieUnum())
			continue;

		Vector2D home_pos = Strategy::i().getPosition(unum);
		if (!home_pos.isValid())
			continue;

		double d2 = home_pos.dist2(wm.ball().pos());
		if (d2 < second_min_dist2) {
			second_kicker_unum = unum;
			second_min_dist2 = d2;

			if (second_min_dist2 < min_dist2) {
				std::swap(second_kicker_unum, kicker_unum);
				std::swap(second_min_dist2, min_dist2);
			}
		}
	}

	dlog.addText(Logger::TEAM,
			__FILE__": (is_kicker) kicker_unum=%d second_kicker_unum=%d",
			kicker_unum, second_kicker_unum);

	const AbstractPlayerObject * kicker = static_cast<AbstractPlayerObject*>(0);
	const AbstractPlayerObject * second_kicker =
			static_cast<AbstractPlayerObject*>(0);

	if (kicker_unum != 0) {
		kicker = wm.ourPlayer(kicker_unum);
	}

	if (second_kicker_unum != 0) {
		second_kicker = wm.ourPlayer(second_kicker_unum);
	}

	if (!kicker) {
		if (!wm.teammatesFromBall().empty()
				&& wm.teammatesFromBall().front()->distFromBall()
						< wm.ball().distFromSelf() * 0.9) {
			dlog.addText(Logger::TEAM, __FILE__": (is_kicker) first kicker",
					kicker_unum, second_kicker_unum);
			return false;
		}

		dlog.addText(Logger::TEAM, __FILE__": (is_kicker) self(1)");
		return true;
	}

	if (kicker && second_kicker
			&& (kicker->unum() == wm.self().unum()
					|| second_kicker->unum() == wm.self().unum())) {
		if (std::sqrt(min_dist2) < std::sqrt(second_min_dist2) * 0.95) {
			dlog.addText(Logger::TEAM,
					__FILE__": (is_kicker) kicker->unum=%d  (1)",
					kicker->unum());
			return (kicker->unum() == wm.self().unum());
		} else if (kicker->distFromBall()
				< second_kicker->distFromBall() * 0.95) {
			dlog.addText(Logger::TEAM,
					__FILE__": (is_kicker) kicker->unum=%d  (2)",
					kicker->unum());
			return (kicker->unum() == wm.self().unum());
		} else if (second_kicker->distFromBall()
				< kicker->distFromBall() * 0.95) {
			dlog.addText(Logger::TEAM,
					__FILE__": (is_kicker) kicker->unum=%d  (3)",
					kicker->unum());
			return (second_kicker->unum() == wm.self().unum());
		} else if (!wm.teammatesFromBall().empty()
				&& wm.teammatesFromBall().front()->distFromBall()
						< wm.self().distFromBall() * 0.95) {
			dlog.addText(Logger::TEAM, __FILE__": (is_kicker) other kicker",
					kicker->unum());
			return false;
		} else {
			dlog.addText(Logger::TEAM, __FILE__": (is_kicker) self(2)");
			return true;
		}
	}

	dlog.addText(Logger::TEAM, __FILE__": (is_kicker) kicker->unum=%d",
			kicker->unum());

	return (kicker->unum() == wm.self().unum());
}

/*-------------------------------------------------------------------*/
/*!

 */
bool Bhv_SetPlay::is_delaying_tactics_situation(const PlayerAgent * agent) {
	const WorldModel & wm = agent->world();

#if 1
	const int real_set_play_count = wm.time().cycle()
			- wm.lastSetPlayStartTime().cycle();
	const int wait_buf = (wm.gameMode().type() == GameMode::GoalKick_ ? 15 : 2);

	if (real_set_play_count >= ServerParam::i().dropBallTime() - wait_buf) {
		return false;
	}
#endif

	int our_score = (
			wm.ourSide() == LEFT ?
					wm.gameMode().scoreLeft() : wm.gameMode().scoreRight());
	int opp_score = (
			wm.ourSide() == LEFT ?
					wm.gameMode().scoreRight() : wm.gameMode().scoreLeft());

#if 1
	if (wm.audioMemory().recoveryTime().cycle() >= wm.time().cycle() - 10) {
		if (our_score > opp_score) {
			return true;
		}
	}
#endif

	long cycle_thr = std::max(0,
			ServerParam::i().nrNormalHalfs()
					* (ServerParam::i().halfTime() * 10) - 500);

	if (wm.time().cycle() < cycle_thr) {
		return false;
	}

	if (our_score > opp_score && our_score - opp_score <= 1) {
		return true;
	}

	return false;
}

/*-------------------------------------------------------------------*/
/*!
 execute action
 */
void Bhv_SetPlay::doBasicTheirSetPlayMove(PlayerAgent * agent) {
	const WorldModel & wm = agent->world();

	Vector2D target_point = Strategy::i().getPosition(wm.self().unum());

	Vector2D ball = wm.ball().pos();
	Vector2D me = wm.self().pos();

	int num = wm.self().unum();

	float MARK_DIST = 2.0;

	if (num < 6)
		MARK_DIST = 1.0;

	const PlayerPtrCont & opps = wm.opponentsFromSelf();
	const PlayerObject * nearestOpp = (
			opps.empty() ? static_cast<PlayerObject *>(0) : opps.front());
	const Vector2D opp = (
			nearestOpp ? nearestOpp->pos() : Vector2D(-1000.0, 0.0));

	double oppDist = opp.dist(target_point);

	double maxDist = 16.0;
	if (num < 6)
		maxDist = 5.0;

	bool amInearest = true;

	const PlayerPtrCont::const_iterator t_end = wm.teammatesFromSelf().end();
	for (PlayerPtrCont::const_iterator t = wm.teammatesFromSelf().begin();
			t != t_end; ++t) {
		Vector2D tpos = (*t)->pos();

		if ((*t)->posCount() > 5)
			continue;

		if (tpos.dist(opp) < me.dist(opp)) {
			amInearest = false;
			break;
		}
	}

	bool isMarking = false;

	if (me.dist(target_point) < maxDist && amInearest && opp.dist(ball) > 9.5) {
		if (num < 6 && opp.x > me.x + 4.0) {

		} else if (target_point.absY() > 29.0) {

		} else {
			target_point = opp
					+ Vector2D::polar2vector(MARK_DIST, (ball - opp).th());
			isMarking = true;
		}
	}

//     if( num < 6 && me.dist( target_point ) < 3.0 && amInearest && opp.dist( ball ) > 10.0 )
//          target_point = opp + Vector2D::polar2vector( 2.0 , (ball-opp).th() );

	dlog.addText(Logger::TEAM,
			__FILE__": their set play. HomePosition=(%.2f, %.2f)",
			target_point.x, target_point.y);

	double dash_power = Bhv_SetPlay::get_set_play_dash_power(agent);

	Vector2D ball_to_target = target_point - wm.ball().pos();

	if (ball_to_target.r() < 10.0) {
		double xdiff = std::sqrt(
				std::pow(10.0, 2) - std::pow(ball_to_target.y, 2));
		target_point.x = wm.ball().pos().x - xdiff;

		dlog.addText(Logger::TEAM,
				__FILE__": avoid circle(1). adjust x. x_diff=%.1f newPos=(%.2f %.2f)",
				xdiff, target_point.x, target_point.y);

		if (target_point.x < -45.0) {
			target_point = wm.ball().pos();
			target_point += ball_to_target.setLengthVector(11.0);

			dlog.addText(Logger::TEAM,
					__FILE__": avoid circle(2). adjust len. new_pos=(%.2f %.2f)",
					target_point.x, target_point.y);
		}
	}

	//
	// avoid kickoff offside
	//
	if (wm.gameMode().type() == GameMode::KickOff_
			&& ServerParam::i().kickoffOffside()) {
		target_point.x = std::min(-1.0e-5, target_point.x);

		dlog.addText(Logger::TEAM,
				__FILE__": avoid kickoff offside. (%.2f %.2f)", target_point.x,
				target_point.y);
	}

	//
	// find sub target
	//
	dlog.addText(Logger::TEAM,
			__FILE__": find sub target to avoid ball circle");
	Vector2D adjusted_point = get_avoid_circle_point(wm, target_point);

	//
	// go to the target point
	//
	double dist_thr = wm.ball().distFromSelf() * 0.1;
	if (dist_thr < 0.7)
		dist_thr = 0.7;

	if (isMarking)
		dist_thr = 0.3;

	if (adjusted_point != target_point
			&& wm.ball().pos().dist(target_point) > 10.0
			&& wm.self().inertiaFinalPoint().dist(adjusted_point) < dist_thr) {
		dlog.addText(Logger::TEAM,
				__FILE__": reverted to the first target point");
		adjusted_point = target_point;
	}

	//agent->debugClient().addMessage( "GoTo" );
	agent->debugClient().setTarget(target_point);
	agent->debugClient().addCircle(target_point, dist_thr);

	if (!Body_GoToPoint(adjusted_point, dist_thr, dash_power).execute(agent)) {
		// already there
		AngleDeg body_angle = wm.ball().angleFromSelf();
		if (body_angle.degree() < 0.0)
			body_angle -= 90.0;
		else
			body_angle += 90.0;

		Body_TurnToAngle(body_angle).execute(agent);
	}

//     agent->setNeckAction( new Neck_TurnToBall() );
	agent->setNeckAction(new Neck_ForMark());

}

//zsq 以下两函数参考11年
void Bhv_SetPlay::Adv_TheirSetPlayMove(PlayerAgent * agent) {
	//改变球员的跑位点
	const WorldModel & wm = agent->world();
	Vector2D home_point = Strategy::i().getPosition(wm.self().unum());
	Vector2D target_point = home_point;

	double min_defense_dir = 9.2;
	if (wm.gameMode().type() == GameMode::KickIn_) {
		min_defense_dir = 2.1;
	}
	//从近到远便历所有对手，不包括守门员/幻影/以及位置不确定的对手
	const PlayerPtrCont::const_iterator end = wm.opponentsFromSelf().end(); //遍历队友到球的距离
	for (PlayerPtrCont::const_iterator p = wm.opponentsFromSelf().begin();
			p != end; ++p) {
		//是守门员
		if ((*p)->goalie())
			continue;
		//是幻影
		if ((*p)->isGhost())
			continue;
		//位置不确定
		if ((*p)->posCount() >= 5)
			continue;

		if ((*p)->distFromBall() < min_defense_dir)
			continue;   //zsq 是发球的人

		//获取离当前对手的最近的队友
		double min_dist = 100.0;
		double self_min_dist = ((*p)->pos() - wm.self().pos()).r();
		//zsq 新加
		const PlayerObject *_zmate = wm.getTeammateNearestTo((*p)->pos(), 5,
				&min_dist);
		//如果对方周围没有队友，跳过
		if (!_zmate) {
			continue;
		}

		Vector2D teammate_point = _zmate->pos();
		if (self_min_dist > min_dist) {
			//				if(min_dist<3.0)
			//				{
			//					continue;
			//				}
			continue;
		} else {
			Vector2D target_opp_point = (*p)->pos();
			rcsc::Line2D target_line = rcsc::Line2D(target_opp_point,
					wm.ball().pos());

			//如果对方球员的距离离自己的home点小于12的话，则将目标点设置为对方的点
			if ((target_opp_point - home_point).r() < 12.0) {
				if (wm.ball().pos().x >= target_opp_point.x) {
					target_point.x = target_opp_point.x + 0.7;  //zsq 原来是0.5
					target_point.y = target_line.getY(target_point.x);
				} else {
					target_point.x = target_opp_point.x - 0.7;
					target_point.y = target_line.getY(target_point.x);
				}
				target_point.x = min_max(-ServerParam::i().pitchHalfLength(),
						target_point.x, +ServerParam::i().pitchHalfLength());
				target_point.y = min_max(-ServerParam::i().pitchHalfWidth(),
						target_point.y, +ServerParam::i().pitchHalfWidth());
				break;
			} else {
				target_point = home_point;
			}

			if (target_point.dist(wm.ball().pos()) < min_defense_dir) {
				target_point = home_point;
			}

		}
	}  //end of the for

	double dash_power = Bhv_SetPlay::get_set_play_dash_power(agent);

	dlog.addText(Logger::TEAM,
			__FILE__": their set play. HomePosition=(%.2f, %.2f)",
			target_point.x, target_point.y);

	//
	// go to the target point
	//
	double dist_thr = wm.ball().distFromSelf() * 0.1;
	if (dist_thr < 0.7)
		dist_thr = 0.7;
	if (!Body_GoToPoint(target_point,	//change adjusted_point to target_point
			dist_thr, dash_power).execute(agent)) {
		//			// already there如果已经到达点，则停止下来转向球
		//			 Body_TurnToBall().execute( agent );
		// already there
		AngleDeg body_angle = wm.ball().angleFromSelf();
		if (body_angle.degree() < 0.0)
			body_angle -= 90.0;
		else
			body_angle += 90.0;

		Body_TurnToAngle(body_angle).execute(agent);
	}
	agent->setNeckAction(new Neck_TurnToBall());

}
