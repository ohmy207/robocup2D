/*
 *Copyright (C) TJNU
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "bhv_ZhPass.h"

#include <rcsc/common/logger.h>
#include <rcsc/common/server_param.h>

#include <rcsc/player/player_agent.h>
#include <rcsc/player/debug_client.h>
#include <rcsc/player/audio_sensor.h>
#include <rcsc/player/say_message_builder.h>
#include <rcsc/player/view_grid_map.h>

#include <rcsc/action/body_kick_one_step.h>
#include <rcsc/action/body_hold_ball.h>
#include <rcsc/action/neck_turn_to_goalie_or_scan.h>
#include <rcsc/action/neck_turn_to_low_conf_teammate.h>
#include <rcsc/action/neck_turn_to_ball_or_scan.h>
#include <rcsc/action/body_smart_kick.h>
#include <rcsc/action/body_intercept.h>

#include <rcsc/geom/triangle_2d.h>
#include <rcsc/geom/rect_2d.h>

#include <math.h>

using namespace std;


Bhv_ZhPass::Bhv_ZhPass() {

}

Bhv_ZhPass::~Bhv_ZhPass() {

}

bool Bhv_ZhPass::execute(rcsc::PlayerAgent * agent) {
	rcsc::Vector2D ball = agent->world().ball().pos();


	const rcsc::WorldModel & wm = agent->world();

	for (int i = 0; i < 20; i++) {
		opp[i] = rcsc::Vector2D(-52.5, 0.0);
		tmm[i] = rcsc::Vector2D(-52.5, 0.0);
		receiver[i] = -1;
		oCount[i] = 100;
		tCount[i] = 100;
	}

	oMax = 0;
	tMax = 0;

	const rcsc::PlayerPtrCont::const_iterator o_end =
			wm.opponentsFromSelf().end();
	int i = 0;

	for (rcsc::PlayerPtrCont::const_iterator o = wm.opponentsFromSelf().begin();
			o != o_end; ++o) {

		if ((*o)->posCount() > 30)
			continue;

		if ((*o)->pos().dist(ball) < 1.7)
			continue;

		opp[i] = (*o)->pos() + (*o)->vel();
		oCount[i] = (*o)->posCount();

		i++;
	}

	oMax = i;

	const rcsc::PlayerPtrCont::const_iterator t_end =
			wm.teammatesFromSelf().end();
	int j = 0;

	for (rcsc::PlayerPtrCont::const_iterator t = wm.teammatesFromSelf().begin();
			t != t_end; ++t) {

		if ((*t)->goalie())
			continue;

		if ((*t)->posCount() > 3)
			continue;

		tmm[j] = (*t)->pos() + (*t)->vel();
		tAngle[j] = (*t)->body().degree();
		receiver[j] = (*t)->unum();
		tCount[j] = (*t)->posCount();

		j++;
	}

	tMax = j;

	if (!agent->world().self().isKickable())
		return false;

	rcsc::Vector2D target = rcsc::Vector2D(0.0, 0.0);

	if (BrainCross2(agent, target)) {
		return true;
	}
	if (CrossPass(agent, target)) {
		return true;
	}
	if (ThroughPass(agent, target)) {
		return true;
	}

	if (DirectPass(agent, target) && ball.x < 36.0) {
		return true;
	}



	agent->setNeckAction(new rcsc::Neck_TurnToLowConfTeammate());


	return false;

}

int Bhv_ZhPass::cycles(rcsc::PlayerAgent * agent, rcsc::Vector2D target,
		float speed, int i, rcsc::Vector2D o[]) {
	const rcsc::WorldModel & wm = agent->world();

	rcsc::Vector2D ball = wm.ball().pos();

	rcsc::AngleDeg passDir = (target - ball).dir();
	float ballSpeed = speed;
	int time = 0;
	ball += rcsc::Vector2D::polar2vector(ballSpeed,
			(target - ball).dir().degree());
	ballSpeed *= 0.94;
	time++;

	while (ballSpeed > 0.4 && time < 60) {
		int kickable = 1;
		for (int kh = 0; kh < i; kh++) {
			if (o[kh].dist(ball) < kickable + (time - 1)) {
				return time;
			}
		}
		ball += rcsc::Vector2D::polar2vector(ballSpeed,
				(target - ball).dir().degree());
		ballSpeed *= 0.94;
		time++;
	}
	return 1000;
}

int Bhv_ZhPass::throughCycles(rcsc::PlayerAgent * agent, rcsc::Vector2D target,
		float speed, int i, rcsc::Vector2D o[]) {
	const rcsc::WorldModel & wm = agent->world();

	rcsc::Vector2D ball = wm.ball().pos();

	rcsc::AngleDeg passDir = (target - ball).dir();
	float ballSpeed = speed;
	int time = 0;
	ball += rcsc::Vector2D::polar2vector(ballSpeed,
			(target - ball).dir().degree());
	ballSpeed *= 0.94;
	time++;

	while (ballSpeed > 0.15 && time < 60) {
		int kickable = 1;
		for (int kh = 0; kh < i; kh++) {
//         if ( agent->world().getOpponentGoalie() != NULL && (o[kh].x == agent->world().getOpponentGoalie()->pos().x && o[kh].y == agent->world().getOpponentGoalie()->pos().y))
//            kickable = 2;
			if (o[kh].dist(ball) < kickable + (time - 1)) {
				if (ball.x > o[kh].x + 1.0)
					return time;
			}
		}
		ball += rcsc::Vector2D::polar2vector(ballSpeed,
				(target - ball).dir().degree());
		ballSpeed *= 0.94;
		time++;
	}

	return 1000;
}

int Bhv_ZhPass::cycles2(rcsc::PlayerAgent * agent, rcsc::Vector2D target,
		float speed, int i, rcsc::Vector2D o[], float *x, float *y) {
	const rcsc::WorldModel & wm = agent->world();

	rcsc::Vector2D ball = wm.ball().pos();

	rcsc::AngleDeg passDir = (target - ball).dir();
	float ballSpeed = speed;
	int time = 0;
	ball += rcsc::Vector2D::polar2vector(ballSpeed,
			(target - ball).dir().degree());
	ballSpeed *= 0.94;
	time++;

	while (ballSpeed > 0.4 && time < 60) {
		int kickable = 1;
		for (int kh = 0; kh < i; kh++) {
			if (o[kh].dist(ball) < kickable + (time - 1)) {
				*x = ball.x;
				*y = ball.y;
				return time;
			}
		}
		ball += rcsc::Vector2D::polar2vector(ballSpeed,
				(target - ball).dir().degree());
		ballSpeed *= 0.94;
		time++;
	}

	return 1000;
}

rcsc::Vector2D Bhv_ZhPass::finalPoint(rcsc::PlayerAgent * agent, float dir,
		float speed) {
	const rcsc::WorldModel & wm = agent->world();
	rcsc::Vector2D ball = wm.ball().pos();
	rcsc::Vector2D me = wm.self().pos();

	int time = 0;

	float ballSpeed = speed;
	ball += rcsc::Vector2D::polar2vector(ballSpeed, dir);
	ballSpeed *= 0.94;
	time++;

	if (ball.dist(me)
			< (wm.self().playerType().playerSize()
					+ wm.self().playerType().kickableArea())) {
		ball += rcsc::Vector2D::polar2vector(ballSpeed, dir);
		ballSpeed *= 0.94;
		time++;
	}

	while (ball.dist(me)
			> (wm.self().playerType().playerSize()
					+ wm.self().playerType().kickableArea())) {
		ball += rcsc::Vector2D::polar2vector(ballSpeed, dir);
		ballSpeed *= 0.94;
		time++;
		me += rcsc::Vector2D::polar2vector(.8, dir);
	}

	return ball;
}

int Bhv_ZhPass::sign(float n) {
	if (n < 0)
		return -1;

	return 1;
}

bool Bhv_ZhPass::SRPD(rcsc::PlayerAgent * agent) {

	const rcsc::WorldModel & wm = agent->world();
	rcsc::Vector2D ball = wm.ball().pos();
	rcsc::Vector2D me = wm.self().pos();


	double goal_angle = (rcsc::Vector2D(52.5, wm.self().pos().y)
			- wm.self().pos()).dir().degree();
	double difAngle = wm.self().body().degree() - goal_angle;
	if (difAngle > 20.0 || difAngle < -20.0)
		return false;

	if (wm.self().vel().x < 0.04)
		return false;

	rcsc::Vector2D p = ball + rcsc::Vector2D(20.0, 0.0);
	rcsc::Vector2D b, c, d;

	if (wm.self().unum() == 9 || wm.self().unum() == 10) {
		d = me + rcsc::Vector2D(-1.0, 0.0);
		b = d + rcsc::Vector2D(5.7, -8.0);
		c = d + rcsc::Vector2D(5.7, 8.0);
		int a = wm.countOpponentsIn(rcsc::Triangle2D(d, b, c), 20, true)
				+ wm.countOpponentsIn(
						rcsc::Rect2D(b, c + rcsc::Vector2D(10.0, 0.0)), 20,
						true);
		if (a == 0 && ball.x < 40 && wm.self().stamina() > 3500.0
				&& ball.x > 0.0) {
			rcsc::Vector2D one_step_vel = rcsc::KickTable::calc_max_velocity(
					0.0,
					agent->world().self().kickRate(),
					agent->world().ball().vel());
			double one_step_speed = one_step_vel.r();
			float v0 = 1.2;

			if (ball.x > 32.0)
				v0 = 0.9;

			if (one_step_speed > v0) {
				rcsc::Body_SmartKick(p, v0, v0 - 0.3, 1).execute(agent);

				agent->setNeckAction(new rcsc::Neck_TurnToBallOrScan());

				return true;
			}
		}

	}
	if (wm.self().unum() == 11 && wm.self().stamina() > 3500.0
			&& ball.x > 0.0) {
		d = me + rcsc::Vector2D(-1.0, 0.0);
		b = d + rcsc::Vector2D(5.7, -8.0);
		c = d + rcsc::Vector2D(5.7, 8.0);
		int a = wm.countOpponentsIn(rcsc::Triangle2D(d, b, c), 20, true)
				+ wm.countOpponentsIn(
						rcsc::Rect2D(b, c + rcsc::Vector2D(10.0, 0.0)), 20,
						true);
		if (a == 0 && ball.x < 25) {
			rcsc::Vector2D one_step_vel = rcsc::KickTable::calc_max_velocity(
					0.0, //好吧,方向
					agent->world().self().kickRate(),
					agent->world().ball().vel());
			double one_step_speed = one_step_vel.r();
			float v0 = 1.2;

			if (ball.x > 32.0)
				v0 = 0.9;

			if (one_step_speed > v0) {
				rcsc::Body_SmartKick(p, v0, v0 - 0.3, 1).execute(agent);

				agent->setNeckAction(new rcsc::Neck_TurnToBallOrScan());
				return true;
			}
		}
	}

	d = me + rcsc::Vector2D(0.0, 0.0);
	b = d + rcsc::Vector2D(3.0, -4.5);
	c = d + rcsc::Vector2D(3.0, 4.5);
	int z = wm.countOpponentsIn(rcsc::Triangle2D(d, b, c), 25, true)
			+ wm.countOpponentsIn(
					rcsc::Rect2D(b, c + rcsc::Vector2D(10.0, 0.0)), 25, true);

	if (z == 0 && ball.x > -30.0 && ball.x < 40.0 && wm.self().unum() > 6
			&& wm.self().stamina() > 3000.0) {
		rcsc::Vector2D one_step_vel = rcsc::KickTable::calc_max_velocity(0.0,
				agent->world().self().kickRate(), agent->world().ball().vel());
		double one_step_speed = one_step_vel.r();
		float v0 = 0.80;
		if (one_step_speed > v0) {
			rcsc::Body_SmartKick(p, v0, v0 - 0.3, 1).execute(agent);

			agent->setNeckAction(new rcsc::Neck_TurnToBallOrScan());
			return true;
		}
	}

	return false;
}

bool Bhv_ZhPass::BrainCross2(rcsc::PlayerAgent * agent,
		rcsc::Vector2D & target) {

	const rcsc::WorldModel & wm = agent->world();
	rcsc::Vector2D ball = wm.ball().pos();
	rcsc::Vector2D me = wm.self().pos();

	if (ball.x < 32.0)
		return false;

	if (agent->world().gameMode().type() != rcsc::GameMode::PlayOn)
		return false;

	if ((wm.self().unum() == 9 || wm.self().unum() == 10) && ball.x < 43.0)
		return false;

	for (int i = 0; i < tMax; i++) {

		if (tmm[i].x < 31.0 || wm.offsideLineX() < 39.0)
			continue;

		if (tmm[i].x > wm.offsideLineX() - 0.1)
			continue;

		if (tCount[i] > 1)
			continue;

		if (receiver[i] == 9 && wm.self().unum() == 7 && tmm[i].absY() > 10.0)
			continue;

		if (receiver[i] == 10 && wm.self().unum() == 8 && tmm[i].absY() > 10.0)
			continue;

		if (wm.self().unum() == 11
				&& ((me.absY() < 13.5 && tmm[i].absY() > 10.0)
						|| (me.absY() < 13.5 && tmm[i].x < me.x - 3.0)))
			continue;

		if ((wm.self().unum() == 7 || wm.self().unum() == 8)
				&& (receiver[i] == 8 || receiver[i] == 7) && ball.x < 40.0
				&& tmm[i].dist(me) < 5.0)
			continue;

		if (((wm.self().unum() == 9 && receiver[i] == 7)
				|| (wm.self().unum() == 10 && receiver[i] == 8))
				&& (ball.x < 44.0 || tmm[i].x < 36.0))
			continue;

		for (int t = 0; t >= -1; t--) {
			rcsc::Vector2D targ = tmm[i] + rcsc::Vector2D::polar2vector(t, 0);

			if (targ.x > 52.0)
				continue;

			if (me.dist(targ) < 3.0)
				continue;

			if (me.absY() > 16.0 && me.dist(targ) < 6.0)
				continue;


			rcsc::Vector2D oneStepVel = rcsc::KickTable::calc_max_velocity(
					(targ - ball).th(), agent->world().self().kickRate(),
					agent->world().ball().vel());
			double v0 = oneStepVel.r();

			rcsc::Vector2D tmptmm[1];

			tmptmm[0] = tmm[i];

			int delCycle = cycles(agent, targ, v0, oMax, opp)
					- cycles(agent, targ, v0, 1, tmptmm);

			if (delCycle < 1)
				continue;

			target = targ;

			rcsc::Body_KickOneStep(target, v0).execute(agent);

			if (agent->config().useCommunication() && receiver[i] != -1) {
				rcsc::Vector2D target_angle = target - tmm[i];
				target_angle.setLength(1.0);
				agent->addSayMessage(
						new rcsc::PassMessage(receiver[i],
								target + target_angle,
								agent->effector().queuedNextBallPos(),
								agent->effector().queuedNextBallVel()));
				agent->setNeckAction(new rcsc::Neck_TurnToLowConfTeammate());
			}
			return true;

		}
	}
	return false;
}

bool Bhv_ZhPass::BrainCross(rcsc::PlayerAgent * agent,
		rcsc::Vector2D & target) {

	const rcsc::WorldModel & wm = agent->world();
	rcsc::Vector2D ball = wm.ball().pos();
	rcsc::Vector2D me = wm.self().pos();

	if (ball.x < 32.0)
		return false;

	if (agent->world().gameMode().type() != rcsc::GameMode::PlayOn)
		return false;

	if ((wm.self().unum() == 9 || wm.self().unum() == 10) && ball.x < 40.0)
		return false;

	for (int i = 0; i < tMax; i++) {

		if (tmm[i].x < 31.0 || wm.offsideLineX() < 39.0)
			continue;

		if (tmm[i].x > wm.offsideLineX() - 0.1)
			continue;

		if (tCount[i] > 1)
			continue;

		if (receiver[i] == 9 && wm.self().unum() == 7 && tmm[i].absY() > 10.0)
			continue;

		if (receiver[i] == 10 && wm.self().unum() == 8 && tmm[i].absY() > 10.0)
			continue;

		if (wm.self().unum() == 11
				&& (tmm[i].absY() > 7.0 || tmm[i].x < me.x - 3.0
						|| tmm[i].absY() < 10.0))
			continue;

		if ((wm.self().unum() == 7 || wm.self().unum() == 8)
				&& (receiver[i] == 8 || receiver[i] == 7) && ball.x < 40.0
				&& tmm[i].dist(me) < 5.0)
			continue;

		if (((wm.self().unum() == 9 && receiver[i] == 7)
				|| (wm.self().unum() == 10 && receiver[i] == 8))
				&& (ball.x < 44.0 || tmm[i].x < 36.0))
			continue;

		for (int t = 0; t >= -1; t--) {
			rcsc::Vector2D targ = tmm[i] + rcsc::Vector2D::polar2vector(t, 0);

			if (targ.x > 52.0)
				continue;

			if (me.dist(targ) < 4.0)
				continue;

			float v0 = 3.0;

			v0 *= ball.dist(targ) / 9.0;

			if (v0 > 3.0)
				v0 = 3.0;

			if (v0 < 1.0)
				v0 = 1.0;

			rcsc::Vector2D tmptmm[1];

			tmptmm[0] = tmm[i];

			int delCycle = cycles(agent, targ, v0, oMax, opp)
					- cycles(agent, targ, v0, 1, tmptmm);

			if (delCycle < 1)
				continue;

			target = targ;

			int kick_step = (
					agent->world().gameMode().type() != rcsc::GameMode::PlayOn
							&& agent->world().gameMode().type()
									!= rcsc::GameMode::GoalKick_ ? 1 : 3);


			while ((!rcsc::Body_SmartKick(target, v0, v0 * 0.97, kick_step).execute(
					agent)) && v0 > 1.5) {
				rcsc::Vector2D ttmptmm[1];
				ttmptmm[0] = target;

				v0 *= 0.90;

				if (cycles(agent, target, (v0 * 0.9), oMax, opp)
						- cycles(agent, target, (v0 * 0.9), 1, ttmptmm) < 1)
					break;
			}

			if (agent->config().useCommunication() && receiver[i] != -1) {
				rcsc::Vector2D target_angle = target - tmm[i];
				target_angle.setLength(1.0);
				agent->addSayMessage(
						new rcsc::PassMessage(receiver[i],
								target + target_angle,
								agent->effector().queuedNextBallPos(),
								agent->effector().queuedNextBallVel()));
				agent->setNeckAction(new rcsc::Neck_TurnToLowConfTeammate());
			}
//			std::cout << " Cycle: "<< agent->world().time().cycle() << "\n";
			return true;

		}
	}
	return false;
}

bool Bhv_ZhPass::CrossPass(rcsc::PlayerAgent * agent, rcsc::Vector2D & target) {

	const rcsc::WorldModel & wm = agent->world();
	rcsc::Vector2D ball = wm.ball().pos();

	if (ball.x < 36 || std::fabs(ball.y) < 6)
		return false;

	if (agent->world().gameMode().type() != rcsc::GameMode::PlayOn)
		return false;

	for (int i = 0; i < tMax; i++) {

		if (tmm[i].x < 36)
			continue;

		if (ball.y > 0)
			if (tmm[i].y > ball.y - 2)
				continue;
		if (ball.y < 0)
			if (tmm[i].y < ball.y + 2)
				continue;

		if (tmm[i].x > wm.offsideLineX() - 0.3)
			continue;

		if (tCount[i] > 1)
			continue;

		for (int t = 4; t > -2; t--) {
			rcsc::Vector2D targ = tmm[i] + rcsc::Vector2D::polar2vector(t, 0);

			if (targ.x > 52)
				continue;

			float v0 = (targ.dist(ball) - 0.5 * -(0.0582) * t * t) / t;

			if (v0 > 3.0)
				continue;

			rcsc::Vector2D tmptmm[1];

			tmptmm[0] = tmm[i];

			int delCycle = cycles(agent, targ, v0, oMax, opp)
					- cycles(agent, targ, v0, 1, tmptmm);

			if (delCycle < 1)
				continue;

			target = targ;

			v0 *= 0.7;

			if (v0 < 1.0)
				v0 = 1.0;

			int kick_step = (
					agent->world().gameMode().type() != rcsc::GameMode::PlayOn
							&& agent->world().gameMode().type()
									!= rcsc::GameMode::GoalKick_ ? 1 : 3);

			if (!rcsc::Body_SmartKick(target, v0, v0 * 0.96, kick_step).execute(
					agent)) {
				if (agent->world().gameMode().type() != rcsc::GameMode::PlayOn
						&& agent->world().gameMode().type()
								!= rcsc::GameMode::GoalKick_) {
					v0 = std::min(
							agent->world().self().kickRate()
									* rcsc::ServerParam::i().maxPower(),
							double(v0));
					rcsc::Body_KickOneStep(target, v0).execute(agent);
				} else {
					continue;
				}
			}

			if (agent->config().useCommunication() && receiver[i] != -1) {
				rcsc::Vector2D target_angle = target - tmm[i];
				target_angle.setLength(1.0);
				agent->addSayMessage(
						new rcsc::PassMessage(receiver[i],
								target + target_angle,
								agent->effector().queuedNextBallPos(),
								agent->effector().queuedNextBallVel()));
				agent->setNeckAction(new rcsc::Neck_TurnToLowConfTeammate());
			}
			return true;

		}
	}
	return false;
}

bool Bhv_ZhPass::ThroughPass(rcsc::PlayerAgent * agent,
		rcsc::Vector2D & target) {

	const rcsc::WorldModel & wm = agent->world();
	rcsc::Vector2D ball = wm.ball().pos();

	for (int i = 0; i < tMax; i++) {

		if (tmm[i].x < 0)
			continue;

		if (tmm[i].x < ball.x - 5.0)
			continue;


		if (tmm[i].x > wm.offsideLineX())
			continue;

		if (tCount[i] > 1)
			continue;

		if (ball.x > wm.offsideLineX() - 1.0)
			continue;

		for (int t = 30; t > 7; t--)
			for (int z = 0; z < 1; z++)
					{

				rcsc::Vector2D targ =
						tmm[i]
								+ rcsc::Vector2D::polar2vector(t,
										(rcsc::Vector2D(52.5, tmm[i].y) - tmm[i]).dir().degree()
												+ sign(tmm[i].y) * (z * 10));

				if (targ.absX() > 45.0)
					continue;

				if (targ.y > 31.0)
					targ.y = 31.0;
				if (targ.y < -31.0)
					targ.y = -31.0;

				if (tmm[i].x < -5.0)
					continue;

				if (receiver[i] < 9)
					continue;

				if (targ.x < wm.offsideLineX() - 2.0)
					continue;

				if (receiver[i] == 9 && targ.absY() < 20.0
						&& targ.x < wm.offsideLineX() + 4.0)
					continue;

				if (targ.dist(rcsc::Vector2D(52.5, 0.0)) < 9.0)
					continue;

				if (targ.x > 43.0 && targ.absY() < 12.0)
					continue;


				float v0 = (targ.dist(ball) - 0.5 * -(0.0582) * t * t) / t;

				if (v0 > 3.1)
					continue;

				rcsc::Vector2D tmptmm[1];

				tmptmm[0] = tmm[i];

				int delCycle = cycles(agent, targ, v0, oMax, opp)
						- throughCycles(agent, targ, v0, 1, tmptmm);

				if (delCycle < 1)
					continue;


				v0 = std::min(float(3.0), v0);


				target = targ;

				int kick_step = (
						agent->world().gameMode().type()
								!= rcsc::GameMode::PlayOn
								&& agent->world().gameMode().type()
										!= rcsc::GameMode::GoalKick_ ? 1 : 3);

				if (!rcsc::Body_SmartKick(target, v0, v0 * 0.96, kick_step).execute(
						agent)) {
					if (agent->world().gameMode().type()
							!= rcsc::GameMode::PlayOn
							&& agent->world().gameMode().type()
									!= rcsc::GameMode::GoalKick_) {
						v0 = std::min(
								agent->world().self().kickRate()
										* rcsc::ServerParam::i().maxPower(),
								double(v0));
						rcsc::Body_KickOneStep(target, v0).execute(agent);
					} else {
						continue;
					}
				}

				if (agent->config().useCommunication() && receiver[i] != -1) {
					rcsc::Vector2D target_angle = target - tmm[i];
					target_angle.setLength(1.0);
					agent->addSayMessage(
							new rcsc::PassMessage(receiver[i],
									target + target_angle,
									agent->effector().queuedNextBallPos(),
									agent->effector().queuedNextBallVel()));
					agent->setNeckAction(
							new rcsc::Neck_TurnToLowConfTeammate());
				}
				return true;


			}
	}
	return false;
}


bool Bhv_ZhPass::LeadingPass(rcsc::PlayerAgent * agent,
		rcsc::Vector2D & target) {

	const rcsc::WorldModel & wm = agent->world();
	rcsc::Vector2D ball = wm.ball().pos();

	float maxRate = -1000.0;
	int bi = -1;
	rcsc::Vector2D bTarget = rcsc::Vector2D(52.5, 0.0);
	float bSpeed = 3.0;

	for (int i = 0; i < tMax; i++) {
		if (tCount[i] > 1)
			continue;
		if (tmm[i].x > wm.offsideLineX())
			continue;
		if ((std::fabs(ball.x) < 36)
				&& (tmm[i].dist(ball) < (9.0 - ((ball.x + 36.0) / 8.0) + 2.0)))
			continue;
		if (ball.x < -36 && tmm[i].dist(ball) < 10)
			continue;
		for (int j = 0; (j / 8) < 3; j++) {
//				 设传球点
			float d0 = 3.0;
			float d1 = 2.0;
			if (tmm[i].x > 34.0 && tmm[i].absY() < 18.0) {
				d0 = 2.0;
				d1 = 1.0;
			}
			rcsc::Vector2D pPoint = (tmm[i]
					+ rcsc::Vector2D::polar2vector((j / 8) * d1 + d0,
							(j / 8 != 0) * (45.0 / pow(2.0, j / 8))
									+ (j % 8) * 45.0));
			if (pPoint.absY() > 32 || abs(pPoint.x) > 51)
				continue;
			if (pPoint.x < ball.x - 20)
				continue;

			float passSpeed = 3.0;
			float endSpeed = 1.8;
			float speedPenalty = 0.0;

			if (std::fabs((tAngle[i] - (ball - tmm[i]).dir()).degree()) < 45.0)
				speedPenalty = 0.05;
			else if (std::fabs((tAngle[i] - (ball - tmm[i]).dir()).degree())
					< 135.0)
				speedPenalty = 0.1;
			else if (std::fabs((tAngle[i] - (ball - tmm[i]).dir()).degree())
					> 170.0)
				speedPenalty = 0.075;
			else
				speedPenalty = 0.15;

			endSpeed -= float(float(int(j / 8) * d1 + d0) * speedPenalty);
			float disToOut = std::min(
					std::min(pPoint.dist(rcsc::Vector2D(pPoint.x, 32.0)),
							pPoint.dist(rcsc::Vector2D(pPoint.x, -32.0))),
					std::min(pPoint.dist(rcsc::Vector2D(-51.0, pPoint.y)),
							pPoint.dist(rcsc::Vector2D(51.0, pPoint.y))));
			if (disToOut < 4.0)
				endSpeed -= ((4.0 - disToOut) * (endSpeed / 4.0));
			if (((endSpeed * endSpeed) - (2 * (-0.0582) * pPoint.dist(ball)))
					>= 0)
				passSpeed = sqrt(
						(endSpeed * endSpeed)
								- (2 * (-0.0582) * pPoint.dist(ball)));
			else
			{
				passSpeed = 2.6;
				passSpeed *= ball.dist(pPoint) / 25.0;
				passSpeed = std::min(float(2.6), passSpeed);
				passSpeed = std::max(float(1.0), passSpeed);
			}
			rcsc::Vector2D tmptmm[1];
			tmptmm[0] = tmm[i];
			int delCycle = cycles(agent, pPoint, passSpeed, oMax, opp)
					- cycles(agent, pPoint, passSpeed, 1, tmptmm);

			if ((delCycle < 2 && !(pPoint.x > 34.0 && pPoint.absY() < 18.0))
					|| delCycle < 0)
				continue;

			float rate = 0.0;
			{
				{
					if (delCycle > 3)
						delCycle = 3;
					delCycle -= 2;
					float dCycleRate = 3;
					if (delCycle < 0)
						dCycleRate = 6;
					rate += (delCycle * dCycleRate);
				}
				{
					float idealDist = 25;
					if (ball.x > 33)
						idealDist = 12;
					float x = (std::fabs(ball.dist(pPoint) - 25.0));
					if (x > 10)
						x = 10.0;
					float xRate = 0.4;
					rate -= (x * xRate);
				}
//  离球门距离
				{
					float dGoal = (ball.dist(rcsc::Vector2D(52.2, 0.0))
							- pPoint.dist(rcsc::Vector2D(52.5, 0.0)));
					if (dGoal > 25)
						dGoal = 25;
					if (dGoal < -25)
						dGoal = -25;
					float dGoalRate = 0.7;
					if (dGoal < 0 || ball.x < 30)
						dGoalRate *= 2.5;
					rate += (dGoal * dGoalRate);
				}
				{
					float xDiffRate = 0.3;
					if ((pPoint.x - ball.x) < -5)
						xDiffRate *= 2;
					if (ball.x > 30 && (pPoint.x - ball.x) < 0)
						xDiffRate /= 3;
					rate += std::min(float(pPoint.x - ball.x), float(20.0))
							* xDiffRate;
				}
				{
					float xDiffRate = 0.5;
					if ((pPoint.x - tmm[i].x) < -3)
						xDiffRate *= 2;
					rate += std::min(float(pPoint.x - tmm[i].x), float(6.0))
							* xDiffRate;
				}
				{
					if (pPoint.absY() > 15.0)
						rate += (pPoint.absY() - 15.0) * 1.0;
				}
///咱们的禁区
				{
					if (pPoint.x < -36.0 && pPoint.absY() < 18.0)
						rate -= 20;
				}
			} //end

			if (rate > maxRate) {
				maxRate = rate;
				bi = i;
				bTarget = pPoint;
				bSpeed = passSpeed;
			}

		} //end
	} //end


// 执行部分
	if (maxRate < 6)
		return false;

	rcsc::Vector2D one_step_vel = rcsc::KickTable::calc_max_velocity(
			(bTarget - agent->world().ball().pos()).th(),
			agent->world().self().kickRate(), agent->world().ball().vel());
	double one_step_speed = one_step_vel.r();

	target = bTarget;

//踢一脚
	if (one_step_speed > bSpeed * 0.5 && tMax > 0)
			{
		rcsc::Body_SmartKick(bTarget, bSpeed, bSpeed * 0.9, 1).execute(agent);

		if (agent->config().useCommunication() && receiver[bi] != -1) {
			rcsc::Vector2D target_angle = bTarget - tmm[bi];
			target_angle.setLength(1.0);
			agent->addSayMessage(
					new rcsc::PassMessage(receiver[bi], bTarget + target_angle,
							agent->effector().queuedNextBallPos(),
							agent->effector().queuedNextBallVel()));
			agent->setNeckAction(new rcsc::Neck_TurnToLowConfTeammate());
		}
		return true;
	}
	return false;
}

bool Bhv_ZhPass::DirectPass(rcsc::PlayerAgent * agent,
		rcsc::Vector2D & target) {

	const rcsc::WorldModel & wm = agent->world();


	rcsc::Vector2D ball = wm.ball().pos();

	float passSpeed = 3.0;
	int delCycle[20];
	rcsc::Vector2D targets[20];

	for (int z = 0; z < 20; z++) {
		delCycle[z] = -10000;
		targets[z] = rcsc::Vector2D(200.0, 0.0);
	}

	for (int i = 0; i < tMax; i++) {
		if (tCount[i] > 1)
			continue;
		targets[i] = tmm[i];

		passSpeed = 3.0;
		if (ball.x < 36.0)
			passSpeed *= ball.dist(targets[i]) / 10.0;
		else
			passSpeed *= ball.dist(targets[i]) / 8.0;
		passSpeed = std::min(float(3.0), passSpeed);
		passSpeed = std::max(float(1.0), passSpeed);

		rcsc::Vector2D tmptmm[1];
		tmptmm[0] = targets[i];
		delCycle[i] = cycles(agent, targets[i], passSpeed, oMax, opp)
				- cycles(agent, targets[i], passSpeed, 1, tmptmm);
	}

	float rate[20];

	for (int z = 0; z < 20; z++)
		rate[z] = 0.0;

	for (int i = 0; i < tMax; i++) {
		int minCycle = 1;

		if (ball.x > 40.0 && targets[i].absY() < ball.absY()
				&& targets[i].x > 40.0
				&& targets[i].dist(rcsc::Vector2D(52.5, 0.0)) < 15.0
				&& targets[i].dist(ball) < 11.75 && targets[i].dist(ball) > 5.0)
			minCycle = 0;

		if (ball.x > wm.offsideLineX() - 20.0 && ball.x < 36.0
				&& wm.opponentsFromSelf().front()->distFromSelf() > 4.0)
			minCycle = 2;

		float idealPassDist = 20.0;

		if (ball.x < 25.0 && targets[i].x > ball.x
				&& targets[i].dist(ball) > 20.0)
			idealPassDist = 30.0;

		if (delCycle[i] < minCycle)
			continue;
		if (targets[i].x < ball.x - 25)
			continue;
		if (targets[i].x > wm.offsideLineX())
			continue;
		if ((std::fabs(ball.x) < 36)
				&& (targets[i].dist(ball)
						< (9.0 - ((ball.x + 36.0) / 8.0) + 2.0)))
			continue;
		if (ball.x < -36.0 && targets[i].dist(ball) < 5.0)
			continue;

//各种权重
		float dCycleRate = 3;
		float dist20Rate = 0.4;
		float dGoalRate = 3.0;
		float xRate = 1.0;
		float sideRate = 3.0;

// 进攻权重
		if (ball.x < -25.0) {
			dCycleRate = 3;
			dist20Rate = 0.4;
			dGoalRate = 3.0;
			xRate = 0.2;
			sideRate = 3.0;
		}
// 防守权重
		if (ball.x < -25.0) {
			dCycleRate = 6;
			dist20Rate = 0.4;
			dGoalRate = 3.0;
			xRate = 2.0;
			sideRate = 3.0;
		}

		if (ball.x < -33.0) {
			dCycleRate = 9;
			dist20Rate = 0.4;
			idealPassDist = 30.0;
			dGoalRate = 3.0;
			xRate = 3.0;
			sideRate = 3.0;
		}

//    预期
		if (delCycle[i] > 8)
			delCycle[i] = 8;

		rate[i] += (delCycle[i] * dCycleRate);

//		各种距离
		float dis = std::fabs(ball.dist(targets[i]) - idealPassDist);
		if (dis > 10)
			dis = 10.0;

		rate[i] -= (dis * dist20Rate);

		float dGoal = (ball.dist(rcsc::Vector2D(52.2, 0.0))
				- targets[i].dist(rcsc::Vector2D(52.5, 0.0)));
		if (dGoal > 25)
			dGoal = 25;
		if (dGoal < -25)
			dGoal = -25;
		if (dGoal < 0.0)
			dGoal *= 2.5;
		dGoal /= 3.0;

		rate[i] += (dGoal * dGoalRate);

///    X diff
		rate[i] += std::min(float(targets[i].x - ball.x), float(20.0)) * xRate;

//		侧传球
		if (targets[i].absY() > 15.0 && ball.x < 36.0)
			rate[i] += (targets[i].absY() - 15.0) * sideRate;

//		自己的禁区
		if (targets[i].x < -36.0 && targets[i].absY() < 18.0)
			rate[i] -= 20;

//    attack situation中的cross passes

		if (targets[i].x > 29.0 && targets[i].absY() < ball.absY())
			rate[i] += 10;

		if (targets[i].x > 38.0 && targets[i].absY() > ball.absY()
				&& targets[i].absY() > 18.0)
			rate[i] -= 5;

		if (targets[i].x < -36.0 && targets[i].absY() < 20.0)
			rate[i] -= 50;

		if (ball.x < -36.0 && receiver[i] > 8)
			rate[i] += 10;

	}

	float maxRate = -1000.0;
	int bi = -1;

	for (int i = 0; i < tMax; i++)
		if (rate[i] > maxRate) {
			maxRate = rate[i];
			bi = i;
		}

	float minRate = 1.0;

	if (ball.absY() < 20.0 && ball.x < 36.0)
		minRate = 0.5;

	if (wm.opponentsFromSelf().front()->pos().dist(wm.self().pos()) < 4.0)
		minRate = 0.25;

	if (wm.opponentsFromSelf().front()->pos().dist(wm.self().pos()) < 2.7)
		minRate = 0.0;

	if (wm.opponentsFromSelf().front()->pos().dist(wm.self().pos()) < 1.9)
		minRate = -0.25;


	if (maxRate < minRate) {
		return false;
	}

	passSpeed = 3.0;

	if (ball.x < 36.0)
		passSpeed *= ball.dist(targets[bi]) / 15.0;
	else
		passSpeed *= ball.dist(targets[bi]) / 11.0;

	passSpeed = std::min(float(3.0), passSpeed);
	passSpeed = std::max(float(1.0), passSpeed);


	target = targets[bi];

	int kickSteps = (
			agent->world().gameMode().type() != rcsc::GameMode::PlayOn
					&& agent->world().gameMode().type()
							!= rcsc::GameMode::GoalKick_ ? 1 : 3);

	while ((!rcsc::Body_SmartKick(target, passSpeed, passSpeed * 0.96,
			kickSteps).execute(agent)) && passSpeed > 1.5) {
		{
			rcsc::Vector2D ttmptmm[1];
			ttmptmm[0] = targets[bi];

			passSpeed *= 0.9;

			if (cycles(agent, targets[bi], (passSpeed * 0.9), oMax, opp)
					- cycles(agent, targets[bi], (passSpeed * 0.9), 1, ttmptmm)
					< 0)
				break;
		}
	}

	if (!rcsc::Body_SmartKick(target, passSpeed, passSpeed * 0.96, kickSteps).execute(
			agent)) {
		return false;
	}

	if (agent->config().useCommunication() && receiver[bi] != -1) {
		rcsc::Vector2D target_angle;
		target_angle.assign(0.0, 0.0);
		agent->addSayMessage(
				new rcsc::PassMessage(receiver[bi], targets[bi] + target_angle,
						agent->effector().queuedNextBallPos(),
						agent->effector().queuedNextBallVel()));
		agent->setNeckAction(new rcsc::Neck_TurnToLowConfTeammate());
	}

	return true;

}

