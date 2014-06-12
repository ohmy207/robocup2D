/*
 *Copyright (C) TJNU
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

//头文件啥的莫要遗漏

#include "bhv_ZhPass2.h"

#include <rcsc/common/logger.h>
#include <rcsc/common/server_param.h>

#include <rcsc/soccer_math.h>

#include <rcsc/common/player_type.h>

#include <rcsc/player/player_agent.h>
#include <rcsc/player/debug_client.h>
#include <rcsc/player/audio_sensor.h>
#include <rcsc/player/say_message_builder.h>

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


Bhv_ZhPass2::Bhv_ZhPass2() {

}

Bhv_ZhPass2::~Bhv_ZhPass2() {

}

bool Bhv_ZhPass2::execute(rcsc::PlayerAgent * agent) {
	rcsc::Vector2D ball = agent->world().ball().pos();

	const rcsc::WorldModel & wm = agent->world();

	int num = wm.self().unum();

	for (int i = 0; i < 20; i++) {
		opp[i] = rcsc::Vector2D(-52.5, 0.0);
		tmm[i] = rcsc::Vector2D(-52.5, 0.0);
		receiver[i] = -1;
		oppNo[i] = -1;
		oCount[i] = 100;
		tCount[i] = 100;
	}

	oMax = 0;
	tMax = 0;

	const rcsc::PlayerPtrCont::const_iterator o_end =
			wm.opponentsFromSelf().end();
	int i = 0;
//遍历离自己的对方球员
	for (rcsc::PlayerPtrCont::const_iterator o = wm.opponentsFromSelf().begin();
			o != o_end; ++o) {
		if ((*o)->posCount() > 80)
			continue;

		if ((*o)->pos().dist(ball) < 1.0)
			continue;
		//通过count限定获取较精确的对方球员位置及速度
		opp[i] = ((*o)->seenPosCount() <= (*o)->posCount() ?(*o)->seenPos() : (*o)->pos());
		rcsc::Vector2D ovel = ((*o)->seenVelCount() <= (*o)->velCount() ?(*o)->seenVel() : (*o)->vel());
		oppPlayerType[i] = (*o)->playerTypePtr();

		opp[i] += ovel;

		oppNo[i] = (*o)->unum();

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

		//与上同理
		tmm[j] = ((*t)->seenPosCount() <= (*t)->posCount() ?(*t)->seenPos() : (*t)->pos());
		rcsc::Vector2D tvel = ((*t)->seenVelCount() <= (*t)->velCount() ?(*t)->seenVel() : (*t)->vel());

		tmm[j] += tvel;

		tAngle[j] = (*t)->body().degree();

		tmmPlayerType[i] = (*t)->playerTypePtr();

		receiver[j] = (*t)->unum();

		tCount[j] = (*t)->posCount();

		j++;
	}

	tMax = j;

	for (int m = 0; m < tMax - 1; m++)
		for (int n = 0; n < tMax - 1 - m; n++) {
			double dist = tmm[n].dist(rcsc::Vector2D(52.0, 0.0));
			double distNext = tmm[n + 1].dist(rcsc::Vector2D(52.0, 0.0));

			if (dist > distNext) {
				rcsc::Vector2D tmp = tmm[n];
				tmm[n] = tmm[n + 1];
				tmm[n + 1] = tmp;

				int tmpNo = receiver[n];
				receiver[n] = receiver[n + 1];
				receiver[n + 1] = tmpNo;
			}
		}

	rcsc::Vector2D me = wm.self().pos();
	rcsc::Vector2D pos6 = wm.ourPlayer(6)->pos();
	rcsc::Vector2D pos7 = wm.ourPlayer(7)->pos();
	rcsc::Vector2D pos8 = wm.ourPlayer(8)->pos();
	rcsc::Vector2D pos9 = wm.ourPlayer(9)->pos();
	rcsc::Vector2D pos10 = wm.ourPlayer(10)->pos();
	rcsc::Vector2D pos11 = wm.ourPlayer(11)->pos();

	if (!agent->world().self().isKickable())
		return false;

	const rcsc::PlayerPtrCont & oppsz = wm.opponentsFromSelf();
	const rcsc::PlayerObject * nearest_opp =
			(oppsz.empty() ?
					static_cast<rcsc::PlayerObject *>(0) : oppsz.front());
	const double nearest_opp_dist = (
			nearest_opp ? nearest_opp->distFromSelf() : 1000.0);
	const rcsc::Vector2D nearest_opp_pos = (
			nearest_opp ? nearest_opp->pos() : rcsc::Vector2D(-1000.0, 0.0));

	rcsc::Vector2D target = rcsc::Vector2D(0.0, 0.0);

	//一系列传球
	if (CrossPass(agent, target)) {
		return true;
	}
	if (BrainCross2(agent, target)) {
		return true;
	}
	if (ThroughPass(agent, target)) {
		return true;
	}
	if (PowerfulThroughPass2011(agent, target)) {
		return true;
	}
	if (PowerfulThroughPass(agent, target)) {
		return true;
	}
	if (DirectPass(agent, target) && ball.x < 36.0) {
		return true;
	}
	if (LeadingPass2011(agent, target)) {
		return true;
	}

	agent->setNeckAction(new rcsc::Neck_TurnToLowConfTeammate());
	return false;
}

bool Bhv_ZhPass2::canPassToNo(rcsc::PlayerAgent * agent, int no) {
	return false;

	const rcsc::WorldModel & wm = agent->world();

	rcsc::Vector2D ball = wm.ball().pos();
	rcsc::Vector2D me = wm.self().pos();

	if (no == wm.self().unum())
		return false;

	int tmmCount = wm.ourPlayer(no)->posCount();
	rcsc::Vector2D tmmPos = wm.ourPlayer(no)->pos();
	float tmmBody = wm.ourPlayer(no)->body().degree();

	if (!tmmPos.isValid() || tmmCount > 2)
		return false;

	double z1 = 0;
	double z2 = 1000;
	double delCycle = -1000;
	rcsc::Vector2D target[20];
	double passSpeed = 2.5;

	for (int x = 0; x < 20; x++) {
		target[x] = rcsc::Vector2D(1000.0, 500.0);
	}
	//以队员的当前位置为基准,做20个target
	target[0] = tmmPos;
	target[1] = tmmPos + rcsc::Vector2D(2, 0);
	target[2] = tmmPos + rcsc::Vector2D(4, 0);
	target[3] = tmmPos + rcsc::Vector2D(6, 0);
	target[4] = tmmPos + rcsc::Vector2D(0, 2);
	target[5] = tmmPos + rcsc::Vector2D(0, 4);
	target[6] = tmmPos + rcsc::Vector2D(0, -2);
	target[7] = tmmPos + rcsc::Vector2D(0, -4);
	target[8] = tmmPos + rcsc::Vector2D(-2, 0);
	target[9] = tmmPos + rcsc::Vector2D(-4, 0);
	target[10] = tmmPos + rcsc::Vector2D(-6, 0);
	target[11] = tmmPos + rcsc::Vector2D(-8, 0);
	target[12] = tmmPos + rcsc::Vector2D(-4, 4);
	target[13] = tmmPos + rcsc::Vector2D(-4, -4);
	target[14] = tmmPos + rcsc::Vector2D(-6, -6);
	target[15] = tmmPos + rcsc::Vector2D(-6, 6);
	target[16] = tmmPos + rcsc::Vector2D(-4, -6);
	target[17] = tmmPos + rcsc::Vector2D(-4, 6);
	target[18] = tmmPos + rcsc::Vector2D(0, 7);
	target[19] = tmmPos + rcsc::Vector2D(0, -7);

	//判定20个target
	for (int kh = 0; kh < 20; kh++) {

		rcsc::Vector2D oneStepVel = rcsc::KickTable::calc_max_velocity(
				(target[kh] - ball).th(), agent->world().self().kickRate(),
				agent->world().ball().vel());
		double v0 = oneStepVel.r();

		return false;

		if (v0 < 1.0)
			continue;

		passSpeed = v0;

		rcsc::Vector2D tmptmm[1];
		tmptmm[0] = tmmPos;
		z1 = cycles(agent, target[kh], passSpeed, oMax, opp, 1);
		z2 = cycles(agent, target[kh], passSpeed, 1, tmptmm, 0);

		delCycle = z1 - z2;

		if (delCycle < 2 || delCycle > 50)
			continue;

//		std::cout << "你妹Cycle: ** " << wm.time().cycle()<< "你妹 CanPassToNo: " << no << " 你妹delCycle = " << delCycle<< " = " << z1 << " - " << z2 << " 你妹target: " << target[kh]<< " 你妹passSpeed = " << passSpeed << "\n";
		//踢一脚
		if (rcsc::Body_KickOneStep(target[kh], passSpeed).execute(agent)) {
			if (agent->config().useCommunication() && no != -1) {
				rcsc::Vector2D target_angle;
				target_angle.assign(0.0, 0.0);
				agent->addSayMessage(
						new rcsc::PassMessage(no, target[kh] + target_angle,
								agent->effector().queuedNextBallPos(),
								agent->effector().queuedNextBallVel()));
				agent->setNeckAction(new rcsc::Neck_TurnToLowConfTeammate());
			}
			return true;
		}
	}

	return false;
}
	//对teammate以离point的距离排序,放入nums数组中,当然没有守门员
rcsc::Vector2D Bhv_ZhPass2::getSortedTmm(rcsc::PlayerAgent * agent,
		rcsc::Vector2D point, int rank, int &num, int maxCount) {
	const rcsc::WorldModel & wm = agent->world();
	rcsc::Vector2D tmmz[10];
	int nums[10];
	int i = 0;

	const rcsc::PlayerPtrCont::const_iterator t_end =
			wm.teammatesFromSelf().end();
	for (rcsc::PlayerPtrCont::const_iterator t = wm.teammatesFromSelf().begin();
			t != t_end; ++t) {
		if ((*t)->posCount() > maxCount)
			continue;
		rcsc::Vector2D tPos = (
				(*t)->seenPosCount() <= (*t)->posCount() ?
						(*t)->seenPos() : (*t)->pos());
		rcsc::Vector2D tvel = (
				(*t)->seenVelCount() <= (*t)->velCount() ?
						(*t)->seenVel() : (*t)->vel());
		tPos += tvel;

		tmmz[i] = tPos;
		nums[i] = (*t)->unum();
		i++;
	}

	for (int kh = 1; kh < i; kh++)
		for (int j = 0; j < i - kh; j++) {
			if (tmmz[j].dist(point) > tmmz[j + 1].dist(point)) {
				int ntmp = nums[j + 1];
				nums[j + 1] = nums[j];
				nums[j] = ntmp;

				rcsc::Vector2D tmp = tmmz[j + 1];
				tmmz[j + 1] = tmmz[j];
				tmmz[j] = tmp;
			}
		}

	if (0 <= rank && rank < i) {
		num = nums[rank];
		return tmmz[rank];
	} else {
		num = -1;
		return rcsc::Vector2D(0.0, 0.0);
	}
}
//各种周期啊
int Bhv_ZhPass2::cycles(rcsc::PlayerAgent * agent, rcsc::Vector2D target,
		float speed, int i, rcsc::Vector2D o[], int x) {
	const rcsc::WorldModel & wm = agent->world();

	rcsc::Vector2D ball = wm.ball().pos();

	rcsc::AngleDeg passDir = (target - ball).dir();
	float ballSpeed = speed;
	int time = 0;
	ball += rcsc::Vector2D::polar2vector(ballSpeed,
			(target - ball).dir().degree());
	ballSpeed *= 0.94;
	time++;

	while (ballSpeed > 0.1 && time < 80 && ball.absX() < 54
			&& ball.absY() < 35.0) {
		int kickable = 1;

		if (x == 1) //遍历对手球员
				{
			const rcsc::PlayerPtrCont::const_iterator o_end =
					wm.opponentsFromSelf().end();
			for (rcsc::PlayerPtrCont::const_iterator o =
					wm.opponentsFromSelf().begin(); o != o_end; ++o) {
				const rcsc::PlayerType * player_type = (*o)->playerTypePtr();
				kickable = player_type->kickableArea();
				int oppCycles = 100;

				if ((*o)->posCount() > 40)
					continue;

				rcsc::Vector2D oPos = (
						(*o)->seenPosCount() <= (*o)->posCount() ?
								(*o)->seenPos() : (*o)->pos());
				rcsc::Vector2D ovel = (
						(*o)->seenVelCount() <= (*o)->velCount() ?
								(*o)->seenVel() : (*o)->vel());

				oPos += ovel;

				if ((*o)->goalie())
					kickable += 1.0;

				oppCycles = player_type->cyclesToReachDistance(
						oPos.dist(ball) - kickable);
				oppCycles -= rcsc::bound(0, (*o)->posCount(), 6);

				float oppBodyDir = std::fabs((*o)->body().degree());
				float oppVelDir = std::fabs((*o)->vel().dir().degree());
				float oppToCurrentBall = std::fabs(
						(ball - oPos).dir().degree());

				if (std::fabs(oppBodyDir - oppVelDir) < 20 && ovel.r() > 0.1
						&& std::fabs(oppBodyDir - oppToCurrentBall) > 40.0)
					oppCycles++;

				if (oppCycles <= time)
					return time;

			}
		} else if (x == 0) //遍历己方球员
				{
			const rcsc::PlayerPtrCont::const_iterator t_end =
					wm.teammatesFromSelf().end();
			for (rcsc::PlayerPtrCont::const_iterator t =
					wm.teammatesFromSelf().begin(); t != t_end; ++t) {
				const rcsc::PlayerType * player_type = (*t)->playerTypePtr();
				kickable = player_type->kickableArea();
				int tmmCycles = 100;

				if ((*t)->goalie())
					continue;

				if ((*t)->posCount() > 3)
					continue;

				rcsc::Vector2D tmmmPos = (
						(*t)->seenPosCount() <= (*t)->posCount() ?
								(*t)->seenPos() : (*t)->pos());

				rcsc::Vector2D tVel = (
						(*t)->seenVelCount() <= (*t)->velCount() ?
								(*t)->seenVel() : (*t)->vel());

				rcsc::Vector2D tPos = tmmmPos + tVel;

				if (tPos.dist(o[0]) >= kickable)
					continue;

				tmmCycles = player_type->cyclesToReachDistance(tPos.dist(ball));
				tmmCycles += rcsc::bound(0, (*t)->posCount(), 2);

				float tmmBodyDir = std::fabs((*t)->body().degree());
				float tmmVelDir = std::fabs((*t)->vel().dir().degree());
				float tmmToCurrentBall = std::fabs(
						(ball - tPos).dir().degree());

				if (std::fabs(tmmBodyDir - tmmVelDir) < 20 && tVel.r() > 0.1
						&& std::fabs(tmmBodyDir - tmmToCurrentBall) > 40.0)
					tmmCycles++;

				if (tmmCycles <= time)
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
//说了是各种周期~
int Bhv_ZhPass2::crossCycles(rcsc::PlayerAgent * agent, rcsc::Vector2D target,
		float speed, rcsc::Vector2D o[]) {

	const rcsc::WorldModel & wm = agent->world();

	rcsc::Vector2D ball = wm.ball().pos();

	rcsc::AngleDeg passDir = (target - ball).dir();
	float ballSpeed = speed;
	int time = 0;
	ball += rcsc::Vector2D::polar2vector(ballSpeed,
			(target - ball).dir().degree());
	ballSpeed *= 0.94;
	time++;

	while (ballSpeed > 0.3 && time < 60 && ball.absX() < 52.0
			&& ball.absY() < 33.0) {
		int kickable = 1;

		const rcsc::PlayerPtrCont::const_iterator o_end =
				wm.opponentsFromSelf().end();
		for (rcsc::PlayerPtrCont::const_iterator o =
				wm.opponentsFromSelf().begin(); o != o_end; ++o) {
			const rcsc::PlayerType * player_type = (*o)->playerTypePtr();
			kickable = player_type->kickableArea();
			int oppCycles = 100;

			if ((*o)->posCount() > 40)
				continue;

			rcsc::Vector2D oPos = ((*o)->seenPosCount() <= (*o)->posCount() ?(*o)->seenPos() : (*o)->pos());
			rcsc::Vector2D ovel = ((*o)->seenVelCount() <= (*o)->velCount() ?(*o)->seenVel() : (*o)->vel());

			oPos += ovel;

			if ((*o)->goalie())
				kickable += 1.0;

			oppCycles = player_type->cyclesToReachDistance(
					oPos.dist(ball) - kickable);
			oppCycles -= rcsc::bound(0, (*o)->posCount(), 3);

			int minTime = time - 1;

			if (minTime == 0)
				minTime++;

			if (oppCycles <= minTime)
				return time;
		}

		ball += rcsc::Vector2D::polar2vector(ballSpeed,
				(target - ball).dir().degree());
		ballSpeed *= 0.94;
		time++;
	}

	return 1000;
}

int Bhv_ZhPass2::leadingCycles(rcsc::PlayerAgent * agent, rcsc::Vector2D target,
		float speed, int tmmNum) {
	const rcsc::WorldModel & wm = agent->world();

	rcsc::Vector2D ball = wm.ball().pos();

	rcsc::AngleDeg passDir = (target - ball).dir();
	float ballSpeed = speed;
	int time = 0;
	ball += rcsc::Vector2D::polar2vector(ballSpeed,
			(target - ball).dir().degree());
	ballSpeed *= 0.94;
	time++;

	const rcsc::PlayerPtrCont::const_iterator t_end =
			wm.teammatesFromSelf().end();
	for (rcsc::PlayerPtrCont::const_iterator t = wm.teammatesFromSelf().begin();
			t != t_end; ++t) {
		if ((*t)->unum() != tmmNum)
			continue;

		const rcsc::PlayerType * player_type = (*t)->playerTypePtr();
		int kickable = 1;
		kickable = player_type->kickableArea();
		rcsc::Vector2D tPos = (*t)->pos() + (*t)->vel();

		while (ballSpeed > 0.15 && time < 60) {
			int tmmCycles = 100;
			tmmCycles = player_type->cyclesToReachDistance(
					tPos.dist(ball) - kickable);
			tmmCycles += rcsc::bound(0, (*t)->posCount(), 2);
			tmmCycles += (
					std::fabs(
							(*t)->body().degree()
									- (target - tPos).dir().degree()) <= 30.0 ?
							0 : 2);

			if (tmmCycles <= time)
				return time;

			ball += rcsc::Vector2D::polar2vector(ballSpeed,
					(target - ball).dir().degree());
			ballSpeed *= 0.94;
			time++;

		}
	}
	return 1000;
}
//应该是长传周期
int Bhv_ZhPass2::throughCycles(rcsc::PlayerAgent * agent, rcsc::Vector2D target,
		float speed, rcsc::Vector2D tmmPos) {
	const rcsc::WorldModel & wm = agent->world();

	rcsc::Vector2D ball = wm.ball().pos();

	rcsc::AngleDeg passDir = (target - ball).dir();
	float ballSpeed = speed;
	int time = 0;
	ball += rcsc::Vector2D::polar2vector(ballSpeed,
			(target - ball).dir().degree());
	ballSpeed *= 0.94;
	time++;

	const rcsc::PlayerPtrCont::const_iterator t_end =
			wm.teammatesFromSelf().end();
	for (rcsc::PlayerPtrCont::const_iterator t = wm.teammatesFromSelf().begin();
			t != t_end; ++t) {
		rcsc::Vector2D tPos = (*t)->pos() + (*t)->vel();

		if (tmmPos.dist(tPos) > 0.3)
			continue;

		if ((*t)->goalie())
			continue;

		if ((*t)->posCount() > 2)
			continue;

		const rcsc::PlayerType * player_type = (*t)->playerTypePtr();
		int kickable = 1;
		kickable = player_type->kickableArea();

		while (ballSpeed > 0.05 && time < 90 && ball.absX() < 52.5
				&& ball.absY() < 34.0) {
			if (ball.x > tPos.x + 4.7) {
				int tmmCycles = 100;
				tmmCycles = player_type->cyclesToReachDistance(
						tPos.dist(ball) - kickable);
				tmmCycles += rcsc::bound(0, (*t)->posCount(), 2);

				if (tmmCycles <= time)
					return time;
			}
			ball += rcsc::Vector2D::polar2vector(ballSpeed,
					(target - ball).dir().degree());
			ballSpeed *= 0.94;
			time++;

		}
	}
	return 1000;
}

int Bhv_ZhPass2::selfCyclesSRP(rcsc::PlayerAgent * agent, rcsc::Vector2D target,
		float speed) {
	const rcsc::WorldModel & wm = agent->world();

	rcsc::Vector2D ball = wm.ball().pos();

	rcsc::AngleDeg passDir = (target - ball).dir();
	float ballSpeed = speed;
	int time = 0;
	ball += rcsc::Vector2D::polar2vector(ballSpeed,
			(target - ball).dir().degree());
	ballSpeed *= 0.94;
	time++;

	while (ballSpeed > 0.05 && time < 80 && ball.absX() < 50.0
			&& ball.absY() < 31) {
		int kickable = 1;
		const rcsc::PlayerType * player_type = wm.self().playerTypePtr();
		kickable = player_type->kickableArea();
		int myCycles = 100;

		rcsc::Vector2D mPos = wm.self().pos() + wm.self().vel();

		myCycles = player_type->cyclesToReachDistance(mPos.dist(ball));
		myCycles += 1;

		if (myCycles <= time && time > 1)
			return time;

		ball += rcsc::Vector2D::polar2vector(ballSpeed,
				(target - ball).dir().degree());
		ballSpeed *= 0.94;
		time++;
	}

	return 1000;
}

int Bhv_ZhPass2::sign(float n) {
	if (n < 0)
		return -1;

	return 1;
}
//你妹的srpd
bool Bhv_ZhPass2::SRPD(rcsc::PlayerAgent * agent) {

	const rcsc::WorldModel & wm = agent->world();
	rcsc::Vector2D ball = wm.ball().pos();
	rcsc::Vector2D me = wm.self().pos();

	rcsc::Vector2D bala = rcsc::Vector2D::polar2vector(5.0,
			((rcsc::Vector2D(52.5, -10.0) - me).dir().degree()));
	rcsc::Vector2D paEn = rcsc::Vector2D::polar2vector(5.0,
			((rcsc::Vector2D(52.5, +10.0) - me).dir().degree()));

	if (wm.self().unum() <= 6)
		return false;

	if (ball.x < me.x)
		return false;

	if ((wm.getPointCount(bala, 5) > 4 || wm.getPointCount(paEn, 5) > 4)
			&& me.x < 25.0)
		return false;

	if (wm.self().stamina() < 3200.0)
		return false;

	if (me.x > 46)
		return false;

	double goal_angle = (rcsc::Vector2D(52.5, wm.self().pos().y)
			- wm.self().pos()).dir().degree();
	double difAngle = wm.self().body().degree() - goal_angle;
	if (difAngle > 80.0 || difAngle < -80.0)
		return false;

	rcsc::Vector2D p = ball + rcsc::Vector2D(20.0, 0.0);

	if (ball.y > 32.0)
		p.y = 32.0;
	if (ball.y < -32.0)
		p.y = -32.0;

	float v0[9] = { 1.6, 1.5, 1.4, 1.3, 1.2, 1.1, 1.0, 0.9, 0.75 };
	int q = 0;
	if (me.x > 34)
		q = 6;
	else if (me.x > 28)
		q = 4;
	else if (me.x > 22)
		q = 2;

	q--;
	q--;

	while (q < 8) {
		q++;
		int mCycles = selfCyclesSRP(agent, p, v0[q]);
		int oCycles = cycles(agent, p, v0[q], oMax, opp, 1);
		int delCycles = oCycles - mCycles;

		if (delCycles < 1)
			continue;

		rcsc::Vector2D one_step_vel = rcsc::KickTable::calc_max_velocity(
				(p - ball).dir().degree(), //direction
				agent->world().self().kickRate(), agent->world().ball().vel());
		double one_step_speed = one_step_vel.r();

		if (one_step_speed > v0[q]) {
			rcsc::Body_SmartKick(p, v0[q], v0[q] - 0.1, 1).execute(agent);

			agent->setNeckAction(new rcsc::Neck_TurnToBallOrScan());

			return true;
		}
	}
	return false;
}

bool Bhv_ZhPass2::SRPtoCenter(rcsc::PlayerAgent * agent) {
	const rcsc::WorldModel & wm = agent->world();
	rcsc::Vector2D ball = wm.ball().pos();
	rcsc::Vector2D me = wm.self().pos();

	rcsc::Vector2D bala = rcsc::Vector2D::polar2vector(5.0,
			((rcsc::Vector2D(50, -5.0) - me).dir().degree()));
	rcsc::Vector2D paEn = rcsc::Vector2D::polar2vector(5.0,
			((rcsc::Vector2D(50, +5.0) - me).dir().degree()));


	if (wm.self().unum() <= 6)
		return false;

	if (ball.x < me.x)
		return false;

	if ((wm.getPointCount(bala, 5) > 4 || wm.getPointCount(paEn, 5) > 4)
			&& me.x < 20.0)
		return false;

	if (wm.self().stamina() < 3200.0)
		return false;

	if (me.x > 48)
		return false;

	rcsc::Vector2D centerTarget[5];

	centerTarget[0] = rcsc::Vector2D(47.0, 0.0);
	centerTarget[1] = rcsc::Vector2D(52.0, 0.0);
	centerTarget[2] = rcsc::Vector2D(56.0, 0.0);
	centerTarget[3] = rcsc::Vector2D(60.0, 0.0);
	centerTarget[4] = rcsc::Vector2D(65.0, 0.0);

	int z = 5;

	if (ball.x > 38)
		z = 2;

	for (int i = 0; i < z; i++) {

		double goal_angle = (centerTarget[i] - wm.self().pos()).dir().degree();
		double difAngle = wm.self().body().degree() - goal_angle;
		if (difAngle > 80.0 || difAngle < -80.0)
			continue;


		rcsc::Vector2D p = me
				+ rcsc::Vector2D::polar2vector(20.0,
						(centerTarget[i] - me).dir());

		if (ball.y > 32.0)
			p.y = 32.0;
		if (ball.y < -32.0)
			p.y = -32.0;

		float v0[9] = { 1.5, 1.3, 1.2, 1.1, 1.05, 1.0, 0.95, 0.9, 0.85 };
		int q = 0;
		if (me.x > 38)
			q = 6;
		else if (me.x > 30)
			q = 4;
		else if (me.x > 25)
			q = 2;

		q--;
		q--;

		while (q < 8) {
			q++;
			int mCycles = selfCyclesSRP(agent, p, v0[q]);
			int oCycles = cycles(agent, p, v0[q], oMax, opp, 1);
			int delCycles = oCycles - mCycles;

			if (delCycles < 1)
				continue;

			rcsc::Vector2D one_step_vel = rcsc::KickTable::calc_max_velocity(
					(p - ball).dir().degree(), //direction
					agent->world().self().kickRate(),
					agent->world().ball().vel());
			double one_step_speed = one_step_vel.r();

			if (one_step_speed >= v0[q]) {
				rcsc::Body_SmartKick(p, v0[q], v0[q] - 0.1, 1).execute(agent);

				agent->setNeckAction(new rcsc::Neck_TurnToBallOrScan());


				return true;
			}
		}
	}
	return false;
}

bool Bhv_ZhPass2::SRPtoOutside(rcsc::PlayerAgent * agent) {
	const rcsc::WorldModel & wm = agent->world();
	rcsc::Vector2D ball = wm.ball().pos();
	rcsc::Vector2D me = wm.self().pos();

	rcsc::Vector2D bala = rcsc::Vector2D(me.x + 5, me.y * 0.8);
	rcsc::Vector2D paEn = rcsc::Vector2D(me.x + 3, me.y * 0.7);

	rcsc::Vector2D target[6];

	target[0] = rcsc::Vector2D(me.x + 10, me.y + rcsc::sign(me.y) * 3);
	target[1] = rcsc::Vector2D(me.x + 10, me.y + rcsc::sign(me.y) * 6);
	target[2] = rcsc::Vector2D(me.x + 10, me.y + rcsc::sign(me.y) * 8);
	target[3] = rcsc::Vector2D(me.x + 10, me.y + rcsc::sign(me.y) * 10);
	target[4] = rcsc::Vector2D(me.x + 10, me.y + rcsc::sign(me.y) * 14);
	target[5] = rcsc::Vector2D(me.x + 10, me.y + rcsc::sign(me.y) * 18);

	if (wm.self().unum() <= 6)
		return false;

	if (ball.x < me.x)
		return false;

	if ((wm.getPointCount(bala, 5) > 4 || wm.getPointCount(paEn, 5) > 4)
			&& me.x < 20.0)
		return false;

	if (wm.self().stamina() < 3200.0)
		return false;

	if (me.x > 46)
		return false;

	if (me.x > 37.0 && me.absY() < 10)
		return false;

	int z = 6;

	if (ball.x > 35.0)
		z = 2;
	if (ball.x > 30.0)
		z = 5;

	for (int i = 0; i < z; i++) {

		double goal_angle = (target[i] - wm.self().pos()).dir().degree();

		double difAngle = wm.self().body().degree() - goal_angle;

		if (difAngle > 60.0 || difAngle < -60.0)
			continue;

		rcsc::Vector2D p = me
				+ rcsc::Vector2D::polar2vector(20.0, (target[i] - me).dir());

		if (ball.y > 32.0)
			p.y = 32.0;
		if (ball.y < -32.0)
			p.y = -32.0;

		float v0[9] = { 1.8, 1.6, 1.5, 1.4, 1.3, 1.2, 1.0, 0.9, 0.8 };
		int q = 0;
		if (me.x > 34)
			q = 6;
		else if (me.x > 28)
			q = 4;
		else if (me.x > 22)
			q = 2;

		q--;
		q--;
		q--;

		while (q < 8) {
			q++;
			int mCycles = selfCyclesSRP(agent, p, v0[q]);
			int oCycles = cycles(agent, p, v0[q], oMax, opp, 1);
			int delCycles = oCycles - mCycles;

			if (delCycles < 1)
				continue;

			rcsc::Vector2D one_step_vel = rcsc::KickTable::calc_max_velocity(   //方向
					(p - ball).dir().degree(),
					agent->world().self().kickRate(),
					agent->world().ball().vel());
			double one_step_speed = one_step_vel.r();

			if (one_step_speed >= v0[q]) {
				rcsc::Body_SmartKick(p, v0[q], v0[q] - 0.1, 1).execute(agent);

				agent->setNeckAction(new rcsc::Neck_TurnToBallOrScan());

				return true;
			}
		}

	}

	return false;
}
//接下来是具体传球部分
bool Bhv_ZhPass2::BrainCross2(rcsc::PlayerAgent * agent,
		rcsc::Vector2D & target) {

	const rcsc::WorldModel & wm = agent->world();
	rcsc::Vector2D ball = wm.ball().pos();
	rcsc::Vector2D me = wm.self().pos();

	if (ball.x < 32.0)
		return false;

	if (agent->world().gameMode().type() != rcsc::GameMode::PlayOn)
		return false;

	if ((wm.self().unum() == 9 || wm.self().unum() == 10) && ball.x < 43.0
			&& ball.absY() > 19.0)
		return false;

	for (int i = 0; i < tMax; i++) {
		int receiverz = -1;
		rcsc::Vector2D tmmz = getSortedTmm(agent, rcsc::Vector2D(52.5, 0.0), i,
				receiverz, 1);

		if (tmmz.x == 0.0 && tmmz.y == 0.0)
			continue;

		if (tmmz.x < 32.0 || wm.offsideLineX() < 40.0)
			continue;

		if (tmmz.x > wm.offsideLineX() - 0.1)
			continue;

		if (receiverz == 9 && wm.self().unum() == 7 && tmmz.absY() > 12.0)
			continue;

		if (receiverz == 10 && wm.self().unum() == 8 && tmmz.absY() > 12.0)
			continue;

		if (wm.self().unum() == 11
				&& ((me.absY() < 10.0 && tmmz.absY() > 10.0)
						|| (me.absY() < 13.5 && tmmz.x < me.x - 4.0)))
			continue;

		if ((wm.self().unum() == 7 || wm.self().unum() == 8)
				&& (receiverz == 8 || receiverz == 7) && ball.x < 40.0
				&& tmmz.dist(me) < 4.0)
			continue;

		if (((wm.self().unum() == 9 && receiverz == 7)
				|| (wm.self().unum() == 10 && receiverz == 8))
				&& (ball.x < 44.0 || tmmz.x < 35.0))
			continue;

		for (int t = 0; t >= -3; t--) {
			if (std::fabs(ball.y - tmmz.y) < 8.0 && (t == -1 || t == -2))
				continue;
			if (std::fabs(ball.y - tmmz.y) < 12.0 && t == -3)
				continue;

			float z = t;

			if (z == -2)
				z = 1;
			if (z == -3)
				z = 2;

			rcsc::Vector2D targ = tmmz + rcsc::Vector2D::polar2vector(z, 0);

			if (targ.x > 52.0)
				continue;

			if (ball.dist(targ) < 3.0)
				continue;

			if (me.absY() > 16.0 && me.dist(targ) < 6.0)
				continue;

			rcsc::Vector2D oneStepVel = rcsc::KickTable::calc_max_velocity(
					(targ - ball).th(), agent->world().self().kickRate(),
					agent->world().ball().vel());
			double v0 = oneStepVel.r();

			rcsc::Vector2D tmptmm[1];

			tmptmm[0] = tmmz;

			float speed[9];
			int delCycle[9];

			speed[0] = 3.0;
			speed[1] = 2.8;
			speed[2] = 2.6;
			speed[3] = 2.45;
			speed[4] = 2.3;
			speed[5] = 2.15;
			speed[6] = 2.0;
			speed[7] = 1.85;
			speed[8] = v0;

			for (int kh = 0; kh < 9; kh++) {
				speed[kh] = std::min(3.0, double(speed[kh]));
				delCycle[kh] = cycles(agent, targ, speed[kh], oMax, opp, 1)
						- cycles(agent, targ, speed[kh], 1, tmptmm, 0);

			}

			target = targ;

			if (delCycle[8] > 0) {

				rcsc::Body_KickOneStep(target, speed[0]).execute(agent);

				if (agent->config().useCommunication() && receiver[i] != -1) {
					rcsc::Vector2D target_angle = target - tmmz;
					target_angle.setLength(1.0);
					agent->addSayMessage(
							new rcsc::PassMessage(receiverz,
									target + target_angle,
									agent->effector().queuedNextBallPos(),
									agent->effector().queuedNextBallVel()));
					agent->setNeckAction(
							new rcsc::Neck_TurnToLowConfTeammate());
				}
				return true;

			}

			for (int x = 0; x < 9; x++) {
				if (delCycle[x] < 1)
					continue;

				if (!rcsc::Body_SmartKick(target, speed[x], speed[x] * 0.96, 3).execute(
						agent))
					continue;
				else {
					if (agent->config().useCommunication()
							&& receiver[i] != -1) {
						rcsc::Vector2D target_angle = target - tmmz;
						target_angle.setLength(1.0);
						agent->addSayMessage(
								new rcsc::PassMessage(receiverz,
										target + target_angle,
										agent->effector().queuedNextBallPos(),
										agent->effector().queuedNextBallVel()));
						agent->setNeckAction(
								new rcsc::Neck_TurnToLowConfTeammate());
					}
				}
				return true;
			}
		}
	}
	return false;
}

bool Bhv_ZhPass2::CrossPass(rcsc::PlayerAgent * agent,
		rcsc::Vector2D & target) {

	const rcsc::WorldModel & wm = agent->world();
	rcsc::Vector2D ball = wm.ball().pos();

	if (ball.x < 26 || std::fabs(ball.y) < 5)
		return false;

	if (agent->world().gameMode().type() != rcsc::GameMode::PlayOn)
		return false;

	for (int i = 0; i < tMax; i++) {

		if (tmm[i].x < 30)
			continue;

		if (ball.y > 0)
			if (tmm[i].y > ball.y - 2)
				continue;
		if (ball.y < 0)
			if (tmm[i].y < ball.y + 2)
				continue;

		if (tmm[i].x > wm.offsideLineX())
			continue;

		if (tCount[i] > 2)
			continue;

		//好吧 这次是30个target
		rcsc::Vector2D targ[31];

		targ[0] = tmm[i] + rcsc::Vector2D(-3, 0);
		targ[1] = tmm[i] + rcsc::Vector2D(-2, 0);
		targ[2] = tmm[i] + rcsc::Vector2D(-1, 0);
		targ[3] = tmm[i] + rcsc::Vector2D(0, 0);
		targ[4] = tmm[i] + rcsc::Vector2D(1, 0);
		targ[5] = tmm[i] + rcsc::Vector2D(2, 0);
		targ[6] = tmm[i] + rcsc::Vector2D(3, 0);
		targ[7] = tmm[i] + rcsc::Vector2D(4, 0);
		targ[8] = tmm[i] + rcsc::Vector2D(5, 0);
		targ[9] = tmm[i] + rcsc::Vector2D(6, 0);
		targ[10] = tmm[i] + rcsc::Vector2D(7, 0);
		targ[11] = tmm[i] + rcsc::Vector2D(-1, -1);
		targ[12] = tmm[i] + rcsc::Vector2D(0, -1);
		targ[13] = tmm[i] + rcsc::Vector2D(1, -1);
		targ[14] = tmm[i] + rcsc::Vector2D(2, -1);
		targ[15] = tmm[i] + rcsc::Vector2D(-2, -2);
		targ[16] = tmm[i] + rcsc::Vector2D(0, -2);
		targ[17] = tmm[i] + rcsc::Vector2D(1, -2);
		targ[18] = tmm[i] + rcsc::Vector2D(2, -2);
		targ[19] = tmm[i] + rcsc::Vector2D(0, -3);
		targ[20] = tmm[i] + rcsc::Vector2D(3, -3);
		targ[21] = tmm[i] + rcsc::Vector2D(-1, 1);
		targ[22] = tmm[i] + rcsc::Vector2D(0, 1);
		targ[23] = tmm[i] + rcsc::Vector2D(1, 1);
		targ[24] = tmm[i] + rcsc::Vector2D(2, 1);
		targ[25] = tmm[i] + rcsc::Vector2D(-2, 2);
		targ[26] = tmm[i] + rcsc::Vector2D(0, 2);
		targ[27] = tmm[i] + rcsc::Vector2D(1, 2);
		targ[28] = tmm[i] + rcsc::Vector2D(2, 2);
		targ[29] = tmm[i] + rcsc::Vector2D(0, 3);
		targ[30] = tmm[i] + rcsc::Vector2D(3, 3);

		// targets 排序
		for (int m = 0; m < 40; m++)
			for (int n = 0; n < 40 - m; n++) {
				double dist = targ[n].dist(rcsc::Vector2D(52.0, 0.0));
				double distNext = targ[n + 1].dist(rcsc::Vector2D(52.0, 0.0));

				if (dist > distNext) {
					rcsc::Vector2D tmp = targ[n];
					targ[n] = targ[n + 1];
					targ[n + 1] = tmp;
				}
			}

		for (int t = 0; t < 31; t++) {

			if (targ[t].x > 52)
				continue;
			if (targ[t].x < 36)
				continue;

			double sum = 0;

			const rcsc::PlayerType * player_type =
					wm.ourPlayer(receiver[i])->playerTypePtr();

			int tTmmToTarget = player_type->cyclesToReachDistance(
					targ[t].dist(tmm[i]));

			if (receiver[i] == -1)
				tTmmToTarget = targ[t].dist(tmm[i]);

			double ballToTarget = targ[t].dist(ball);

			for (int j = 0; j <= tTmmToTarget; j++)
				sum += std::pow(0.94, j);

			float v0 = ballToTarget / sum;


			if (v0 > 3.0)
				continue;


			if (v0 < 1.0)
				continue;

			rcsc::Vector2D tmptmm[1];

			tmptmm[0] = tmm[i];

			int delCycle = crossCycles(agent, targ[t], v0, opp)
					- cycles(agent, targ[t], v0, 1, tmptmm, 0);

			if (delCycle < 1)
				continue;

			target = targ[t];

			int kick_step = (
					agent->world().gameMode().type() != rcsc::GameMode::PlayOn
							&& agent->world().gameMode().type()
									!= rcsc::GameMode::GoalKick_ ? 1 : 3);

			rcsc::Vector2D one_step_vel = rcsc::KickTable::calc_max_velocity(
					(target - agent->world().ball().pos()).th(),
					agent->world().self().kickRate(),
					agent->world().ball().vel());
			double one_step_speed = one_step_vel.r();

			if (one_step_speed >= v0 * 0.98) {
				rcsc::Body_KickOneStep(target, v0).execute(agent);

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
//长传部分
bool Bhv_ZhPass2::ThroughPass(rcsc::PlayerAgent * agent,
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

		if (ball.x > wm.offsideLineX() - 1.5)
			continue;

		for (int t = 30; t > 5; t--) //t,z大小具体可商量
			for (int z = 0; z < 7; z++)
					{

				rcsc::Vector2D targ =
						tmm[i]
								+ rcsc::Vector2D::polar2vector(t,
										(rcsc::Vector2D(52.5, tmm[i].y) - tmm[i]).dir().degree()
												+ sign(tmm[i].y) * (z * 10));

				if (targ.absX() > 46.0)
					continue;

				int extraxX = 5.0;

				if (ball.x > wm.offsideLineX() - 8.0)
					extraxX = 1.0;

				if (wm.getPointCount(
						rcsc::Vector2D(tmm[i].x + extraxX, tmm[i].y * 0.96), 10)
						> 2)
					continue;

				if (targ.y > 31.0)
					targ.y = 31.0;
				if (targ.y < -31.0)
					targ.y = -31.0;

				if (tmm[i].x < -5.0)
					continue;

				if (receiver[i] < 7)
					continue;

				if (targ.x < wm.offsideLineX() - 15.0)
					continue;

				if (targ.dist(rcsc::Vector2D(52.5, 0.0)) < 9.0)
					continue;

				if (targ.x > 44.0 && targ.absY() < 12.0)
					continue;

				double sum = 0;

				const rcsc::PlayerType * player_type =
						wm.ourPlayer(receiver[i])->playerTypePtr();

				int tTmmToTarget = player_type->cyclesToReachDistance(
						targ.dist(tmm[i])) + 1;

				if (receiver[i] == -1)
					tTmmToTarget = targ.dist(tmm[i]);

				double ballToTarget = targ.dist(ball);

				for (int j = 0; j <= tTmmToTarget; j++)
					sum += std::pow(0.94, j);

				float v0 = ballToTarget / sum;

				if (v0 > 3.1)
					continue;

				v0 = std::min(float(2.96), v0);

				rcsc::Vector2D tmptmm[1];

				tmptmm[0] = tmm[i];

				double z1 = cycles(agent, targ, v0, oMax, opp, 1);
				double z2 = throughCycles(agent, targ, v0, tmm[i]);

				double delCycle = z1 - z2;

				if (receiver[i] < 9 && delCycle < 2)
					continue;

				if (delCycle < 1)
					continue;

				rcsc::Vector2D one_step_vel =
						rcsc::KickTable::calc_max_velocity(
								(targ - agent->world().ball().pos()).th(),
								agent->world().self().kickRate(),
								agent->world().ball().vel());
				double one_step_speed = one_step_vel.r();

				target = targ;

				int kick_step = (
						agent->world().gameMode().type()
								!= rcsc::GameMode::PlayOn
								&& agent->world().gameMode().type()
										!= rcsc::GameMode::GoalKick_ ? 1 : 3);

				if (one_step_speed >= v0 * 0.98) {
					rcsc::Body_KickOneStep(target, v0).execute(agent);

					if (agent->config().useCommunication()
							&& receiver[i] != -1) {
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
//更长的长传
bool Bhv_ZhPass2::PowerfulThroughPass(rcsc::PlayerAgent * agent,
		rcsc::Vector2D & target) {

	const rcsc::WorldModel & wm = agent->world();
	rcsc::Vector2D ball = wm.ball().pos();

	if (wm.offsideLineX() > 30.0)
		return false;

	if (ball.x < -10.0)
		return false;

	for (int i = 0; i < tMax; i++) {

		if (tmm[i].x < wm.offsideLineX() - 5.0)
			continue;

		if (tmm[i].x > wm.offsideLineX() + 0.2)
			continue;

		if (ball.x > wm.offsideLineX() - 1.0)
			continue;

		if (ball.x < wm.offsideLineX() - 15.0)
			continue;

		if (tCount[i] > 1)
			continue;

		if (ball.absY() > 25.0)
			continue;

		bool isTmmUp = true;
		bool isTmmDown = true;

		const rcsc::PlayerPtrCont::const_iterator o_end =
				wm.opponentsFromSelf().end();
		for (rcsc::PlayerPtrCont::const_iterator o =
				wm.opponentsFromSelf().begin(); o != o_end; ++o) {
			rcsc::Vector2D oppx = (*o)->pos() + (*o)->vel();

			if ((*o)->goalie())
				continue;

			if (oppx.x < wm.offsideLineX() - 10.0)
				continue;

			if ((*o)->posCount() > 20)
				continue;

			if (tmm[i].y < oppx.y)
				isTmmDown = false;
			if (tmm[i].y > oppx.y)
				isTmmUp = false;

		}

		if (!isTmmUp && !isTmmDown)
			continue;

		if (isTmmUp && isTmmDown)
			continue;

		float plusX = 27.0;
		if (wm.offsideLineX() > 15.0)
			plusX = 27.0;
		if (wm.offsideLineX() > 20.0)
			plusX = 24.0;
		if (wm.offsideLineX() > 25.0)
			plusX = 22.0;
		if (wm.offsideLineX() > 33.0)
			continue;

		target = rcsc::Vector2D(tmm[i].x + plusX, tmm[i].y * 1.4);

		if (target.x + plusX > 45.0)
			plusX = 45.0 - target.x;

		target.y = std::max(-31.0, target.y);
		target.y = std::min(31.0, target.y);

		float v0 = 3.0;

		rcsc::Vector2D vel30 = rcsc::Vector2D::polar2vector(3.0,
				(target - ball).dir());
		rcsc::Vector2D vel285 = rcsc::Vector2D::polar2vector(2.85,
				(target - ball).dir());
		rcsc::Vector2D vel27 = rcsc::Vector2D::polar2vector(2.7,
				(target - ball).dir());
		rcsc::Vector2D vel255 = rcsc::Vector2D::polar2vector(2.55,
				(target - ball).dir());
		rcsc::Vector2D vel24 = rcsc::Vector2D::polar2vector(2.4,
				(target - ball).dir());
		rcsc::Vector2D vel225 = rcsc::Vector2D::polar2vector(2.25,
				(target - ball).dir());
		rcsc::Vector2D vel21 = rcsc::Vector2D::polar2vector(2.1,
				(target - ball).dir());

		int tmmCycles30 = throughCycles(agent, target, 3.0, tmm[i]) + 2;
		int tmmCycles285 = throughCycles(agent, target, 2.85, tmm[i]) + 2;
		int tmmCycles27 = throughCycles(agent, target, 2.7, tmm[i]) + 2;
		int tmmCycles255 = throughCycles(agent, target, 2.55, tmm[i]) + 2;
		int tmmCycles24 = throughCycles(agent, target, 2.4, tmm[i]) + 2;
		int tmmCycles225 = throughCycles(agent, target, 2.25, tmm[i]) + 2;
		int tmmCycles21 = throughCycles(agent, target, 2.1, tmm[i]) + 2;

		rcsc::Vector2D pv30 = rcsc::inertia_n_step_point(ball, vel30,
				tmmCycles30, rcsc::ServerParam::i().ballDecay());
		rcsc::Vector2D pv285 = rcsc::inertia_n_step_point(ball, vel285,
				tmmCycles285, rcsc::ServerParam::i().ballDecay());
		rcsc::Vector2D pv27 = rcsc::inertia_n_step_point(ball, vel27,
				tmmCycles27, rcsc::ServerParam::i().ballDecay());
		rcsc::Vector2D pv255 = rcsc::inertia_n_step_point(ball, vel255,
				tmmCycles255, rcsc::ServerParam::i().ballDecay());
		rcsc::Vector2D pv24 = rcsc::inertia_n_step_point(ball, vel24,
				tmmCycles24, rcsc::ServerParam::i().ballDecay());
		rcsc::Vector2D pv225 = rcsc::inertia_n_step_point(ball, vel225,
				tmmCycles225, rcsc::ServerParam::i().ballDecay());
		rcsc::Vector2D pv21 = rcsc::inertia_n_step_point(ball, vel21,
				tmmCycles21, rcsc::ServerParam::i().ballDecay());

		if (pv30.absY() < 31.0 && pv30.x < 50.0)
			v0 = 3.0;
		else if (pv285.absY() < 31.0 && pv285.x < 50.0)
			v0 = 2.85;
		else if (pv27.absY() < 31.0 && pv27.x < 50.0)
			v0 = 2.7;
		else if (pv255.absY() < 31.0 && pv255.x < 50.0)
			v0 = 2.55;
		else if (pv24.absY() < 31.0 && pv24.x < 50.0)
			v0 = 2.4;
		else if (pv225.absY() < 31.0 && pv225.x < 50.0)
			v0 = 2.25;
		else if (pv21.absY() < 31.0 && pv21.x < 50.0)
			v0 = 2.1;

		bool oppCatches = false;

		int tmmCycles = 10001;

		if (v0 > 2.9)
			tmmCycles = tmmCycles30;
		else if (v0 > 2.75)
			tmmCycles = tmmCycles285;
		else if (v0 > 2.6)
			tmmCycles = tmmCycles27;
		else if (v0 > 2.45)
			tmmCycles = tmmCycles255;
		else if (v0 > 2.3)
			tmmCycles = tmmCycles24;
		else if (v0 > 2.15)
			tmmCycles = tmmCycles225;
		else if (v0 > 2.0)
			tmmCycles = tmmCycles21;

		for (rcsc::PlayerPtrCont::const_iterator o =
				wm.opponentsFromSelf().begin(); o != o_end; ++o) {
			rcsc::Vector2D oppx = (*o)->pos() + (*o)->vel();

			if ((*o)->goalie())
				continue;

			if ((*o)->posCount() > 20)
				continue;

			double ballSpeed = v0;
			int time = 1;
			rcsc::Vector2D thisBall = ball;
			thisBall += rcsc::Vector2D::polar2vector(ballSpeed,
					(target - ball).dir().degree());
			ballSpeed *= 0.94;

			while (time < tmmCycles && ballSpeed > 0.3) {
				double minDist = std::max(1.1, std::min(double(time - 1), 7.0));

				if (thisBall.dist(oppx) < minDist) {
					oppCatches = true;
					break;
				}

				thisBall += rcsc::Vector2D::polar2vector(ballSpeed,
						(target - ball).dir().degree());
				ballSpeed *= 0.94;
				time++;
			}

		}

		if (oppCatches)
			continue;

		rcsc::Vector2D one_step_vel = rcsc::KickTable::calc_max_velocity(
				(target - agent->world().ball().pos()).th(),
				agent->world().self().kickRate(), agent->world().ball().vel());
		double one_step_v0 = one_step_vel.r();

		if (one_step_v0 > v0) {
			if (rcsc::Body_KickOneStep(target, v0).execute(agent)) {
			} else
				return false;
		} else {
			if (rcsc::Body_SmartKick(target, v0, v0 * 0.96, 3).execute(agent)) {
			} else
				return false;
		}

		if (agent->config().useCommunication() && receiver[i] != -1) {
			rcsc::Vector2D target_angle = target - tmm[i];
			target_angle.setLength(1.0);
			agent->addSayMessage(
					new rcsc::PassMessage(receiver[i], target + target_angle,
							agent->effector().queuedNextBallPos(),
							agent->effector().queuedNextBallVel()));
			agent->setNeckAction(new rcsc::Neck_TurnToLowConfTeammate());
		}
		return true;

	}
	return false;
}
//这个...其实是最新的
bool Bhv_ZhPass2::PowerfulThroughPass2011(rcsc::PlayerAgent * agent,
		rcsc::Vector2D & target) {

	const rcsc::WorldModel & wm = agent->world();
	rcsc::Vector2D ball = wm.ball().pos();

	if (wm.offsideLineX() > 30.0)
		return false;

	if (ball.x < -10.0)
		return false;

	for (int i = 0; i < tMax; i++) {

		if (tmm[i].x < wm.offsideLineX() - 5.0)
			continue;

		if (tmm[i].x > wm.offsideLineX() + 0.1)
			continue;

		if (ball.x > wm.offsideLineX() - 1.0)
			continue;

		if (ball.x < wm.offsideLineX() - 17.0)
			continue;

		if (tCount[i] > 1)
			continue;

		for (int t = 30; t > 5; t--)
			for (int z = 0; z < 7; z++)
					{
				rcsc::Vector2D targ =
						tmm[i]
								+ rcsc::Vector2D::polar2vector(t,
										(rcsc::Vector2D(52.5, tmm[i].y) - tmm[i]).dir().degree()
												+ sign(tmm[i].y) * (z * 10));

				if (targ.absX() > 46.0)
					continue;

				int extraxX = 5.0;

				if (ball.x > wm.offsideLineX() - 8.0)
					extraxX = 1.0;

				if (wm.getPointCount(
						rcsc::Vector2D(tmm[i].x + extraxX, tmm[i].y * 0.96), 10)
						> 2)
					continue;

				if (targ.y > 31.0)
					targ.y = 31.0;
				if (targ.y < -31.0)
					targ.y = -31.0;

				if (tmm[i].x < -5.0)
					continue;

				if (receiver[i] < 7)
					continue;

				if (targ.x < wm.offsideLineX() - 15.0)
					continue;

				if (targ.dist(rcsc::Vector2D(52.5, 0.0)) < 9.0)
					continue;

				if (targ.x > 44.0 && targ.absY() < 9.0)
					continue;

				if (targ.absY() < ball.absY()
						&& wm.ourPlayer(receiver[i])->vel().x < 0.1)
					continue;

				double sum = 0;

				const rcsc::PlayerType * player_type =
						wm.ourPlayer(receiver[i])->playerTypePtr();

				int tTmmToTarget = player_type->cyclesToReachDistance(
						targ.dist(tmm[i])) + 1;

				if (receiver[i] == -1)
					tTmmToTarget = targ.dist(tmm[i]);

				double ballToTarget = targ.dist(ball);

				for (int j = 0; j <= tTmmToTarget; j++)
					sum += std::pow(0.94, j);

				float v0 = ballToTarget / sum;

				if (v0 > 3.1)
					continue;

				v0 = std::min(float(2.96), v0);

				rcsc::Vector2D tmptmm[1];

				tmptmm[0] = tmm[i];

				double tmmCycles = tTmmToTarget;

				if (tmmCycles == 1000)
					continue;

				bool oppCatches = false;

				const rcsc::PlayerPtrCont::const_iterator o_end =
						wm.opponentsFromSelf().end();
				for (rcsc::PlayerPtrCont::const_iterator o =
						wm.opponentsFromSelf().begin(); o != o_end; ++o) {
					rcsc::Vector2D oPos = (
							(*o)->seenPosCount() <= (*o)->posCount() ?
									(*o)->seenPos() : (*o)->pos());
					rcsc::Vector2D ovel = (
							(*o)->seenVelCount() <= (*o)->velCount() ?
									(*o)->seenVel() : (*o)->vel());

					rcsc::Vector2D oppx = oPos + ovel;

					if ((*o)->posCount() > 20)
						continue;

					double ballSpeed = v0;
					int time = 1;
					rcsc::Vector2D thisBall = ball;
					thisBall += rcsc::Vector2D::polar2vector(ballSpeed,
							(targ - ball).dir().degree());
					ballSpeed *= 0.94;

					const rcsc::PlayerType * player_type =
							(*o)->playerTypePtr();
					double kickable = player_type->kickableArea();

					int oppCycles = 1000;

					while (time < tmmCycles && ballSpeed > 0.3) {

						if ((*o)->goalie()) {
							kickable += 0.7;

							oppCycles = player_type->cyclesToReachDistance(
									oppx.dist(thisBall) - kickable);
							oppCycles -= rcsc::bound(0, (*o)->posCount(), 3);

							if (oppCycles <= time) {
								oppCatches = true;
								break;
							}

						}

						double minDist = std::max(1.1,
								std::min(double(time - 1), 10.0));

						if (time == 2)
							minDist = 1.8;
						if (time == 3)
							minDist = 2.5;

						if (thisBall.dist(oppx) < minDist) {
							oppCatches = true;
							break;
						}

						thisBall += rcsc::Vector2D::polar2vector(ballSpeed,
								(targ - ball).dir().degree());
						ballSpeed *= 0.94;
						time++;
					}
				}

				if (oppCatches)
					continue;

				if (receiver[i] < 9)
					continue;

				rcsc::Vector2D one_step_vel =
						rcsc::KickTable::calc_max_velocity(
								(targ - agent->world().ball().pos()).th(),
								agent->world().self().kickRate(),
								agent->world().ball().vel());
				double one_step_speed = one_step_vel.r();

				target = targ;

				int kick_step = (
						agent->world().gameMode().type()
								!= rcsc::GameMode::PlayOn
								&& agent->world().gameMode().type()
										!= rcsc::GameMode::GoalKick_ ? 1 : 3);

				if (one_step_speed >= v0 * 0.98) {
					rcsc::Body_KickOneStep(target, v0).execute(agent);

					if (agent->config().useCommunication()
							&& receiver[i] != -1) {
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
//					std::cout << "你妹Cycle: " << wm.time().cycle()<< receiver[i] << " 你妹target: " << targ<< " 你妹tmmCycles: " << tmmCycles << "\n";
					return true;
				}

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
						rcsc::Body_KickOneStep(targ, v0).execute(agent);
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

//				std::cout << "你妹Cycle: " << wm.time().cycle()<< receiver[i] << " 你妹target: " << targ << " 你妹tmmCycle: "<< tmmCycles << "\n";
				return true;

			}

	}
	return false;
}
//长传end
bool Bhv_ZhPass2::LeadingPass(rcsc::PlayerAgent * agent,
		rcsc::Vector2D & target) {
	const rcsc::WorldModel & wm = agent->world();
	rcsc::Vector2D ball = wm.ball().pos();

	if (ball.x > wm.offsideLineX() - 0.5)
		return false;

	int num = 0;
	rcsc::Vector2D point = rcsc::Vector2D(52.5, 0.0);

	for (int i = 0; num != -1; i++) {
		rcsc::Vector2D tmmz = getSortedTmm(agent, point, i, num, 3);

		if (num == -1)
			break;

		if (num < 7)
			continue;

//    if (tmmz.x < 0)
//         continue;

		if (tmmz.x > wm.offsideLineX())
			continue;

		int z[] = { 0, 15, -15, 30, -30, 60, -60, 90, -90 };

		for (int r = 4; r <= 10; r += 2)
			for (int j = 9; j >= 0; j--) {
				rcsc::Vector2D targ =
						tmmz
								+ rcsc::Vector2D::polar2vector(r,
										(rcsc::Vector2D(-52.5, tmmz.y) - tmmz).dir().degree()
												+ sign(tmmz.y) * (z[j]));

				if (targ.absY() > 31 || targ.absX() > 49)
					continue;

				if (wm.getPointCount(targ, 10) > 4)
					continue;

				float v0 = (targ.dist(ball)
						- 0.5 * -(0.0582) * (r + 2.0) * (r + 2.0)) / (r + 2.0);
				if (v0 > 3.1)
					continue;

				v0 = std::min(float(2.96), v0);

				rcsc::Vector2D tmptmm[1];

				tmptmm[0] = tmmz;

				double z1 = cycles(agent, targ, v0, oMax, opp, 1);
				double z2 = leadingCycles(agent, targ, v0, num);

				double delCycle = z1 - z2;

				if (delCycle < 1)
					continue;

				rcsc::Vector2D one_step_vel =
						rcsc::KickTable::calc_max_velocity(
								(targ - agent->world().ball().pos()).th(),
								agent->world().self().kickRate(),
								agent->world().ball().vel());
				double one_step_speed = one_step_vel.r();

				target = targ;

				int kick_step = (
						agent->world().gameMode().type()
								!= rcsc::GameMode::PlayOn
								&& agent->world().gameMode().type()
										!= rcsc::GameMode::GoalKick_ ? 1 : 3);

				if (one_step_speed >= v0 * 0.96) {
					rcsc::Body_KickOneStep(target, v0).execute(agent);

					if (agent->config().useCommunication() && num != -1) {
						rcsc::Vector2D target_angle = target - tmmz;
						target_angle.setLength(1.0);
						agent->addSayMessage(
								new rcsc::PassMessage(num,
										target + target_angle,
										agent->effector().queuedNextBallPos(),
										agent->effector().queuedNextBallVel()));
						agent->setNeckAction(
								new rcsc::Neck_TurnToLowConfTeammate());
					}
					return true;
				} else if (!rcsc::Body_SmartKick(target, v0, v0 * 0.96,
						kick_step).execute(agent)) {
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

				if (agent->config().useCommunication() && num != -1) {
					rcsc::Vector2D target_angle = target - tmmz;
					target_angle.setLength(1.0);
					agent->addSayMessage(
							new rcsc::PassMessage(num, target + target_angle,
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

bool Bhv_ZhPass2::LeadingPass2011(rcsc::PlayerAgent * agent,
		rcsc::Vector2D & targetzz) {
	const rcsc::WorldModel & wm = agent->world();

	rcsc::Vector2D ball = wm.ball().pos();
	rcsc::Vector2D me = wm.self().pos();

	if (ball.x > wm.offsideLineX() - 0.5)
		return false;

	if (ball.x > 36 && ball.absY() < 20.0)
		return false;

	for (int i = 0; i < tMax; i++) {
		int no = -1;
		rcsc::Vector2D tmmz = getSortedTmm(agent, rcsc::Vector2D(52.5, 0.0), i,
				no, 2);

		const rcsc::PlayerPtrCont::const_iterator t_end =
				wm.teammatesFromSelf().end();
		for (rcsc::PlayerPtrCont::const_iterator t =
				wm.teammatesFromSelf().begin(); t != t_end; ++t) {

			if (no != (*t)->unum())
				continue;

			if ((*t)->unum() == wm.self().unum())
				continue;

			const rcsc::PlayerType * player_type = (*t)->playerTypePtr();
			float kickable = player_type->kickableArea();
			int tmmCycles = 100;

			if ((*t)->goalie())
				continue;

			if ((*t)->posCount() > 2)
				continue;

			rcsc::Vector2D tmmmPos = (
					(*t)->seenPosCount() <= (*t)->posCount() ?
							(*t)->seenPos() : (*t)->pos());

			rcsc::Vector2D tVel = (
					(*t)->seenVelCount() <= (*t)->velCount() ?
							(*t)->seenVel() : (*t)->vel());

			rcsc::Vector2D tPos = tmmmPos + tVel;

			if (tPos.x > wm.offsideLineX())
				continue;

			rcsc::Vector2D target[41];

			int tTmmToTarget[41];

			double ballToTarget[41];

			double v0forTarget[41];

			target[0] = tPos;
			target[1] = tPos + rcsc::Vector2D(2.5, 0);
			target[2] = tPos + rcsc::Vector2D(5, 0);
			target[3] = tPos + rcsc::Vector2D(7.5, 0);
			target[4] = tPos + rcsc::Vector2D(10, 0);
			target[5] = tPos + rcsc::Vector2D(2.5, -2.5);
			target[6] = tPos + rcsc::Vector2D(5, -2.5);
			target[7] = tPos + rcsc::Vector2D(2.5, -5);
			target[8] = tPos + rcsc::Vector2D(5, -5);
			target[9] = tPos + rcsc::Vector2D(7.5, -5);
			target[10] = tPos + rcsc::Vector2D(5, -7.5);
			target[11] = tPos + rcsc::Vector2D(0, -2.5);
			target[12] = tPos + rcsc::Vector2D(0, -5);
			target[13] = tPos + rcsc::Vector2D(0, -7.5);
			target[14] = tPos + rcsc::Vector2D(0, -10);
			target[15] = tPos + rcsc::Vector2D(0, 2.5);
			target[16] = tPos + rcsc::Vector2D(0, 5);
			target[17] = tPos + rcsc::Vector2D(0, 7.5);
			target[18] = tPos + rcsc::Vector2D(0, 10);
			target[19] = tPos + rcsc::Vector2D(2.5, 2.5);
			target[20] = tPos + rcsc::Vector2D(2.5, 5);
			target[21] = tPos + rcsc::Vector2D(5, 2.5);
			target[22] = tPos + rcsc::Vector2D(5, 5);
			target[23] = tPos + rcsc::Vector2D(7.5, 5);
			target[24] = tPos + rcsc::Vector2D(5, 7.5);
			target[25] = tPos + rcsc::Vector2D(-2.5, 0);
			target[26] = tPos + rcsc::Vector2D(-5, 0);
			target[27] = tPos + rcsc::Vector2D(-7.5, 0);
			target[28] = tPos + rcsc::Vector2D(-10, 0);
			target[29] = tPos + rcsc::Vector2D(-2.5, 2.5);
			target[30] = tPos + rcsc::Vector2D(-5, 2.5);
			target[31] = tPos + rcsc::Vector2D(-2.5, 5);
			target[32] = tPos + rcsc::Vector2D(-5, 5);
			target[33] = tPos + rcsc::Vector2D(-7.5, 5);
			target[34] = tPos + rcsc::Vector2D(-5, 7.5);
			target[35] = tPos + rcsc::Vector2D(-2.5, -2.5);
			target[36] = tPos + rcsc::Vector2D(-5, -2.5);
			target[37] = tPos + rcsc::Vector2D(-2.5, -5);
			target[38] = tPos + rcsc::Vector2D(-5, -5);
			target[39] = tPos + rcsc::Vector2D(-7.5, -5);
			target[40] = tPos + rcsc::Vector2D(-5, -7.5);

			//对目标排序
			for (int m = 0; m < 40; m++)
				for (int n = 0; n < 40 - m; n++) {
					double dist = target[n].dist(rcsc::Vector2D(52.0, 0.0));
					double distNext = target[n + 1].dist(
							rcsc::Vector2D(52.0, 0.0));

					if (dist > distNext) {
						rcsc::Vector2D tmp = target[n];
						target[n] = target[n + 1];
						target[n + 1] = tmp;
					}
				}

			for (int z = 0; z < 41; z++) {

				if (target[z].x < -36.0)
					target[z].x = -36.0;
				if (target[z].y > 32.0)
					target[z].y = 32.0;
				if (target[z].y < -32.0)
					target[z].y = -32.0;
				if (target[z].x > 50.0)
					target[z].x = 50.0;

				if (target[z].x < 27.0 && target[z].absY() < 20.0
						&& wm.countTeammatesIn(rcsc::Circle2D(target[z], 4.5),
								3, false) > 2)
					target[z].x += 20.0;

				double sum = 0;

				tTmmToTarget[z] = player_type->cyclesToReachDistance(
						target[z].dist(tPos)) + 1;

				ballToTarget[z] = target[z].dist(ball);

				if ((*t)->unum() >= 9 && target[z].x < tPos.x - 1.0)
					tTmmToTarget[z] += 1;
				if ((*t)->unum() >= 9 && target[z].x < tPos.x - 0.1
						&& tVel.x > 0)
					tTmmToTarget[z] += 1;

				if ((*t)->unum() < 9 && target[z].x < tmm[i].x - 2.0
						&& (*t)->vel().x > 0.0)
					tTmmToTarget[z] += 1;

				if ((*t)->unum() < 9 && target[z].x > tmm[i].x
						&& (*t)->vel().x < 0.0)
					tTmmToTarget[z] += 1;

				if (target[z].x < 20.0 && ball.x > 30.0
						&& wm.countTeammatesIn(
								rcsc::Circle2D(rcsc::Vector2D(52.5, 0.0), 25.0),
								5, false) > 3)
					target[z].x += 20.0;

				if (((ball.x) < 36)&& (ball.dist(tPos))< (9.0 - ((ball.x + 36.0) / 8.0) + 2.0))
					continue;

				for (int j = 0; j <= tTmmToTarget[z]; j++)
					sum += std::pow(0.94, j);

				v0forTarget[z] = ballToTarget[z] / sum;

				rcsc::Vector2D tmptmm[1];
				tmptmm[0] = tPos;

				int z1 = cycles(agent, target[z], v0forTarget[z], oMax, opp, 1);
				int z2 = cycles(agent, target[z], v0forTarget[z], 1, tmptmm, 0);

				int delCycle = z1 - z2;

				if (delCycle < 1)
					continue;

				rcsc::Vector2D oneStepVel = rcsc::KickTable::calc_max_velocity(
						(target[z] - wm.ball().pos()).th(),
						agent->world().self().kickRate(),
						agent->world().ball().vel());
				double v0 = oneStepVel.r();

				if (v0 < v0forTarget[z])
					continue;

				if (v0forTarget[z] < 1.0)
					continue;

				if (v0 >= v0forTarget[z]) {
					rcsc::Body_KickOneStep(target[z], v0forTarget[z]).execute(
							agent);

					if (agent->config().useCommunication()
							&& (*t)->unum() != -1) {
						rcsc::Vector2D target_angle;
						target_angle.assign(0.0, 0.0);
						agent->addSayMessage(
								new rcsc::PassMessage((*t)->unum(),
										target[z] + target_angle,
										agent->effector().queuedNextBallPos(),
										agent->effector().queuedNextBallVel()));
						agent->setNeckAction(
								new rcsc::Neck_TurnToLowConfTeammate());
					}

					return true;
				}

			} //counter=z end

		} //iterator end

	} //teammates end

	return false;
}
//直传部分
bool Bhv_ZhPass2::DirectPass(rcsc::PlayerAgent * agent,
		rcsc::Vector2D & target) {

	const rcsc::WorldModel & wm = agent->world();


	rcsc::Vector2D ball = wm.ball().pos();

	float passSpeed = 3.0;
	double delCycle[20];
	double z1[20];
	double z2[20];
	rcsc::Vector2D targets[20];

	for (int z = 0; z < 20; z++) {
		delCycle[z] = -10000;
		z1[z] = 0;
		z2[z] = 1000;
		targets[z] = rcsc::Vector2D(1000.0, 500.0);
	}

	for (int i = 0; i < tMax; i++) {
		if (tCount[i] > 2)
			continue;
		targets[i] = tmm[i];

		passSpeed = 3.0;
		if (ball.x < 36.0)
			passSpeed *= ball.dist(targets[i]) / 10.0;
		else
			passSpeed *= ball.dist(targets[i]) / 8.0;
		passSpeed = std::min(float(3.0), passSpeed);
		passSpeed = std::max(float(1.0), passSpeed);
		if (ball.dist(targets[i]) < 5.0)
			passSpeed -= 0.3;

		rcsc::Vector2D tmptmm[1];
		tmptmm[0] = targets[i];
		z1[i] = cycles(agent, targets[i], passSpeed, oMax, opp, 1);
		z2[i] = cycles(agent, targets[i], passSpeed, 1, tmptmm, 0);
		delCycle[i] = z1[i] - z2[i];

		if (delCycle[i] < 1)
			delCycle[i] = -10000;
	}

	float rate[20];

	for (int z = 0; z < 20; z++)
		rate[z] = 0.0;

	for (int i = 0; i < tMax; i++) {
		if (delCycle[i] == -10000)
			continue;

		int minCycle = 1;

		if (ball.x > 40.0 && targets[i].absY() < ball.absY()
				&& targets[i].x > 40.0
				&& targets[i].dist(rcsc::Vector2D(52.5, 0.0)) < 15.0
				&& targets[i].dist(ball) < 11.75 && targets[i].dist(ball) > 5.0)
			minCycle = 0;

		if (ball.x > wm.offsideLineX() - 20.0 && ball.x < 36.0
				&& wm.opponentsFromSelf().front()->distFromSelf() > 4.0)
			minCycle = 1;

		float idealPassDist = 20.0;

		if (delCycle[i] < minCycle)
			continue;
		if (targets[i].x < ball.x - 35)
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

// 防守权重
		if (ball.x < -25.0) {
			dCycleRate = 2;
			dist20Rate = 0.2;
			dGoalRate = 10.0;
			xRate = 12.0;
			sideRate = 10.0;
		}

		if (ball.x < -33.0) {
			dCycleRate = 10;
			dist20Rate = 0.4;
			idealPassDist = 30.0;
			dGoalRate = 4.0;
			xRate = 5.0;
			sideRate = 3.0;
		}

//   预期rate
		if (delCycle[i] > 8)
			delCycle[i] = 8;

		rate[i] += (delCycle[i] * dCycleRate);

		float dis = std::fabs(ball.dist(targets[i]) - idealPassDist);
		if (dis > 10)
			dis = 10.0;


		rate[i] -= (dis * dist20Rate);

//    离球门距离
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

		rate[i] += std::min(float(targets[i].x - ball.x), float(20.0)) * xRate;

//    边路通过情况
		if (targets[i].absY() > 15.0 && ball.x < 36.0)
			rate[i] += (targets[i].absY() - 15.0) * sideRate;

//   己方禁区
		if (targets[i].x < -36.0 && targets[i].absY() < 18.0)
			rate[i] -= 20;

		if (targets[i].x > 38.0 && targets[i].absY() > ball.absY()
				&& targets[i].absY() > 18.0)
			rate[i] -= 5;

		if (targets[i].x < -36.0 && targets[i].absY() < 20.0)
			rate[i] -= 200;

		if (ball.x < -25.0 && targets[i].dist(ball) < 10.0
				&& targets[i].x < ball.x)
			rate[i] -= 50;

	}

	float maxRate = -1000.0;
	int bi = -1;

	for (int i = 0; i < tMax; i++)
		if (rate[i] > maxRate) {
			maxRate = rate[i];
			bi = i;
		}

	float minRate = 0.15;

	if (ball.absY() < 20.0 && ball.x < 36.0)
		minRate = 0.1;

	if (wm.opponentsFromSelf().front()->pos().dist(wm.self().pos()) < 5.0)
		minRate = -5.2;

	if (wm.opponentsFromSelf().front()->pos().dist(wm.self().pos()) < 2.7)
		minRate = -5.7;

	if (wm.opponentsFromSelf().front()->pos().dist(wm.self().pos()) < 1.9)
		minRate = -7.0;

	minRate = -9999999;

	if (maxRate < minRate) {
		return false;
	}

	passSpeed = 3.0;

	if (ball.x < 36.0)
		passSpeed *= ball.dist(targets[bi]) / 10.0;
	else
		passSpeed *= ball.dist(targets[bi]) / 8.0;

	passSpeed = std::min(float(3.0), passSpeed);
	passSpeed = std::max(float(1.0), passSpeed);

	target = targets[bi];

	if (target.x == 1000.0 || target.y == 500.0)
		return false;

	if (receiver[bi] == 1 || receiver[bi] == -1)
		return false;

	if (delCycle[bi] < 1)
		return false;

	if (maxRate == 0)
		return false;

	int kickSteps = (
			agent->world().gameMode().type() != rcsc::GameMode::PlayOn
					&& agent->world().gameMode().type()
							!= rcsc::GameMode::GoalKick_ ? 1 : 3);

	rcsc::Vector2D oneStepVel = rcsc::KickTable::calc_max_velocity(
			(target - wm.ball().pos()).th(), agent->world().self().kickRate(),
			agent->world().ball().vel());
	double v0 = oneStepVel.r();

	rcsc::Vector2D ttmptmm[1];
	ttmptmm[0] = targets[bi];

	if (v0 >= passSpeed
			&& cycles(agent, targets[bi], passSpeed, oMax, opp, 1)
					- cycles(agent, targets[bi], passSpeed, 1, ttmptmm, 0)
					> 0) {
		rcsc::Body_KickOneStep(target, passSpeed).execute(agent);

		if (agent->config().useCommunication() && receiver[bi] != -1) {
			rcsc::Vector2D target_angle;
			target_angle.assign(0.0, 0.0);
			agent->addSayMessage(
					new rcsc::PassMessage(receiver[bi],
							targets[bi] + target_angle,
							agent->effector().queuedNextBallPos(),
							agent->effector().queuedNextBallVel()));
			agent->setNeckAction(new rcsc::Neck_TurnToLowConfTeammate());
		}
		return true;
	} else if (v0 >= (passSpeed * 0.9)
			&& cycles(agent, targets[bi], (passSpeed * 0.9), oMax, opp, 1)
					- cycles(agent, targets[bi], (passSpeed * 0.9), 1, ttmptmm,
							0) > 0) {
		rcsc::Body_KickOneStep(target, passSpeed * 0.9).execute(agent);

		if (agent->config().useCommunication() && receiver[bi] != -1) {
			rcsc::Vector2D target_angle;
			target_angle.assign(0.0, 0.0);
			agent->addSayMessage(
					new rcsc::PassMessage(receiver[bi],
							targets[bi] + target_angle,
							agent->effector().queuedNextBallPos(),
							agent->effector().queuedNextBallVel()));
			agent->setNeckAction(new rcsc::Neck_TurnToLowConfTeammate());
		}
		return true;
	} else if (v0 >= (passSpeed * 0.8)
			&& cycles(agent, targets[bi], (passSpeed * 0.8), oMax, opp, 1)
					- cycles(agent, targets[bi], (passSpeed * 0.8), 1, ttmptmm,
							0) > 0) {
		rcsc::Body_KickOneStep(target, passSpeed * 0.8).execute(agent);

		if (agent->config().useCommunication() && receiver[bi] != -1) {
			rcsc::Vector2D target_angle;
			target_angle.assign(0.0, 0.0);
			agent->addSayMessage(
					new rcsc::PassMessage(receiver[bi],
							targets[bi] + target_angle,
							agent->effector().queuedNextBallPos(),
							agent->effector().queuedNextBallVel()));
			agent->setNeckAction(new rcsc::Neck_TurnToLowConfTeammate());
		}
		return true;
	}

	//这0.96不是0.94吗
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

bool Bhv_ZhPass2::DKhBClear(rcsc::PlayerAgent * agent) {
	const rcsc::WorldModel & wm = agent->world();
	rcsc::Vector2D ball = wm.ball().pos();

	if (ball.x > -34.0)
		return false;

	//这次是19个
	rcsc::Vector2D targ[19];

	targ[0] = rcsc::Vector2D(-52.0, 16.0 * sign(ball.y));
	targ[1] = rcsc::Vector2D(-52.0, 20.0 * sign(ball.y));
	targ[2] = rcsc::Vector2D(-52.0, 28.0 * sign(ball.y));
	targ[3] = rcsc::Vector2D(-52.0, 34.0 * sign(ball.y));
	targ[4] = rcsc::Vector2D(-45.0, 34.0 * sign(ball.y));
	targ[5] = rcsc::Vector2D(-40.0, 34.0 * sign(ball.y));
	targ[6] = rcsc::Vector2D(-35.0, 34.0 * sign(ball.y));
	targ[7] = rcsc::Vector2D(-30.0, 34.0 * sign(ball.y));
	targ[8] = rcsc::Vector2D(-25.0, 34.0 * sign(ball.y));
	targ[9] = rcsc::Vector2D(-20.0, 34.0 * sign(ball.y));
	targ[10] = rcsc::Vector2D(-15.0, 34.0 * sign(ball.y));
	targ[11] = rcsc::Vector2D(-10.0, 34.0 * sign(ball.y));
	targ[12] = rcsc::Vector2D(-5.0, 34.0 * sign(ball.y));
	targ[13] = rcsc::Vector2D(0.0, 34.0 * sign(ball.y));
	targ[14] = rcsc::Vector2D(0.0, 28.0 * sign(ball.y));
	targ[15] = rcsc::Vector2D(0.0, 20.0 * sign(ball.y));
	targ[16] = rcsc::Vector2D(0.0, 13.0 * sign(ball.y));
	targ[17] = rcsc::Vector2D(0.0, 7.0 * sign(ball.y));
	targ[18] = rcsc::Vector2D(0.0, 0.0 * sign(ball.y));

	int delCycle[19];
	int z1[19];

	for (int kh = 0; kh < 19; kh++) {
		rcsc::Vector2D one_step_vel = rcsc::KickTable::calc_max_velocity(
				(targ[kh] - agent->world().ball().pos()).th(),
				agent->world().self().kickRate(), agent->world().ball().vel());
		double one_step_v0 = one_step_vel.r();

		z1[kh] = cycles(agent, targ[kh], one_step_v0, oMax, opp, 1);
		int z2 = cycles(agent, targ[kh], one_step_v0, tMax, tmm, 0);
		delCycle[kh] = z1[kh] - z2;
	}

	int bestCycle = -2000;
	int bestID = -1;

	for (int kh = 0; kh < 19; kh++) {
		if (delCycle[kh] > bestCycle) {
			bestID = kh;
			bestCycle = delCycle[kh];
		} else if (delCycle[kh] == bestCycle)
			if (z1[kh] == 1000) {
				bestID = kh;
				bestCycle = delCycle[kh];
			}
	}

	if (bestID != -1) {
		rcsc::Vector2D one_step_vel = rcsc::KickTable::calc_max_velocity(
				(targ[bestID] - agent->world().ball().pos()).th(),
				agent->world().self().kickRate(), agent->world().ball().vel());
		double one_step_v0 = one_step_vel.r();

		if (rcsc::Body_KickOneStep(targ[bestID], one_step_v0).execute(agent)) {
			agent->setNeckAction(new rcsc::Neck_TurnToLowConfTeammate());
//			std::cout << "你妹cycle: " << wm.time().cycle()<< "你妹Target: " << targ[bestID] << "\n";
			return true;
		} else
			return false;

	}
	return false;
}
