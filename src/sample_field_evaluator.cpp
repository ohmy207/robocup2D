// -*-c++-*-

/*
 *Copyright:

 Copyright (C) Hiroki SHIMORA

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

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include "sample_field_evaluator.h"

#include "field_analyzer.h"
#include "simple_pass_checker.h"

#include <rcsc/player/player_evaluator.h>
#include <rcsc/common/server_param.h>
#include <rcsc/common/logger.h>
#include <rcsc/math_util.h>

#include <iostream>
#include <algorithm>
#include <cmath>
#include <cfloat>

// #define DEBUG_PRINT

using namespace rcsc;

static const int VALID_PLAYER_THRESHOLD = 8;

/*-------------------------------------------------------------------*/
/*!

 */
static double evaluate_state(const PredictState & state);

/*-------------------------------------------------------------------*/
/*!

 */
SampleFieldEvaluator::SampleFieldEvaluator() {

}

/*-------------------------------------------------------------------*/
/*!

 */
SampleFieldEvaluator::~SampleFieldEvaluator() {

}

/*-------------------------------------------------------------------*/
/*!

 */

/*
 double
 SampleFieldEvaluator::operator()( const PredictState & state,
 const std::vector< ActionStatePair > & path ) const
 {
 const double final_state_evaluation = evaluate_state( state );

 //
 // add by gou
 //

 //     实现10,9无法突破时回传;2345678选择更有利进攻方向传球(效果不明显);底线回传;限制11运球区域;10,9的躲避运球;提高传球优先级
 //	试图通过修改权值实现传边时贯穿突破对手球员,失败


 double point = 0.0;
 const ServerParam & SP = ServerParam::i();
 Vector2D opp_goal(SP.pitchHalfLength(),0);

 //	if(path.empty())
 //		return  -1.0e+9;

 for( std::vector<ActionStatePair>::const_iterator it = path.begin();
 it!=path.end();
 ++it){

 if(((*it).action().playerUnum() > 11 || (*it).action().playerUnum() < 1) || ((*it).action().targetPlayerUnum() > 11 || (*it).action().targetPlayerUnum() < 1))
 return  -1.0e+9;

 //声明
 const AbstractPlayerObject *from = state.ourPlayer((*it).action().playerUnum());
 const AbstractPlayerObject *to = state.ourPlayer((*it).action().targetPlayerUnum());
 const AbstractPlayerObject *pn = state.ourPlayer(9);
 const AbstractPlayerObject *pt = state.ourPlayer(10);
 const AbstractPlayerObject *pe = state.ourPlayer(11);
 const AbstractPlayerObject *ps = state.ourPlayer(7);
 const AbstractPlayerObject *pb = state.ourPlayer(8);
 Vector2D topoint = (*it).action().targetPoint();
 const SimplePassChecker M_Pass_check;
 M_Pass_check(state,(*from),(*to),(*it).action().targetPoint(),state.ball().vel().r());


 int count_from = 0;
 int count_nine = 0;
 int count_ten = 0;
 int count_sev = 0;
 int count_eig = 0;
 double min_dis_from = 1000.0;
 double min_dis_toball = 1000.0;
 double min_dis_nine = 1000.0;
 double min_dis_ten = 1000.0;
 //声明end

 //		if(state.theirPlayers().empty())
 //			return -1.0e+9;

 //遍历对手球员，获取关系
 for(AbstractPlayerCont::const_iterator q = state.theirPlayers().begin();
 q!=state.theirPlayers().end();
 ++q){

 if((*q)->isGhost())
 continue;

 double gou_from_x = (((*q)->pos().y)*((*from).pos().y) - ((*from).pos().y)*((*from).pos().y))/(52.5 - ((*from).pos().x)) + ((*from).pos().x);
 double gou_nine_x = (((*q)->pos().y)*((*pn).pos().y) - ((*pn).pos().y)*((*pn).pos().y))/(52.5 - ((*pn).pos().x)) + ((*pn).pos().x);
 double gou_ten_x = (((*q)->pos().y)*((*pt).pos().y) - ((*pt).pos().y)*((*pt).pos().y))/(52.5 - ((*pt).pos().x)) + ((*pt).pos().x);
 //传球队员
 if((*q)->pos().dist((*from).pos()) < 7.0 && (*q)->pos().x >= gou_from_x ){
 count_from ++;
 if((*from).pos().dist((*q)->pos()) < min_dis_from)
 min_dis_from = (*from).pos().dist((*q)->pos());
 }

 if((state.ball().pos() + state.ball().vel()).dist((*q)->pos()) < min_dis_toball)
 min_dis_toball = (state.ball().pos() + state.ball().vel()).dist((*q)->pos());
 //9号
 if((*q)->pos().dist((*pn).pos()) < 18.0 && (*q)->pos().x >= gou_nine_x ){
 count_nine ++;
 if((*pn).pos().dist((*q)->pos()) < min_dis_nine)
 min_dis_nine = (*pn).pos().dist((*q)->pos());
 }
 //10号
 if((*q)->pos().dist((*pt).pos()) < 18.0 && (*q)->pos().x >= gou_ten_x ){
 count_ten ++;
 if((*pt).pos().dist((*q)->pos()) < min_dis_ten)
 min_dis_ten = (*pt).pos().dist((*q)->pos());
 }

 //7号
 if((*q)->pos().dist((*ps).pos()) < 20.0){
 count_sev ++;
 //				if((*ps).pos().dist((*q)->pos()) < min_dis_nine)
 //					min_dis_nine = (*ps).pos().dist((*q)->pos());
 }
 //8号
 if((*q)->pos().dist((*pb).pos()) < 20.0){
 count_eig ++;
 //				if((*pb).pos().dist((*q)->pos()) < min_dis_ten)
 //					min_dis_ten = (*pb).pos().dist((*q)->pos());
 }

 }
 //遍历对手球员end


 bool toseven = true;
 bool togoal =false;
 if(count_eig < count_sev)
 toseven = false;
 if(count_from <=2 && min_dis_toball >= 5.0)
 togoal =true;

 switch((*it).action().category()){
 case CooperativeAction::Dribble:{

 switch((*from).unum()){
 case 11:{
 if(abs((*from).pos().y) > 22.0)
 if(topoint.x > (*from).pos().x)
 point += -1000.0;
 if((*from).pos().x >= state.ourOffensePlayerLineX()&&topoint.x - 3.0 > (*from).pos().x)
 point += 10.0;
 break;
 }
 case 10:
 case 9:{
 if( abs((*from).pos().y) > 20.0 && togoal)
 point += 50.0;
 break;
 }
 case 8:
 case 7:
 case 6:
 case 5:
 case 4:
 case 3:
 case 2:
 default:break;
 }
 break;
 }
 case CooperativeAction::Pass:{

 if((*to).isGhost())
 return -1.0e+6;
 if((*to).pos().x > state.offsideLineX())
 return -1.0e+6;

 if(FieldAnalyzer::get_pass_count(state,M_Pass_check,(*it).action().firstBallSpeed())<2){
 point += topoint.dist((*from).pos());
 if((*from).pos().x > 36.0)
 point += 20.0;
 if(state.ourOffensePlayerLineX() < 36.0 && state.ourOffensePlayerLineX() > 0.0)
 switch((*from).unum()){
 case 11:{
 if(((*to).unum() == 9 || (*to).unum() == 10) && topoint.x >= state.offsideLineX())
 point = point + topoint.dist((*from).pos())*100.0;
 break;
 }
 case 10:
 case 9:{

 if((*from).pos().x > (*to).pos().x)
 point = point + (*from).pos().x -(*to).pos().x + std::max( 0.0,
 40.0 - ServerParam::i().theirTeamGoalPos().dist((*from).pos())) - std::max( 0.0,
 40.0 - ServerParam::i().theirTeamGoalPos().dist((*to).pos())) ;

 break;

 }
 case 8:
 case 7:{
 if((*to).unum() == 10 || (*to).unum() == 9)
 point += 100.0;
 else if(toseven){
 point = point + (*from).pos().x -(*to).pos().x + std::max( 0.0,
 40.0 - ServerParam::i().theirTeamGoalPos().dist((*from).pos())) - std::max( 0.0,
 40.0 - ServerParam::i().theirTeamGoalPos().dist((*to).pos())) + 50.0 - abs((*from).pos().y - (*ps).pos().y)*2;
 }
 else {
 point = point + (*from).pos().x -(*to).pos().x + std::max( 0.0,
 40.0 - ServerParam::i().theirTeamGoalPos().dist((*from).pos())) - std::max( 0.0,
 40.0 - ServerParam::i().theirTeamGoalPos().dist((*to).pos())) + 50.0 - abs((*from).pos().y - (*pb).pos().y)*2;
 }
 }
 case 6:{

 if(((*to).unum() == 9 || (*to).unum() == 10) && topoint.x >= state.offsideLineX())
 point = point + topoint.dist((*from).pos())*100.0;
 if((*to).unum() == 10 || (*to).unum() == 9)
 point += 100.0;
 else if(toseven){
 point = point + (*from).pos().x -(*to).pos().x + std::max( 0.0,
 40.0 - ServerParam::i().theirTeamGoalPos().dist((*from).pos())) - std::max( 0.0,
 40.0 - ServerParam::i().theirTeamGoalPos().dist((*to).pos())) + 50.0 - state.ball().pos().dist((*ps).pos())*2;
 }
 else {
 point = point + (*from).pos().x -(*to).pos().x + std::max( 0.0,
 40.0 - ServerParam::i().theirTeamGoalPos().dist((*from).pos())) - std::max( 0.0,
 40.0 - ServerParam::i().theirTeamGoalPos().dist((*to).pos())) + 50.0 - state.ball().pos().dist((*pb).pos())*2;
 }
 break;
 }
 case 5:
 case 4:
 case 3:
 case 2:{
 if((*to).unum() == 10 || (*to).unum() == 9)
 point += 100.0;
 else if(toseven){
 if((*from).pos().x > (*to).pos().x)
 point = point + (*from).pos().x -(*to).pos().x + std::max( 0.0,
 40.0 - ServerParam::i().theirTeamGoalPos().dist((*from).pos())) - std::max( 0.0,
 40.0 - ServerParam::i().theirTeamGoalPos().dist((*to).pos())) + 50.0 - state.ball().pos().dist((*ps).pos())*2;
 }
 else {
 if((*from).pos().x > (*to).pos().x)
 point = point + (*from).pos().x -(*to).pos().x + std::max( 0.0,
 40.0 - ServerParam::i().theirTeamGoalPos().dist((*from).pos())) - std::max( 0.0,
 40.0 - ServerParam::i().theirTeamGoalPos().dist((*to).pos())) + 50.0 - state.ball().pos().dist((*pb).pos())*2;
 }
 break;
 }
 default:break;
 }
 }
 break;
 }
 case CooperativeAction::Shoot:
 case CooperativeAction::Hold:
 case CooperativeAction::Move:
 case CooperativeAction::Clear:
 case CooperativeAction::NoAction:
 default:break;
 }

 }

 if(state.ball().pos().x > 27.0 && state.ball().pos().x < 43.5 && abs(state.ball().pos().y) < 24.0)
 point += 30.0;

 //
 //end
 //

 double result = final_state_evaluation + point;

 return result;
 }
 */

double SampleFieldEvaluator::operator()(const PredictState & state,
		const std::vector<ActionStatePair> & path) const {
	const double final_state_evaluation = evaluate_state(state);

	//
	// add by gou
	//

//     实现10,9无法突破时回传;2345678选择更有利进攻方向传球(效果不明显);底线回传;限制11运球区域;10,9的躲避运球;提高传球优先级
//	试图通过修改权值实现传边时贯穿突破对手球员,失败

	double point = 0.0;

	int count_from = 0;
	double min_dis_toball = 1000.0;
	for (std::vector<ActionStatePair>::const_iterator it = path.begin();
			it != path.end(); ++it) {
		//声明
		const AbstractPlayerObject *from = state.ourPlayer(
				(*it).action().playerUnum());
		if (from == NULL) {
			return -1.0e+9;
		}

		Vector2D topoint = (*it).action().targetPoint();

		count_from = 0;
		min_dis_toball = 1000.0;
		//声明end

		//遍历对手球员，获取关系
		for (AbstractPlayerCont::const_iterator q =
				state.theirPlayers().begin(); q != state.theirPlayers().end();
				++q) {

			if ((*q)->isGhost())
				continue;

			double gou_from_x = (((*q)->pos().y) * ((*from).pos().y)
					- ((*from).pos().y) * ((*from).pos().y))
					/ (52.5 - ((*from).pos().x)) + ((*from).pos().x);
			//传球队员
			if ((*q)->pos().dist((*from).pos()) < 5.0
					&& (*q)->pos().x >= gou_from_x) {
				count_from++;

			}

			if ((state.ball().pos() + state.ball().vel()).dist(
					(*q)->pos() + (*q)->vel()) < min_dis_toball)
				min_dis_toball = (state.ball().pos() + state.ball().vel()).dist(
						(*q)->pos() + (*q)->vel());

		}
		//遍历对手球员end

		switch ((*it).action().category()) {
		case CooperativeAction::Dribble: {
			bool togoal = false;
			if (min_dis_toball >= 3.0)
				togoal = true;

			switch ((*from).unum()) {
			case 11: {
				if ((*from).pos().x > 27.0 && (*from).pos().x <46.0&& (*from).pos().absY() < 20.0
						&& min_dis_toball >= 3.0)
					point += 15.0;
				break;
			}
			case 10:
			case 9:
			case 8:
			case 7:
			{
				if (togoal && ((*from).pos().x <46.0 || (*from).pos().absY() >11.0 ))
					point += 15.0;
				break;
			}
			case 6:
			case 5:
			case 4:
			case 3:
			case 2:
			default:
				break;
			}
			break;
		}
		case CooperativeAction::Pass: {

			const AbstractPlayerObject *to = state.ourPlayer(
					(*it).action().targetPlayerUnum());
			if (to == NULL) {
				return -1.0e+9;
			}
			const SimplePassChecker M_Pass_check;
			M_Pass_check(state, (*from), (*to), (*it).action().targetPoint(),
					state.ball().vel().r());

			if ((*to).isGhost())
				return -1.0e+6;

//			if(((*to).pos() + (*to).vel()).x > state.offsideLineX())
//				return -1.0e+6;

			if (FieldAnalyzer::get_pass_count(state, M_Pass_check,
					(*it).action().firstBallSpeed()) < 2) {
				double distTemp=topoint.dist((*from).pos());
				if(distTemp<5.0)
				{
					point+=distTemp*0.4;
				}
				else if(distTemp<45.0)
				{
					point+=distTemp;
				}
				else if(distTemp<60.0)
				{
					point+=distTemp*0.7;
				}

				switch ((*from).unum()) {
				case 11:
				case 10:
				case 9: {
					if ((*from).pos().x > (*to).pos().x)
						point =point + (*from).pos().x - (*to).pos().x+ std::max(0.0,
						40.0- ServerParam::i().theirTeamGoalPos().dist((*from).pos()))
						- std::max(0.0,40.0- ServerParam::i().theirTeamGoalPos().dist(
						(*to).pos()))+ (*to).pos().x / 10;
					break;

				}
				default:
					break;
				}
			}
			break;
		}
		case CooperativeAction::Shoot:
		case CooperativeAction::Hold:
		case CooperativeAction::Move:
		case CooperativeAction::Clear:
		case CooperativeAction::NoAction:
		default:
			break;
		}
	}
	//
	//end
	//
	if(state.ball().pos().x > 27.0 && state.ball().pos().absY() < 20.0)
		point += 20.0;
	double result = final_state_evaluation + point;

	return result;
}

/*-------------------------------------------------------------------*/
/*!

 */
static
double evaluate_state(const PredictState & state) {
	const ServerParam & SP = ServerParam::i();

	const AbstractPlayerObject * holder = state.ballHolder();

#ifdef DEBUG_PRINT
	dlog.addText( Logger::ACTION_CHAIN,
			"========= (evaluate_state) ==========" );
#endif

	//
	// if holder is invalid, return bad evaluation
	//
	if (!holder) {
#ifdef DEBUG_PRINT
		dlog.addText( Logger::ACTION_CHAIN,
				"(eval) XXX null holder" );
#endif
		return -DBL_MAX / 2.0;
	}

	const int holder_unum = holder->unum();
	//
	// ball is in opponent goal
	//
	if (state.ball().pos().x > +(SP.pitchHalfLength() - 0.1)
			&& state.ball().pos().absY() < SP.goalHalfWidth() + 1.0) {
#ifdef DEBUG_PRINT
		dlog.addText( Logger::ACTION_CHAIN,
				"(eval) *** in opponent goal" );
#endif
		return +1.0e+7;
	}
	//
	// ball is in our goal
	//
	if (state.ball().pos().x < -(SP.pitchHalfLength() - 0.1)
			&& state.ball().pos().absY() < SP.goalHalfWidth()) {
#ifdef DEBUG_PRINT
		dlog.addText( Logger::ACTION_CHAIN,
				"(eval) XXX in our goal" );
#endif

		return -1.0e+7;
	}

	//
	// out of pitch
	//
	if (state.ball().pos().absX() > SP.pitchHalfLength()
			|| state.ball().pos().absY() > SP.pitchHalfWidth()) {
#ifdef DEBUG_PRINT
		dlog.addText( Logger::ACTION_CHAIN,
				"(eval) XXX out of pitch" );
#endif

		return -DBL_MAX / 2.0;
	}

	//
	// set basic evaluation
	//
	double point = state.ball().pos().x;

	double _zpointExtra=0.0;
	double _zdirball2goal=ServerParam::i().theirTeamGoalPos().dist( state.ball().pos() );
	_zpointExtra=std::max( 0.0,
			40.0 - _zdirball2goal);

	//    //边路
	//    if(state.ball().pos().x>10.0 && state.ball().pos().x<30.0
	//    		&& ((state.ball().pos().y>24.0)||(state.ball().pos().y<-24.0)))
	//    {
	//    	_zpointExtra+=3;
	//    }
	//
	//    //中路
	//    if(state.ball().pos().x>17.0 && state.ball().pos().x<30.0
	//    		&& state.ball().pos().absY()<15.0)
	//    {
	//    	_zpointExtra+=15.0;
	//    }
	//两个角上
	if(state.ball().pos().y<state.ball().pos().x-66.0
			|| state.ball().pos().y>-state.ball().pos().x+66.0)
	{
		_zpointExtra=std::max( 0.0,
				30.0 - _zdirball2goal );
	}
	point+=_zpointExtra;

#ifdef DEBUG_PRINT
	dlog.addText( Logger::ACTION_CHAIN,
			"(eval) ball pos (%f, %f)",
			state.ball().pos().x, state.ball().pos().y );

	dlog.addText( Logger::ACTION_CHAIN,
			"(eval) initial value (%f)", point );
#endif

	//
	// add bonus for goal, free situation near offside line
	//
	if (FieldAnalyzer::can_shoot_from(holder->unum() == state.self().unum(),
			holder->pos(),
			state.getPlayerCont(
					new OpponentOrUnknownPlayerPredicate(state.ourSide())),
			VALID_PLAYER_THRESHOLD)) {
		point += 1.0e+6;
#ifdef DEBUG_PRINT
		dlog.addText( Logger::ACTION_CHAIN,
				"(eval) bonus for goal %f (%f)", 1.0e+6, point );
#endif

		if (holder_unum == state.self().unum()) {
			point += 5.0e+5;
#ifdef DEBUG_PRINT
			dlog.addText( Logger::ACTION_CHAIN,
					"(eval) bonus for goal self %f (%f)", 5.0e+5, point );
#endif
		}
	}

	return point;
}
