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

#include "role_side_back.h"

#include "bhv_chain_action.h"
#include "bhv_basic_offensive_kick.h"
#include "bhv_basic_move.h"
#include <rcsc/player/intercept_table.h>
#include <rcsc/action/body_intercept2009.h>

#include <rcsc/action/body_go_to_point.h>
#include <rcsc/action/body_turn_to_ball.h>

#include <rcsc/action/neck_turn_to_ball_or_scan.h>
#include <rcsc/action/neck_turn_to_ball.h>
#include <rcsc/action/neck_turn_to_player_or_scan.h>
#include <rcsc/action/body_pass.h>
#include <rcsc/action/body_advance_ball.h>
#include <rcsc/action/neck_scan_field.h>

#include <rcsc/player/player_agent.h>
#include <rcsc/player/debug_client.h>
#include<strategy.h>

#include <rcsc/geom/ray_2d.h>

#include <rcsc/common/logger.h>

using namespace rcsc;

const std::string RoleSideBack::NAME( "SideBack" );

/*-------------------------------------------------------------------*/
/*!

 */
namespace {
rcss::RegHolder role = SoccerRole::creators().autoReg( &RoleSideBack::create,
                                                       RoleSideBack::NAME );
}

/*-------------------------------------------------------------------*/
/*!

*/
bool
RoleSideBack::execute( PlayerAgent * agent )
{
    bool kickable = agent->world().self().isKickable();
    if ( agent->world().existKickableTeammate()
         && agent->world().teammatesFromBall().front()->distFromBall()
         < agent->world().ball().distFromSelf() )
    {
        kickable = false;
    }

    if ( kickable )
    {
        doKick( agent );
    }
    else
    {
        doMove( agent );
    }

    return true;
}

/*-------------------------------------------------------------------*/
/*!

*/
void
RoleSideBack::doKick( PlayerAgent * agent )
{
     if ( Bhv_ChainAction().execute( agent ) )
     {
         dlog.addText( Logger::TEAM,
                       __FILE__": (execute) do chain action" );
         agent->debugClient().addMessage( "ChainAction" );
         return;
     }

    Bhv_BasicOffensiveKick().execute( agent );
}

/*-------------------------------------------------------------------*/
/*!

*/
void
RoleSideBack::doMove( PlayerAgent * agent )
{
	//zxy begin 12-08
	//以下代码段是为了让后卫和中场在我方控球时不要太过靠近
	const WorldModel & wm = agent->world();

	int self_min = wm.interceptTable()->selfReachCycle();
	int mate_min = wm.interceptTable()->teammateReachCycle();
	int opp_min = wm.interceptTable()->opponentReachCycle();
	int our_min = std::min( self_min, mate_min );

	//如果是进攻状态
	if ( our_min <= opp_min - 2 )
	{
		if( wm.existKickableTeammate() )
		{
			PlayerObject ball_holder = (*wm.getTeammateNearestToBall(3,false));

			//以带球队员为圆心画一个范围
			Circle2D ball_holder_circle( ball_holder.pos(),12.0 );

			const Vector2D self_pos = wm.self().pos();
			const Vector2D holder_pos = ball_holder.pos();
			const Vector2D ball_pos = wm.ball().pos();

			//目标位置。默认值可以任意
			Vector2D target_pos1( -20,0 );
			Vector2D target_pos2( 0,0 );	//这个点基本上不会用到
			//从球到自己引一条射线
			Ray2D self_ball_ray( ball_pos,self_pos );
			AngleDeg ball_th = wm.ball().vel().th();
			if( std::fabs((self_ball_ray.dir()-ball_th).degree()) < 60.0 )
			{

				if (ball_holder_circle.contains( self_pos ) )
				{
					int sec_count = ball_holder_circle.intersection( self_ball_ray,&target_pos1,&target_pos2 );

					//若有交点
					if( sec_count == 1 )
					{
						double dist_thr = wm.ball().distFromSelf() * 0.1;
							if ( dist_thr < 1.0 ) dist_thr = 1.0;
						const double dash_power = Strategy::get_normal_dash_power( wm );
						if ( ! Body_GoToPoint( target_pos1, dist_thr, dash_power
												   ).execute( agent ) )
							{
								Body_TurnToBall().execute( agent );
							}

							if ( wm.existKickableOpponent()
								 && wm.ball().distFromSelf() < 18.0 )
							{
								agent->setNeckAction( new Neck_TurnToBall() );
							}
							else
							{
								agent->setNeckAction( new Neck_TurnToBallOrScan() );
							}
					}
					else if( sec_count == 0 )
					{
						//默认跑为点因后卫球员现在的站位决定
						if( self_pos.y>0 )
							target_pos1.assign(-30.0,20.0);
						else
							target_pos1.assign(-30.0,-20.0);

						double dist_thr = wm.ball().distFromSelf() * 0.1;
							if ( dist_thr < 1.0 ) dist_thr = 1.0;
						const double dash_power = Strategy::get_normal_dash_power( wm );
						if ( ! Body_GoToPoint( target_pos1, dist_thr, dash_power
												   ).execute( agent ) )
							{
								Body_TurnToBall().execute( agent );
							}

						if ( wm.existKickableOpponent()
							 && wm.ball().distFromSelf() < 18.0 )
						{
							agent->setNeckAction( new Neck_TurnToBall() );
						}
						else
						{
							agent->setNeckAction( new Neck_TurnToBallOrScan() );
						}
					}
					return;
				}
			}
		}
	}
	//zxy end
	
    Bhv_BasicMove().execute( agent );
}
