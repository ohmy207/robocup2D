#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <rcsc/geom/vector_2d.h>
#include <rcsc/geom/line_2d.h>

#include <boost/shared_ptr.hpp>

#include "bhv_goalie_one_to_one.h"
#include <rcsc/action/basic_actions.h>
#include <rcsc/action/body_go_to_point.h>
#include <rcsc/action/bhv_go_to_point_look_ball.h>
#include <rcsc/action/body_turn_to_point.h>
#include <rcsc/action/neck_scan_field.h>

#include <rcsc/player/player_agent.h>
#include <rcsc/player/intercept_table.h>
#include <rcsc/player/debug_client.h>

#include <rcsc/common/logger.h>
#include <rcsc/common/server_param.h>
#include <rcsc/soccer_math.h>

#include <rcsc/action/view_wide.h>
#include <rcsc/action/neck_turn_to_ball_or_scan.h>

#include <rcsc/action/body_intercept2009.h>

//zsq 此函数还可用在禁区防守中
bool
Bhv_GoalieOneToOne::execute( rcsc::PlayerAgent * agent )
{
	rcsc::Vector2D pos_target = getTargetPoint( agent );
 
        rcsc::Bhv_GoToPointLookBall( pos_target,
				       0.5,
				      rcsc::ServerParam::i().maxDashPower()).execute( agent );
         agent->setNeckAction( new rcsc::Neck_ScanField() );
         return true;
	
}

rcsc::Vector2D
Bhv_GoalieOneToOne::getTargetPoint( rcsc::PlayerAgent * agent )
{

	const rcsc::WorldModel & wm = agent->world();

	rcsc::Vector2D  ball_pos  = wm.ball().pos();
	rcsc::Vector2D  ball_vel  = wm.ball().vel();
	rcsc::Line2D    ball_line = rcsc::Line2D( ball_pos , ball_pos + ball_vel );//球和球的下一个位置组成一条直线
        
        double move_x = ball_pos.x - 3.0;  //zsq 2011是5
        double move_y = ball_line.getY( move_x );

        return rcsc::Vector2D(move_x,move_y);//移动这个点，出击缩小防守
         
}

