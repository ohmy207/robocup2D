// -*-c++-*-

/*
 *Copyright (C) TJNU
 */

#include "best_shoot_point.h"
#include "WorldModel.h"


WorldModelHighLevel WM_HIGHLEVEL;

Vector2D Best_ShootPoint::Point( PlayerAgent * agent , int & point )
{
	Vector2D my_pos = agent->world().self().pos();


	Vector2D * pos_goal = new Vector2D [5];

	pos_goal[0] = Vector2D( 52.5 , 5.8 );
	pos_goal[1] = Vector2D( 52.5 , 2.9 );
	pos_goal[2] = Vector2D( 52.5 , 0.0 );
	pos_goal[3] = Vector2D( 52.5 ,-2.9 );
	pos_goal[4] = Vector2D( 52.5 ,-5.8 );


	Triangle2D triangle_goal[4] = {
					Triangle2D( my_pos , pos_goal[0] , pos_goal[1] ),
					Triangle2D( my_pos , pos_goal[1] , pos_goal[2] ),
					Triangle2D( my_pos , pos_goal[2] , pos_goal[3] ),
					Triangle2D( my_pos , pos_goal[3] , pos_goal[4] )
				      };

	int * points = new int [4];	
	points[0] = 0 , points[1] = 0 , points[2] = 0 , points[3] = 0;

	Vector2D * pos_middle = new Vector2D [4];
	pos_middle[0] = Vector2D( 52.5 , 5.0 );
	pos_middle[1] = Vector2D( 52.5 , 2.0 );
	pos_middle[2] = Vector2D( 52.5 ,-2.0 );
	pos_middle[3] = Vector2D( 52.5 ,-5.0 );

	AngleDeg * ang_pos = new AngleDeg [4];

	ang_pos[0] = 0 , ang_pos[1] = 0 , ang_pos[2] = 0 , ang_pos[3] = 0;

//because of some problems, fast initializing(by FOR ) change to direct initializing.
	ang_pos[0] = ( pos_goal[0] - my_pos ).th() - ( pos_goal[1] - my_pos ).th();
	ang_pos[1] = ( pos_goal[1] - my_pos ).th() - ( pos_goal[2] - my_pos ).th();
	ang_pos[2] = ( pos_goal[2] - my_pos ).th() - ( pos_goal[3] - my_pos ).th();
	ang_pos[3] = ( pos_goal[3] - my_pos ).th() - ( pos_goal[4] - my_pos ).th();

////////////////////////////////////////////

	if( ( agent->world().getOpponentGoalie() && my_pos.dist( pos_goal[3] ) > 22.0 ) )
	{
		point = 0;

		delete [] pos_goal     ;
		delete [] points       ;
		delete [] pos_middle   ;
		delete [] ang_pos      ;

		return Vector2D( 52.5 , 7.0 );
	}


	if( agent->world().getOpponentGoalie() &&
	    Triangle2D( my_pos , pos_goal[0] ,  pos_goal[2] ).contains( agent->world().getOpponentGoalie()->pos() ) )
	{
		points[0] -= 100;
		points[1] -= 100;

		if( ang_pos[2].degree() < 10 )
		{
			if( my_pos.y < 0 )
				points[2] += 50;
		}
		if( ang_pos[3].degree() < 10 )
		{
			if( my_pos.y < 0 )
				points[3] += 50;
		}
	}
	else
	{
		points[2] -= 100;
		points[3] -= 100;

		if( ang_pos[0].degree() < 10 )
		{
			if( my_pos.y > 0 )
				points[0] += 50;
		}
		if( ang_pos[1].degree() < 10 )
		{
			if( my_pos.y > 0 )
				points[1] += 50;
		}
	}

	for( int i = 0 ; i <= 3 ; i ++ )
	{
		if( WM_HIGHLEVEL.getNrInSetInTriangle( agent , triangle_goal[i] ) == 0 )
			points[i] += 40;
		else
			points[i] -= 40;

		if( i != 0 )
		{
			if ( WM_HIGHLEVEL.getNrInSetInTriangle( agent , triangle_goal[i-1] ) == 0 )
				points[i] += 20;
			else
				points[i] -= 10;
		}

		if( i != 3 )
		{
			if ( WM_HIGHLEVEL.getNrInSetInTriangle( agent , triangle_goal[i+1] ) == 0 )
				points[i] += 20;
			else
				points[i] -= 10;
		}

		

		if( !agent->world().getOpponentGoalie() )
			points[i] += 100;
		else
		if( my_pos.dist( pos_middle[i] ) < agent->world().getOpponentGoalie()->pos().dist( pos_middle[i] ) )
				points[i] += 50;


	}

	if( agent->world().getOpponentGoalie() )
	{
		for( int i = 0 ; i <= 3 ; i ++ )
		{
			if( triangle_goal[i].contains( agent->world().getOpponentGoalie()->pos() ) )
			{
				points[i] -= 100;
			}
		}
	}

///////////////////////////////
	int max_point = 0;

	for( int i = 0 ; i <= 3 ; i ++ )
	{
		if( points[i] > points[max_point] )
			max_point = i ;
	}

	if( points[max_point] >= 100 )
	{

		point = points[max_point];
		delete [] pos_goal     ;
		delete [] points       ;
		delete [] pos_middle   ;
		delete [] ang_pos      ;
		return pos_middle[max_point];

	}
	else
	{
		point = 0;
		delete [] pos_goal     ;
		delete [] points       ;
		delete [] pos_middle   ;
		delete [] ang_pos      ;
		return Vector2D( 52.5 , 0.0  );
	}
}
