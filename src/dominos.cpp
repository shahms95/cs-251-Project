#include "cs251_base.hpp"
#include "render.hpp"
#include <cmath>
#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include "GL/freeglut.h"
#endif

#include <cstring>
using namespace std;

#include "dominos.hpp"
#include <vector>
#include <iostream>
class Explosion {
public:
  b2Body* body;
  b2FixtureDef fd;
  b2World* world;
  vector<b2Body*> v;
  int numRays;
  float x,y;
    void explode(){
     for (int i = 0; i < numRays; i++) {

  
       b2BodyDef bd;
       bd.type = b2_dynamicBody;
       bd.fixedRotation = true; // rotation not necessary
       bd.bullet = true; // prevent tunneling at high speed
       //bd.linearDamping = 1; // drag due to moving through air
       bd.gravityScale = 0.0; // ignore gravity
       bd.position.Set(x,y+10); // start at blast center
       //bd.linearVelocity = 30 * rayDir;
       b2Body* bod = world->CreateBody( &bd );
       v.push_back(bod);
       b2CircleShape circleShape;
       circleShape.m_radius = 0.2; // very small
       b2FixtureDef fd;
       fd.shape = &circleShape;
      fd.density = 60 / (float)numRays; // very high - shared across all particles
      fd.friction = 0; // friction not necessary
      fd.restitution = 0.99f; // high restitution to reflect off obstacles
      fd.filter.groupIndex = -1; // particles should not collide with each other
       bod->CreateFixture( &fd );
   }
}
  Explosion(int num,float xc,float yc,b2World* m_world){
    b2BodyDef bd;
    numRays=num;
    x=xc;y=yc;
    world=m_world;
    bd.type = b2_dynamicBody;
      bd.gravityScale = 0; // ignore gravity
      bd.position.Set(x,y);
     body = m_world->CreateBody( &bd );
  
      b2CircleShape circleShape;
       circleShape.m_radius = .412; // very small
  
       b2FixtureDef fd;
       fd.shape = &circleShape;
       fd.density = 0 / (float)numRays; // very high - shared across all particles
       fd.filter.groupIndex = -1; 
       body->CreateFixture( &fd );
       body->SetUserData(this);
       explode();
  }
  void start(){
  	for(int i=0;i<v.size();i++){
  	float angle = (((i*360 / numRays) )%90-35) * 3.1428/180;
       b2Vec2 rayDir( sinf(angle), cosf(angle) );
  		v[i]->SetLinearVelocity(30*rayDir);
  	}
  	body->SetLinearVelocity(b2Vec2(0,30));
  }

};
  class MyContactListener : public b2ContactListener
  {
    void BeginContact(b2Contact* contact) {
      //check if fixture A was a ball
      void* bodyUserData = contact->GetFixtureA()->GetBody()->GetUserData();
      void* bodyUserData1 = contact->GetFixtureB()->GetBody()->GetUserData();
      if ( bodyUserData ){
        
        static_cast<Explosion*>( bodyUserData )->start();}
  
      //check if fixture B was a ball
      bodyUserData = contact->GetFixtureB()->GetBody()->GetUserData();
      if ( bodyUserData1 ){
        cout<<'z';
        static_cast<Explosion*>( bodyUserData1 )->start();
      }
  
    }
  
  };
 MyContactListener myContactListenerInstance;

namespace cs251
{
  /**  The is the constructor 
   * This is the documentation block for the constructor.
   */ 
  dominos_t::dominos_t()
  	{
	    //Ground
	    /*! \var b1 
	     * \brief pointer to the body ground 
	     */ 
	    b2Body* b1;  
	    {
	      
	      b2EdgeShape shape; 
	      shape.Set(b2Vec2(-90.0f, 0.0f), b2Vec2(90.0f, 0.0f));
	      b2BodyDef bd; 
	      b1 = m_world->CreateBody(&bd); 
	      b1->CreateFixture(&shape, 0.0f);
	    }
	    float posx_shelfs = -18.0f;
		float posy_shelfs = 9.0f;
	    {
	    	float posy_shelf1 = posy_shelfs;
	    	float posy_shelf2 = posy_shelfs + 3.8;
	    	float posy_shelf3 = posy_shelfs + 11.1;
	    	float posy_shelf4 = posy_shelfs + 16.0;

		    // horizontal shelfs
		    {
		    	float posx = posx_shelfs;
		    	float posy = posy_shelfs;
				b2PolygonShape shape;

				{
					shape.SetAsBox(11.0f, 0.25f);
					{ 					//the first bottom platforms
						b2BodyDef bd;
						bd.position.Set(posx, posy_shelf1);
						b2Body* ground = m_world->CreateBody(&bd);
						ground->CreateFixture(&shape, 0.0f);
					}
					{ 					//the second bottom platforms
						b2BodyDef bd;
						bd.position.Set(posx, posy_shelf2);
						b2Body* ground = m_world->CreateBody(&bd);
						ground->CreateFixture(&shape, 0.0f);
					}
				}
				{
					shape.SetAsBox(2.5f, 0.25f);
					b2BodyDef bd;
					bd.position.Set(posx + 8.3f, posy_shelf3);
					b2Body* ground = m_world->CreateBody(&bd);
					ground->CreateFixture(&shape, 0.0f);
		      	}
		      	{
					shape.SetAsBox(13.2f, 0.25f);
					b2BodyDef bd;
					bd.position.Set(posx + 2.0f, posy_shelf4);
					b2Body* ground = m_world->CreateBody(&bd);
					ground->CreateFixture(&shape, 0.0f);
		      	}
		    }
		    
		    float posx_hinge_left = posx_shelfs - 12.0f;
		    float posx_hinge_right = posx_shelfs + 12.0f;

		    //The hinged joints
		 	{					//hinge 1
				b2PolygonShape shape;
				shape.SetAsBox(0.2f, 2.6f);

				b2BodyDef bd;
				bd.position.Set(posx_hinge_left, posy_shelfs + 1.3f);
				bd.type = b2_dynamicBody;
				b2Body* body = m_world->CreateBody(&bd);
				b2FixtureDef *fd = new b2FixtureDef;
				fd->density = 8.0f;
				fd->shape = new b2PolygonShape;
				fd->shape = &shape;
				body->CreateFixture(fd);

				b2PolygonShape shape2;
				shape2.SetAsBox(2.0f, .2f);
				b2BodyDef bd2;
				bd2.position.Set(posx_hinge_left, posy_shelfs + 3.0f);
				b2Body* body2 = m_world->CreateBody(&bd2);

				b2RevoluteJointDef jointDef;
				jointDef.bodyA = body;
				jointDef.bodyB = body2;
				jointDef.localAnchorA.Set(0,0);
				jointDef.localAnchorB.Set(0,0);
				jointDef.collideConnected = false;
				m_world->CreateJoint(&jointDef);
		    }
		    {				//hinge 2
				b2PolygonShape shape;
				shape.SetAsBox(0.2f, 2.6f);

				b2BodyDef bd;
				bd.position.Set(posx_hinge_right + 0.5f,  posy_shelfs - 2.0f);
				bd.type = b2_dynamicBody;
				b2Body* body = m_world->CreateBody(&bd);
				b2FixtureDef *fd = new b2FixtureDef;
				fd->density = 8.0f;
				fd->shape = new b2PolygonShape;
				fd->shape = &shape;
				body->CreateFixture(fd);

				b2PolygonShape shape2;
				shape2.SetAsBox(2.0f, .2f);
				b2BodyDef bd2;
				bd2.position.Set(posx_hinge_right + 0.5f,  posy_shelfs);
				b2Body* body2 = m_world->CreateBody(&bd2);

				b2RevoluteJointDef jointDef;
				jointDef.bodyA = body;
				jointDef.bodyB = body2;
				jointDef.localAnchorA.Set(0,0);
				jointDef.localAnchorB.Set(0,0);
				jointDef.collideConnected = false;
				m_world->CreateJoint(&jointDef);
		    }
		    //right up
		    {			//hinge 3
				b2PolygonShape shape;
				shape.SetAsBox(0.2f, 2.6f+1.55f);	//plank

				b2BodyDef bd;
				bd.position.Set(posx_hinge_right + 10.0f,  posy_shelfs + 17.7f);
				bd.type = b2_dynamicBody;
				b2Body* body = m_world->CreateBody(&bd);
				b2FixtureDef *fd = new b2FixtureDef;
				fd->density = 8.0f;
				fd->shape = new b2PolygonShape;
				fd->shape = &shape;
				body->CreateFixture(fd);

				b2PolygonShape shape2;		//mid point
				shape2.SetAsBox(2.0f, 0.2f);
				b2BodyDef bd2;
				bd2.position.Set(posx_hinge_right + 0.5f,  posy_shelfs + 8.7f);
				b2Body* body2 = m_world->CreateBody(&bd2);

				b2RevoluteJointDef jointDef;
				jointDef.bodyA = body;
				jointDef.bodyB = body2;
				jointDef.localAnchorA.Set(0,0);
				jointDef.localAnchorB.Set(0,0);
				jointDef.collideConnected = false;
				m_world->CreateJoint(&jointDef);
		    }
		    {			//hinge 4
				b2PolygonShape shape;
				shape.SetAsBox(0.2f, 2.6f+2.55f);	//plank

				b2BodyDef bd;
				bd.position.Set(posx_hinge_left,  posy_shelfs + 13.7f);
				bd.type = b2_dynamicBody;
				b2Body* body = m_world->CreateBody(&bd);
				b2FixtureDef *fd = new b2FixtureDef;
				fd->density = 8.0f;
				fd->shape = new b2PolygonShape;
				fd->shape = &shape;
				body->CreateFixture(fd);

				b2PolygonShape shape2;		//mid point
				shape2.SetAsBox(2.0f, 0.2f);
				b2BodyDef bd2;
				bd2.position.Set(posx_hinge_left, posy_shelfs + 13.7f);
				b2Body* body2 = m_world->CreateBody(&bd2);

				b2RevoluteJointDef jointDef;
				jointDef.bodyA = body;
				jointDef.bodyB = body2;
				jointDef.localAnchorA.Set(0,0);
				jointDef.localAnchorB.Set(0,0);
				jointDef.collideConnected = false;
				m_world->CreateJoint(&jointDef);
		    }
		    float posx_ball_left = posx_shelfs + 10.6f;
		    float posx_ball_right = posx_shelfs - 11.0f;
		    //The balls
			{			//ball 1
				b2Body* sbody;
				b2CircleShape circle;
				circle.m_radius = 0.7;

				b2FixtureDef ballfd;
				ballfd.shape = &circle;
				ballfd.density = 3.0f;
				ballfd.friction = 0.01f;
				ballfd.restitution = 1.0f;
				b2BodyDef ballbd;
				ballbd.type = b2_dynamicBody;
				ballbd.position.Set(posx_ball_left, posy_shelf1 + 0.5f);
				sbody = m_world->CreateBody(&ballbd);
				sbody->CreateFixture(&ballfd);
		    }
		    {		//ball 2
				b2Body* sbody;
				b2CircleShape circle;
				circle.m_radius = 0.7;

				b2FixtureDef ballfd;
				ballfd.shape = &circle;
				ballfd.density = 2.0f;
				ballfd.friction = 0.01f;
				ballfd.restitution = 1.0f;
				b2BodyDef ballbd;
				ballbd.type = b2_dynamicBody;
				ballbd.position.Set(posx_ball_right, posy_shelf2 + 0.5f);
				sbody = m_world->CreateBody(&ballbd);
				sbody->CreateFixture(&ballfd);
		    }
		    {	//ball 3
				b2Body* sbody;
				b2CircleShape circle;
				circle.m_radius = 0.7;

				b2FixtureDef ballfd;
				ballfd.shape = &circle;
				ballfd.density = 5.0f;
				ballfd.friction = 0.0f;
				ballfd.restitution = -10.0f;
				b2BodyDef ballbd;
				ballbd.type = b2_dynamicBody;
				ballbd.position.Set(posx_ball_left, posy_shelf3 + 0.5f);
				sbody = m_world->CreateBody(&ballbd);
				sbody->CreateFixture(&ballfd);
		    }
		    {	//ball 4
				b2Body* sbody;
				b2CircleShape circle;
				circle.m_radius = 0.7;

				b2FixtureDef ballfd;
				ballfd.shape = &circle;
				ballfd.density = 2.0f;
				ballfd.friction = 0.0f;
				ballfd.restitution = -10.0f;
				b2BodyDef ballbd;
				ballbd.type = b2_dynamicBody;
				ballbd.position.Set(posx_ball_left, posy_shelf4 + 0.5f);
				sbody = m_world->CreateBody(&ballbd);
				sbody->CreateFixture(&ballfd);
		    }
		}


		float posx_dominos = -35.5f;
		float posy_dominos = 2.0f;
	    //Dominos
	    {
			float posx = posx_dominos;
			float posy = posy_dominos;
			//the ones with the increasing sizes
			for (int i = 0; i < 10; ++i)
			{
				b2PolygonShape shape;
				shape.SetAsBox(0.2f, 0.2f*(i+1));
				//
				b2FixtureDef fd;
				fd.shape = &shape;
				fd.density = 20.0f;
				fd.friction = 0.1f;
				b2BodyDef bd;
				bd.type = b2_staticBody;
				bd.position.Set(posx + 15.0f + 1.5f * i, posy + i*0.2f - 2.0f);
				b2Body* body = m_world->CreateBody(&bd);
				body->CreateFixture(&fd);
			}
			//the ones with fixed sizes on top of the steps
			for (int i = 0; i < 10; ++i)
			{
				b2PolygonShape shape;
				shape.SetAsBox(0.1f, 1.5f);

				b2FixtureDef fd;
				fd.shape = &shape;
				fd.density = 70.0f;
				fd.friction = 0.1f;

				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position.Set(posx + 15.0f + 1.5f * i, posy + 3.2f);
				b2Body* body = m_world->CreateBody(&bd);
				body->CreateFixture(&fd);
			}
			//the ones with the fixed sizes and on the ground
			for (int i = 0; i < 10; ++i)
			{
				b2PolygonShape shape;
				shape.SetAsBox(0.1f, 1.5f);				
				b2FixtureDef fd;
				fd.shape = &shape;
				fd.density = 70.0f;
				fd.friction = 0.1f;
					
				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position.Set(posx + 1.5f * i, posy);
				b2Body* body = m_world->CreateBody(&bd);
				body->CreateFixture(&fd);
			}
			//the ones on the top platform
			for (int i = 0; i < 15; ++i)
			{
				b2PolygonShape shape;
				shape.SetAsBox(0.1f, 1.7f);

				b2FixtureDef fd;
				fd.shape = &shape;
				fd.density = 10.0f;
				fd.friction = 0.1f;

				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position.Set(posx_shelfs - 11.0f + 1.3f * i, posy_shelfs + 19.0f);
				b2Body* body = m_world->CreateBody(&bd);
				body->CreateFixture(&fd);
			}
	    } 
	    //The newtons pendulum that knocks the dominos off
	    {
			float posx = posx_dominos - 9.0f + 0.5f;
			float posy = posy_dominos - 0.5f;

		    for(int i=0;i<6;i++)
		    {
		    	// The base of the pendulums
		    	b2Body* b2;
				{
					b2PolygonShape shape;
					shape.SetAsBox(0.25f, 0.5f);
					  
					b2BodyDef bd;
					bd.position.Set(posx + 1.5f*i, posy - 1.0f);
					b2 = m_world->CreateBody(&bd);
					b2->CreateFixture(&shape, 5.0f);
				}
			    b2Body* b4;
			    {
					b2CircleShape shape;
				        shape.m_radius = 0.77;
					  
					b2BodyDef bd;
					bd.type = b2_dynamicBody;
					if(i==0)bd.position.Set(posx - 3.5f, posy + 4.5f);
					else
					bd.position.Set(posx + 1.5*i, posy + 1.0f);
					b4 = m_world->CreateBody(&bd);
					b4->CreateFixture(&shape, 2.0f);
			    }
			
			    b2RevoluteJointDef jd;
			    b2Vec2 anchor;
			    anchor.Set(posx + 1.5*i, posy + 4.5f);
			    jd.Initialize(b2, b4, anchor);
			    m_world->CreateJoint(&jd);
		    }
		}


	    //The pulley system
	    float posx_pulley = 2.0f;
	    float posy_pulley = 20.0f;
	    {
	    	float posx = posx_pulley;
	    	float posy = posy_pulley;
	    	float density_plane = 5.0f;

	    	b2BodyDef *bd = new b2BodyDef;
			bd->type = b2_dynamicBody;
			bd->position.Set(posx, posy);
			bd->fixedRotation = true;

			//The open box
			b2FixtureDef *fd1 = new b2FixtureDef;
			fd1->density = density_plane;
			fd1->friction = 0.5;
			fd1->restitution = 0.f;
			fd1->shape = new b2PolygonShape;
			b2PolygonShape bs1;
			bs1.SetAsBox(2,0.2, b2Vec2(0.f,-1.9f), 0);
			fd1->shape = &bs1;

			b2FixtureDef *fd2 = new b2FixtureDef;
			fd2->density = density_plane;
			fd2->friction = 0.5;
			fd2->restitution = 0.f;
			fd2->shape = new b2PolygonShape;
			b2PolygonShape bs2;
			bs2.SetAsBox(0.2,2, b2Vec2(2.0f,0.f), 0);
			fd2->shape = &bs2;

			b2FixtureDef *fd3 = new b2FixtureDef;
			fd3->density = density_plane;
			fd3->friction = 0.5;
			fd3->restitution = 0.f;
			fd3->shape = new b2PolygonShape;
			b2PolygonShape bs3;
			bs3.SetAsBox(0.2,2, b2Vec2(-2.0f,0.f), 0);
			fd3->shape = &bs3;

			b2Body* box1 = m_world->CreateBody(bd);
			box1->CreateFixture(fd1);
			box1->CreateFixture(fd2);
			box1->CreateFixture(fd3);

			//The bar
			bd->position.Set(posx + 10.0f, posy);	
			//	fd1.SetAsBox(15.0f, 0.2f);
			fd1->density = 3 * density_plane;	  
			b2Body* box2 = m_world->CreateBody(bd);
			box2->CreateFixture(fd1);

			// The pulley joint
			b2PulleyJointDef* myjoint = new b2PulleyJointDef();
			b2Vec2 worldAnchorOnBody1(posx, posy); // Anchor point on body 1 in world axis
			b2Vec2 worldAnchorOnBody2(posx + 10.0f, posy); // Anchor point on body 2 in world axis
			b2Vec2 worldAnchorGround1(posx, posy + 5.0f); // Anchor point for ground 1 in world axis
			b2Vec2 worldAnchorGround2(posx + 10.0f, posy + 5.0f); // Anchor point for ground 2 in world axis
			float32 ratio = 1.0f; // Define ratio
			myjoint->Initialize(box1, box2, worldAnchorGround1, worldAnchorGround2, box1->GetWorldCenter(), box2->GetWorldCenter(), ratio);
			m_world->CreateJoint(myjoint);
	    }

	    // Shelf to be activated by the pulley
	    {
	    	float posx = posx_pulley + 19.0f;
	    	float posy = posy_pulley + 3.0f;

			b2PolygonShape shape;
			shape.SetAsBox(5.0f, 0.25f);
			for(int i=0;i<4;i++)
			{
				b2BodyDef bd;
				bd.position.Set(posx, posy);
				b2Body* ground = m_world->CreateBody(&bd);
				ground->CreateFixture(&shape, 0.0f);
			}
			// Plank
		 	{
				b2PolygonShape shape;
				shape.SetAsBox(3.0f, 0.2f);

				b2BodyDef bd;
				bd.position.Set(posx - 4.5f, posy + 2.0f);
				bd.type = b2_dynamicBody;
				b2Body* body = m_world->CreateBody(&bd);
				b2FixtureDef *fd = new b2FixtureDef;
				fd->density = 1.0f;
				fd->shape = new b2PolygonShape;
				fd->shape = &shape;
				body->CreateFixture(fd);
		    }
		    // Ball
		    {
			    b2Body* sbody;
				b2CircleShape circle;
				circle.m_radius = 0.7;

				b2FixtureDef ballfd;
				ballfd.shape = &circle;
				ballfd.density = 2.0f;
				ballfd.friction = 0.1f;
				ballfd.restitution = 0.5f;
				b2BodyDef ballbd;
				ballbd.type = b2_dynamicBody;
				ballbd.position.Set(posx - 5.0f, posy + 3.0f);
				sbody = m_world->CreateBody(&ballbd);
				sbody->CreateFixture(&ballfd);
			}
		}
	    // Shelf to split
	    {
	    	float posx = posx_pulley + 31.0f;
	    	float posy = posy_pulley + 3.0f;

			b2PolygonShape shape;
			shape.SetAsBox(5.0f, 0.25f);
			for(int i=0;i<4;i++)
			{
				b2BodyDef bd;
				bd.position.Set(posx, posy);
				b2Body* ground = m_world->CreateBody(&bd);
				ground->CreateFixture(&shape, 0.0f);
			}
		    // Ball
		    {
			    b2Body* sbody;
				b2CircleShape circle;
				circle.m_radius = 0.7;

				b2FixtureDef ballfd;
				ballfd.shape = &circle;
				ballfd.density = 1.0f;
				ballfd.friction = 0.5f;
				ballfd.restitution = 0.8f;
				b2BodyDef ballbd;
				ballbd.type = b2_dynamicBody;
				ballbd.position.Set(posx - 5.0f, posy + 3.0f);
				sbody = m_world->CreateBody(&ballbd);
				sbody->CreateFixture(&ballfd);
			}

			for(int i=0;i<5;i++)
			Explosion* anar=new Explosion(25, posx-3+i*2 , 2.0+posy , m_world);
			m_world->SetContactListener(&myContactListenerInstance);
		}

		//The slant edge on which one of the small spheres slides down.
		{
			int steps = 3;						//number of steps
			float slantEdgePosX = 28.0f;		//coordinates
			float slantEdgePosY = 2.0f;		//coordinates
			float direction = 1;				//one when top-left to bottom right ; -1 for top-right to bottom-left
			float horSize = 5.5f;				// width of plank's shadow on X-axis
			float vertSize = 4.0f ;			// height of plank's shadow on Y-axis
			float shiftRight = 4.5f;			// horizontal distance between two opposite facing planks
			float shiftUp = vertSize;			// vertical distance between two opposite facing planks
			float distance = 2*vertSize;		//distance between two same-side facing steps
			for(int i =0 ; i< steps ; i++)
			{
				b2EdgeShape shape;                                            //  line segment used to define the path.
				shape.Set(b2Vec2(slantEdgePosX-horSize/2, slantEdgePosY+vertSize/2*direction + i*distance), b2Vec2(slantEdgePosX+horSize/2, slantEdgePosY-vertSize/2*direction + i*distance));      // The two end points of the path are specified.
				b2BodyDef bd;                                                 //Used to define the body.
				b1 = m_world->CreateBody(&bd);                            // Body created in the Box2D world.
				b1->CreateFixture(&shape, 0.0f);                      // Static Body, so the fixture is created without changing its default values.
			}

			direction = -1;
			slantEdgePosX += shiftRight;
			slantEdgePosY += shiftUp;
			for(int i =0 ; i< steps-1 ; i++)
			{
				b2EdgeShape shape;                                            //  line segment used to define the path.
				shape.Set(b2Vec2(slantEdgePosX-horSize/2, slantEdgePosY+vertSize/2*direction + i*distance), b2Vec2(slantEdgePosX+horSize/2, slantEdgePosY-vertSize/2*direction + i*distance));      // The two end points of the path are specified.
				b2BodyDef bd;                                                 //Used to define the body.
				b1 = m_world->CreateBody(&bd);                            // Body created in the Box2D world.
				b1->CreateFixture(&shape, 0.0f);                      // Static Body, so the fixture is created without changing its default values.
			}
		}

		//The conveyor belt
		//The idea is to make a rectangular chain
		//Then, we insert rotating discs "in" the chain
		//We keep a very high friction between these discs and the chain
		//We give the discs a constant angular velocity, due to which the conveyor belt rotates
		float posx_conveyer = posx_shelfs - 9.0f + 0.5f;
		float posy_conveyer = posy_shelfs + 7.2f;
		{
			//overall shift parameters for the entire conveyor belt system
			double conveyorleft = posx_conveyer;
			double conveyorup = posy_conveyer;
			//the radius of the disk
			double discRadius = 1.0;
			//The number of links in the conveyor belt system either above or below.
			//The total number of links will be 2*links + 4 (2 links also on either side)
			int links = 9; //This number must be divisible by 4 and > 20 so that 5 well spaced disks which rotate and drive the pulley can be layed out
			double speed = 4.0; //This is the speed at which the conveyor belts' driving wheels rotate
			//The links are made as follows.
			//First, we make the lower chain.
			//Then, we make the upper chain
			//Then, we make the two links which join these two chains on the left hand side
			//Then, we make the two links which join these two chains on the right hand side
			b2Body* leftup; //this is the body which is the top left link
			b2Body* rightup; //this is the body which is the top right link
			b2Body* leftdown; //this is the body which is the bottom left link
			b2Body* rightdown; //this is the body which is the bottom right link
			//Note that these four special links are required because they are used specifically to connect the upper and lower chains
			//here we initialize them to default initial values
			leftup = NULL;
			rightup = NULL;
			leftdown = NULL;
			rightdown = NULL;
			double density = 25.0f;
			double friction = 0.5f;
			double restitution = 0.05f;
			//Making the upper part of the conveyor belt
			{
				//defining each piece of the chain, along with the shape, size, fixture and relevant properties
				b2BodyDef bd;
				//The body is a dynamic body
				bd.type = b2_dynamicBody;
				bd.position.Set(conveyorleft, conveyorup);
				b2Body* ground = m_world->CreateBody(&bd);
				b2PolygonShape chainpiece;
				//Dimensions
				chainpiece.SetAsBox(1, 0.1);
				b2FixtureDef chainfix;
				//Properties
				chainfix.density = density;
				chainfix.friction = friction;
				chainfix.restitution = restitution;
				chainfix.shape = &chainpiece;
				//Defining a revolute joint for our purpose
				b2Body *lastLink = ground;
				b2RevoluteJointDef chainJoint;
				//If the bodies are connected in the Box 2D world, then they should not collide. This is why this line is needed.
				chainJoint.collideConnected = false;
				for(int32 i = 0; i < links; i++)
				{
					double cons = 2.0;//This is the difference in length in between two anchors
					b2BodyDef bd;
					bd.type = b2_dynamicBody;
					bd.position.Set(i*cons + conveyorleft, 0.0 + conveyorup);//setting up a new dynamic chain link
					b2Body* nextLink = m_world->CreateBody(&bd);
					nextLink->CreateFixture(&chainfix);
					b2Vec2 anchor(i*cons - 1.0 + conveyorleft, 0.0 + conveyorup);//The anchor is set exactly at the meeting point of two chain links
					if(i > 0)//We should not define a link for the first piece alone. We need atleast two pieces to begin linking
					{
						chainJoint.Initialize(lastLink, nextLink, anchor);
						m_world->CreateJoint(&chainJoint);
					}
					//Updating the current link
					lastLink = nextLink;
					//assignment to the last link. The last link will essentially be rightdown, once we exit this loop
					rightdown = nextLink;
					//for the first link, we need to set leftdown, which is supposed to point to that link (body)
					if(i == 0)
					{
						leftdown = nextLink;
					}
				}
			}
			//here we make the lower part of the conveyor belt chain
			{
				double shiftup = 2.0*discRadius;//this denotes the height to which we shift up the upper part of the chain
				// The rest of the process is identical to the first part except that everything is shifted up
				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position.Set(conveyorleft, conveyorup + shiftup);
				b2Body* ground = m_world->CreateBody(&bd);
				b2PolygonShape chainpiece;
				chainpiece.SetAsBox(1, 0.1);
				b2FixtureDef chainfix;
				chainfix.density = density;
				chainfix.friction = friction;
				chainfix.restitution = restitution;
				chainfix.shape = &chainpiece;
				b2Body *lastLink = ground;
				b2RevoluteJointDef chainJoint;
				chainJoint.collideConnected = false;
				for(int32 i = 0; i < links; i++)
				{
					double cons = 2.0;
					b2BodyDef bd;
					bd.type = b2_dynamicBody;
					bd.position.Set(i*cons + conveyorleft, shiftup + conveyorup);
					b2Body* nextLink = m_world->CreateBody(&bd);
					nextLink->CreateFixture(&chainfix);
					b2Vec2 anchor(i*cons - 1.0 + conveyorleft, shiftup + conveyorup);
					if(i > 0)
					{
						chainJoint.Initialize(lastLink, nextLink, anchor);
						m_world->CreateJoint(&chainJoint);
					}
					lastLink = nextLink;
					rightup = nextLink;//"rightup" is updated to the right most link in the top part of the chain in every iteration
					if(i == 0)
					{
						leftup = nextLink;//"leftup" stores information about the topleft link
						//More Precisely, it stores a link to the body of the upper left link
					}
				}
			}
			//Now, we make the two links on the left hand side which connect the upper and lower halves of the chain
			{
				b2BodyDef bd;
				bd.type = b2_dynamicBody;//Note that this is a dynamic body
				//Position
				bd.position.Set(-1.0 + conveyorleft, conveyorup + 0.5);
				b2Body* ground = m_world->CreateBody(&bd);
				b2PolygonShape chainpiece;
				//Dimensions. Note that the dimensions are reversed, so that this is a vertical link, while the earlier one was a horizontal link
				chainpiece.SetAsBox(0.2, 1, b2Vec2(0.0,0.0),0.0);
				//Again defining a fixture for the chain
				b2FixtureDef chainfix;
				//Chain Properties
				chainfix.density = 50;
				chainfix.friction = 0.5;
				chainfix.restitution = 0.5;
				chainfix.shape = &chainpiece;
				//Creating the first vertical link on the left
				ground->CreateFixture(&chainfix);
				bd.position.Set(-1.0 + conveyorleft, conveyorup + 1.0);
				//Creating the second vertical link on the left
				b2Body*groundnew = m_world->CreateBody(&bd);
				groundnew->CreateFixture(&chainfix);
				//defining the revolute joint to connect links up
				b2RevoluteJointDef chainJoint;
				chainJoint.collideConnected = false;
				b2Vec2 anchor( conveyorleft - 1.0, conveyorup);
				//Connecting the lower left horizontal link and the lower vertical link
				chainJoint.Initialize(leftdown, ground, anchor);
				m_world->CreateJoint(&chainJoint);
				b2RevoluteJointDef chainJoint2;
				chainJoint2.collideConnected = false;
				b2Vec2 anchor2( conveyorleft - 1.0, 1.0 + conveyorup);
				//Connecting the lower and upper vertical links
				chainJoint2.Initialize(ground, groundnew, anchor2);
				m_world->CreateJoint(&chainJoint2);
				b2RevoluteJointDef chainJoint3;
				chainJoint3.collideConnected = false;
				b2Vec2 anchor3( conveyorleft - 1.0, 2.0 + conveyorup);
				//Connecting the upper vertical link and the top left horizontal link
				chainJoint3.Initialize(groundnew, leftup, anchor3);
				m_world->CreateJoint(&chainJoint3);
			}
			//Now again we connect the top and bottom portions of the conveyor belt, this time at the right end.
			{
				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				//Note that the position is shifted towards the rights by a number proportional to the number of links in the chain
				bd.position.Set(2.0 * links - 1.0+ conveyorleft, conveyorup + 0.5);
				b2Body* ground = m_world->CreateBody(&bd);
				b2PolygonShape chainpiece;
				chainpiece.SetAsBox(0.2, 1, b2Vec2(0.0,0.0),0.0);
				//Defining the links and setting properties
				b2FixtureDef chainfix;
				chainfix.density = 50;
				chainfix.friction = 0.5;
				chainfix.restitution = 0.5;
				chainfix.shape = &chainpiece;
				//Making the lower vertical RHS link
				ground->CreateFixture(&chainfix);
				//Making the upper vertical RHS link
				bd.position.Set(2.0* links - 1.0+ conveyorleft, conveyorup + 1.0);
				b2Body*groundnew = m_world->CreateBody(&bd);
				groundnew->CreateFixture(&chainfix);
				//Defining the revolute joint
				b2RevoluteJointDef chainJoint;
				chainJoint.collideConnected = false;
				b2Vec2 anchor( 2.0 * links - 1.0 + conveyorleft, 0.0 + conveyorup);
				//Joining the right most link of the lower chain and the lower vertical link on the right
				chainJoint.Initialize(rightdown, ground, anchor);
				m_world->CreateJoint(&chainJoint);
				b2RevoluteJointDef chainJoint2;
				chainJoint2.collideConnected = false;
				b2Vec2 anchor2( 2.0 * links - 1.0+ conveyorleft, 1.0 + conveyorup);
				//Joining the lower and upper right vertical links
				chainJoint2.Initialize(ground, groundnew, anchor2);
				m_world->CreateJoint(&chainJoint2);
				b2RevoluteJointDef chainJoint3;
				chainJoint3.collideConnected = false;
				b2Vec2 anchor3( 2.0 * links - 1.0+ conveyorleft, 2.0 + conveyorup);
				//joining the upper right vertical link with the top right horizontal link
				chainJoint3.Initialize(groundnew, rightup, anchor3);
				m_world->CreateJoint(&chainJoint3);
			}
			//Now, we create the rotating high friction discs
			//
			//The parameters are similar for all the discs, except that they are all translated along the x axis
			//The left most Disc:
			{
				b2BodyDef bd;
				bd.type = b2_kinematicBody;//This disc is a kinematic body
				bd.position.Set(conveyorleft + 0.0, conveyorup + 1.0);//Position
				bd.angle = 0;//starts rotating at angle 0
				b2Body* body = m_world->CreateBody(&bd);
				b2CircleShape circle;//shape of body = circular
				circle.m_radius = discRadius;//radius of disc used (same as the rectangular length of the boxes)
				b2FixtureDef fd;
				//properties of the disc
				fd.shape = &circle;
				fd.density = 100;
				fd.friction = 1;//Note the friction. It is set as 1!
				body->CreateFixture(&fd);
				body->SetAngularVelocity(speed);
			}
			//The last disc (right most disc)
			{
				b2BodyDef bd;
				bd.type = b2_kinematicBody;
				//Note the shift in position from the earlier disc
				bd.position.Set(links * 2.0 + conveyorleft + -2.0, conveyorup + 1.0);
				bd.angle = 0;
				b2Body* body = m_world->CreateBody(&bd);
				b2CircleShape circle;
				circle.m_radius = discRadius;
				b2FixtureDef fd;
				fd.shape = &circle;
				fd.density = 100;
				fd.friction = 1;
				body->CreateFixture(&fd);
				body->SetAngularVelocity(speed);
			}
			//The second disc from the left hand side
			{
				b2BodyDef bd;
				bd.type = b2_kinematicBody;
				//Note the position
				bd.position.Set(links/2 * 2.0 + conveyorleft + -2.0, conveyorup + 1.0);
				bd.angle = 0;
				b2Body* body = m_world->CreateBody(&bd);
				b2CircleShape circle;
				circle.m_radius = discRadius;
				b2FixtureDef fd;
				fd.shape = &circle;
				fd.density = 100;
				fd.friction = 1;
				body->CreateFixture(&fd);
				body->SetAngularVelocity(speed);
			}
			//The third disc from the left hand side
			{
				b2BodyDef bd;
				bd.type = b2_kinematicBody;
				//Note the position
				bd.position.Set(3 * links/4 * 2.0 + conveyorleft + -2.0, conveyorup + 1.0);
				bd.angle = 0;
				b2Body* body = m_world->CreateBody(&bd);
				b2CircleShape circle;
				circle.m_radius = discRadius;
				b2FixtureDef fd;
				fd.shape = &circle;
				fd.density = 100;
				fd.friction = 1;
				body->CreateFixture(&fd);
				body->SetAngularVelocity(speed);
			}
		}
	}
	sim_t *sim = new sim_t("Dominos", dominos_t::create);
}