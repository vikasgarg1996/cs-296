/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

/*
 * Base code for CS 296 Software Systems Lab
 * Department of Computer Science and Engineering, IIT Bombay
 * Instructor: Parag Chaudhuri
 */


#include "cs296_base.hpp"
#include "render.hpp"

#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include "GL/freeglut.h"
#endif

#include <cstring>
using namespace std;

#include "dominos.hpp"

namespace cs296
{
  /**  The is the constructor
   * This is the documentation block for the constructor.
   */

  dominos_t::dominos_t()
  {
    //Ground
   /*! \par Making Ground
	 * variable b1 :: <br>
	 * Data Type is b2Body* <br>
	 * pointer to the ground <br>
	 * variable shape :: <br>
	 * Data Type is b2EdgeShape <br>
	 * Its left value is (-90,0) and right value is (90,0) <br>
	 * variable bd :: <br>
	 * Data Type is b2BodyDef <br>
	 * CreateBody is used for Creating Bodies in the Physical world <br>
	 * CreateFixture is used for fixing body in its parent <br>
	 */
    b2Body* b1;
    {

      b2EdgeShape shape;
      shape.Set(b2Vec2(-90.0f, 0.0f), b2Vec2(90.0f, 0.0f));
      b2BodyDef bd;
      b1 = m_world->CreateBody(&bd);
      b1->CreateFixture(&shape, 0.0f);
    }

    //Top horizontal shelf
    /*! \par Top Horizontal Shelf
     * variable shape :: <br>
     * Data Type is b2PolygonShape <br>
     * length : 6 , width : 0.25 , Its Center coordinates are (-31,30) <br>
     * variable bd :: <br>
     * Data Type is b2BodyDef <br>
     * variable ground :: <br>
     * Data Type is b2Body* <br>
     */
    {
      b2PolygonShape shape;
      shape.SetAsBox(6.0f, 0.25f);

      b2BodyDef bd;
      bd.position.Set(-31.0f, 30.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 34.0f);
    }

    //Dominos
    /*! \par Dominos
     * variable shape :: <br>
     * Data Type is b2PolygonShape <br>
     * length : 0.1 , width : 1 <br>
     * 10 dominos are created using a for loop and center position of each domino is different <br>
     * variable fd :: <br>
     * Data Type is b2FixtureDef <br>
     * Density : 20 , Friction : 0.1 <br>
     * variable bd :: <br>
     * Data Type is b2BodyDef <br>
     * type is set to be dynamic <br>
     * variable body :: <br>
     * Data Type is b2Body* <br>
     */
    {
      b2PolygonShape shape;
      shape.SetAsBox(0.1f, 1.0f);

      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 20.0f;
      fd.friction = 0.1f;
      
      for (int i = 0; i < 10; ++i)
	{
	  b2BodyDef bd;
	  bd.type = b2_dynamicBody;
	  bd.position.Set(-35.5f + 1.0f * i, 31.25f);
	  
	  b2Body* body = m_world->CreateBody(&bd);
	  body->CreateFixture(&fd);
	}
    }

    //Another horizontal shelf
    /*! \par Another Horizontal Shelf
     * variable shape :: <br>
     * Data Type is b2PolygonShape <br>
     * length : 7 , width : 0.25 , Its Center coordinates are (-19,26) and Angle is 0 <br>
     * variable bd :: <br>
     * Data Type is b2BodyDef <br>
     * variable ground :: <br>
     * Data Type is b2Body* <br>
     */
    {
      b2PolygonShape shape;
      shape.SetAsBox(7.0f, 0.25f, b2Vec2(-20.f,20.f), 0.0f);

      b2BodyDef bd;
      bd.position.Set(1.0f, 6.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }


    //The pendulum that knocks the dominos off
    /*! \par The pendulum that knocks the dominos off
     * variable shape :: <br>
     * Data Type is b2PolygonShape <br>
     * length : 0.25 , width : 1.5 <br>
     * variable bd :: <br>
     * Data Type is b2BodyDef <br>
     * Its position set to be (-36.5,28) <br>
     * Pendulam Bob is attached to it <br>
     * variable b2 :: <br>
     * Data Type is b2Body* <br> <br>
     * variable shape :: <br>
     * Data Type is b2PolygonShape <br>
     * length : 0.25 , width : 0.25 <br>
     * variable bd :: <br>
     * Data Type is b2BodyDef <br>
     * Its position set to be (-40,33) <br>
     * Pendulam's Bob <br>
     * type is dynamic <br>
     * variable b4 :: <br>
     * Data Type is b2Body* <br><br>
     * variable jd :: <br>
     * Data Type is b2RevoluteJointDef <br>
     * connecting rope and bob <br>
     * Bob is set at coordinates(-37,40) <br>
     */
    {
      b2Body* b2;
      {
	b2PolygonShape shape;
	shape.SetAsBox(0.25f, 1.5f);

	b2BodyDef bd;
	bd.position.Set(-36.5f, 28.0f);
	b2 = m_world->CreateBody(&bd);
	b2->CreateFixture(&shape, 10.0f);
      }

      b2Body* b4;
      {
	b2PolygonShape shape;
	shape.SetAsBox(0.25f, 0.25f);

	b2BodyDef bd;
	bd.type = b2_dynamicBody;
	bd.position.Set(-40.0f, 33.0f);
	b4 = m_world->CreateBody(&bd);
	b4->CreateFixture(&shape, 2.0f);
      }

      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(-37.0f, 40.0f);
      jd.Initialize(b2, b4, anchor);
      m_world->CreateJoint(&jd);
    }

    //The train of small spheres
    /*! \par The train of small spheres
     * variable circle :: <br>
     * Data Type is b2CircleShape <br>
     * radius : 0.5 <br>
     * 10 circles are created using a for loop and center position of each circle is different <br>
     * variable ballfd :: <br>
     * Data Type is b2FixtureDef <br>
     * Density : 1 , Friction : 0.0 , Restitution : 0.0 <br>
     * variable ballbd :: <br>
     * Data Type is b2BodyDef <br>
     * Type is Dynamic <br>
     * Position is (-22.2+i,26.6) of ith circle from the left <br>
     * variable spherebody :: <br>
     * Data Type is b2Body* <br>
     */
    {
      b2Body* spherebody;

      b2CircleShape circle;
      circle.m_radius = 0.5;

      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 1.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;

      for (int i = 0; i < 10; ++i)
	{
	  b2BodyDef ballbd;
	  ballbd.type = b2_dynamicBody;
	  ballbd.position.Set(-22.2f + i*1.0, 26.6f);
	  spherebody = m_world->CreateBody(&ballbd);
	  spherebody->CreateFixture(&ballfd);
	}
    }

    //The pulley system
    /*! \par The pulley system
     * variable bd :: <br>
     * Data Type is b2BodyDef* <br>
     * Type is defined dynamic <br>
     * Its position set to be (-10,15) <br>
     * Fixed Rotation set to be true to remove its rotating degree of freedom <br><br>
     * \par The Open Box
     * variable fd1 :: <br>
     * Data Type is b2FixtureDef* <br>
     * density : 10 , friction : 0.5 , restitution : 0 <br>
     * variable bs1 :: <br>
     * Data Type is b2PolygonShape <br>
     * length : 2 , width : 0.2 its center coordinates are (0,-1.9) , Angle : 0 <br> <br>
     * variable fd2 :: <br>
     * Data Type is b2FixtureDef* <br>
     * density : 10 , friction : 0.5 , restitution : 0 <br>
     * variable bs2 :: <br>
     * Data Type is b2PolygonShape <br>
     * length : 0.2 , width : 2 its center coordinates are (2,0) , Angle : 0 <br> <br>
     * variable fd3 :: <br>
     * Data Type is b2FixtureDef* <br>
     * density : 10 , friction : 0.5 , restitution : 0 <br>
     * variable bs3 :: <br>
     * Data Type is b2PolygonShape <br>
     * length : 0.2 , width : 2 its center coordinates are (-2,0) , Angle : 0 <br> <br>
     * variable box1 :: <br>
     * Data Type is b2Body* <br>
     * \par The Bar
     * position of bd is resetted to (10,15) <br>
     * density of fd1 is changed to 34 <br>
     * variable box2 :: <br>
     * Data Type is b2Body* <br>
     * fd1 is created again with new parameters <br>
     * \par The Pully Joint
     * variable myjoint :: <br>
     * Data Type is b2PulleyJointDef* <br>
     * 4 2D vectors are defined <br>
     * worldAnchorOnBody1(-10, 15) , worldAnchorOnBody2(10, 15) ,worldAnchorGround1(-10, 20) ,worldAnchorGround2(10, 20) <br>
     * Ratio is Defined to be 1 <br>
     * intialise myjoint with box1,box2 and all 4 2D vectors <br>
     */
    {
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(-10,15);
      bd->fixedRotation = true;

      //The open box
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 10.0;
      fd1->friction = 0.5;
      fd1->restitution = 0.f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(2,0.2, b2Vec2(0.f,-1.9f), 0);
      fd1->shape = &bs1;
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 10.0;
      fd2->friction = 0.5;
      fd2->restitution = 0.f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(0.2,2, b2Vec2(2.0f,0.f), 0);
      fd2->shape = &bs2;
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 10.0;
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
      bd->position.Set(10,15);
      fd1->density = 34.0;
      b2Body* box2 = m_world->CreateBody(bd);
      box2->CreateFixture(fd1);

      // The pulley joint
      b2PulleyJointDef* myjoint;
      b2Vec2 worldAnchorOnBody1(-10, 15); // Anchor point on body 1 in world axis
      b2Vec2 worldAnchorOnBody2(10, 15); // Anchor point on body 2 in world axis
      b2Vec2 worldAnchorGround1(-10, 20); // Anchor point for ground 1 in world axis
      b2Vec2 worldAnchorGround2(10, 20); // Anchor point for ground 2 in world axis
      float32 ratio = 1.0f; // Define ratio
      myjoint->Initialize(box1, box2, worldAnchorGround1, worldAnchorGround2, box1->GetWorldCenter(), box2->GetWorldCenter(), ratio);
      m_world->CreateJoint(myjoint);
    }

    //The revolving horizontal platform
    /*! \par Revolving Horizontal Platform
     * variable shape :: <br>
     * Data Type is b2PolygonShape <br>
     * length : 2.2 , width : 0.2 <br>
     * variable bd :: <br>
     * Data Type is b2BodyDef <br>
     * Position is (14,16) <br>
     * Type is dynamic <br>
     * variable fd :: <br>
     * density : 1 <br>
     * variable bd2 :: <br>
     * Data Type is b2BodyDef <br>
     * Position is (14,16) <br><br>
     * variable jointDef; <br>
     * Data Type is b2RevoluteJointDef <br>
     * It is used to connect two bodies from each other <br>
     * collideConnected is set to be false so that there is no collision between them <br>
     * variable ground :: <br>
     * Data Type is b2Body* <br>
     */
    {
      b2PolygonShape shape;
      shape.SetAsBox(2.2f, 0.2f);

      b2BodyDef bd;
      bd.position.Set(14.0f, 16.0f);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);
      b2BodyDef bd2;
      bd2.position.Set(14.0f, 16.0f);
      b2Body* body2 = m_world->CreateBody(&bd2);

      b2RevoluteJointDef jointDef;
      jointDef.bodyA = body;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(0,0);
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);
    }

    //The heavy sphere on the platform
    /*! \par The Heavy Sphere on the Platform
     * variable circle :: <br>
     * Data Type is b2CircleShape <br>
     * radius : 1 <br>
     * variable ballfd :: <br>
     * Data Type is b2FixtureDef <br>
     * Density : 40 , Friction : 0.0 , Restitution : 0.0 <br>
     * variable ballbd :: <br>
     * Data Type is b2BodyDef <br>
     * Type is set to be Dynamic <br>
     * Position is (14,18) <br>
     * variable sbody :: <br>
     * Data Type is b2Body* <br>
     */
    {
      b2Body* sbody;
      b2CircleShape circle;
      circle.m_radius = 1.0;

      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 40.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(14.0f, 18.0f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
    }


    //The see-saw system at the bottom
    /*! \par The See-saw system at the bottom
     *  \par Triangle Wedge
     * variable poly :: <br>
     * Data Type is b2PolygonShape <br>
     * 3 2D vertices are defined (-1,0) (1,0) (0,1.5) <br>
     * variable wedgefd :: <br>
     * Data Type is b2FixtureDef <br>
     * Density : 10 , Friction : 0.0 , Restitution : 0.0 <br>
     * variable wedgebd :: <br>
     * Data Type is b2BodyDef <br>
     * Position is (30,0) <br>
     * variable sbody :: <br>
     * Data Type is b2Body* <br>
     */
    {
      //The triangle wedge
      b2Body* sbody;
      b2PolygonShape poly;
      b2Vec2 vertices[3];
      vertices[0].Set(-1,0);
      vertices[1].Set(1,0);
      vertices[2].Set(0,1.5);
      poly.Set(vertices, 3);
      b2FixtureDef wedgefd;
      wedgefd.shape = &poly;
      wedgefd.density = 10.0f;
      wedgefd.friction = 0.0f;
      wedgefd.restitution = 0.0f;
      b2BodyDef wedgebd;
      wedgebd.position.Set(30.0f, 0.0f);
      sbody = m_world->CreateBody(&wedgebd);
      sbody->CreateFixture(&wedgefd);

      //The plank on top of the wedge
      /*! \par Plank on top of the wedge
     * variable shape :: <br>
     * Data Type is b2PolygonShape <br>
     * length : 20 , width : 0.2 <br>
     * variable bd2 :: <br>
     * Data Type is b2BodyDef <br>
     * Position is (30,1.5) <br>
     * Type of body is Dynamic <br>
     * variable fd2 :: <br>
     * Data Type is b2FixtureDef* <br>
     * Density : 1  <br>
     * variable body :: <br>
     * Data Type is b2Body* <br>
     * variable jd :: <br>
     * Data Type is b2RevoluteJointDef <br>
     * Joint point is set to be (30,1.5) <br>
     */
      b2PolygonShape shape;
      shape.SetAsBox(20.0f, 0.2f);
      b2BodyDef bd2;
      bd2.position.Set(30.0f, 1.5f);
      bd2.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd2);
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 1.f;
      fd2->shape = new b2PolygonShape;
      fd2->shape = &shape;
      body->CreateFixture(fd2);

      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(30.0f, 1.5f);
      jd.Initialize(sbody, body, anchor);
      m_world->CreateJoint(&jd);

      //The light box on the right side of the see-saw
      /*! \par Light box on the right side of the see-saw
     * variable shape2 :: <br>
     * Data Type is b2PolygonShape <br>
     * length : 2 , width : 2 <br>
     * variable bd3 :: <br>
     * Data Type is b2BodyDef <br>
     * Position is (45,2) <br>
     * Type of body is Dynamic <br>
     * variable fd3 :: <br>
     * Data Type is b2FixtureDef* <br>
     * Density : 0.01  <br>
     * variable body3 :: <br>
     * Data Type is b2Body* <br>
     */
      b2PolygonShape shape2;
      shape2.SetAsBox(2.0f, 2.0f);
      b2BodyDef bd3;
      bd3.position.Set(45.0f, 2.0f);
      bd3.type = b2_dynamicBody;
      b2Body* body3 = m_world->CreateBody(&bd3);
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 0.01f;
      fd3->shape = new b2PolygonShape;
      fd3->shape = &shape2;
      body3->CreateFixture(fd3);
    }

      //New Things added
      /*! \par New Objects added for assignment <br>
     * \par Slant wedge
     * variable bdoo :: <br>
     * Data Type is b2BodyDef <br>
     * Position is (19,11) <br>
     * variable fd1 :: <br>
     * Data Type is b2FixtureDef* <br>
     * Density : 10 , friction : 0 , restitution : 0 <br>
     * variable :: bs1 <br>
     * Data Type is b2PolygonShape <br>
     * length : 2 , width : 0.2 , position is set to be (1,0) according to the position of body , Angle 0 <br> <br>
     * variable fd2 :: <br>
     * Data Type is b2FixtureDef* <br>
     * Density : 10 , friction : 0 , restitution : 0 <br>
     * variable :: bs2 <br>
     * Data Type is b2PolygonShape <br>
     * length : 2 , width : 0.2 , position is set to be (-2.5,1.5) according to the position of body , Angle is 2.5 <br>
     * variable box1 :: <br>
     * Data Type is b2Body* <br>
     */
	{
      b2BodyDef bdoo;
      bdoo.position.Set(19,11);
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 10.0;
      fd1->friction = 0.0;
      fd1->restitution = 0.f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(2,0.2, b2Vec2(1.0,0.0), 0);
      fd1->shape = &bs1;
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 10.0;
      fd2->friction = 0.0;
      fd2->restitution = 0.f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(2,0.2, b2Vec2(-2.5f,1.5f), 2.5);
      fd2->shape = &bs2;

      b2Body* box1 = m_world->CreateBody(&bdoo);
      box1->CreateFixture(fd1);
      box1->CreateFixture(fd2);
    }

    //Another pendulum
    /*! \par Second Pendulum  <br>
     * \par Attaching Floor
     * variable :: shape <br>
     * Data Type is b2PolygonShape <br>
     * length : 1.5 , width : 0.25 <br>
     * variable bd :: <br>
     * Data Type is b2BodyDef <br>
     * Position is (25,26) <br>
     * variable b2 :: <br>
     * Data Type is b2Body* <br>
     * density : 10 <br>
     * \par Bob of Pendulum
     * variable :: shape <br>
     * Data Type is b2CircleShape <br>
     * radius : 1 <br>
     * variable bd :: <br>
     * Data Type is b2BodyDef <br>
     * Position is (25,12) <br>
     * variable b2 :: <br>
     * Data Type is b2Body* <br>
     * density : 0.01 <br><br>
     * variable jd :: <br>
     * Data Type is b2RevoluteJointDef <br>
     * Joint is set to be at (25,26) <br>
     * Pendulum is used for controlling the speed of ball of revolving plateform <br>     
     */
    {
	// Floor
      b2Body* b2;
      {
	b2PolygonShape shape;
	shape.SetAsBox(1.5f, 0.25f);

	b2BodyDef bd;
	bd.position.Set(25.0f, 26.0f);
	b2 = m_world->CreateBody(&bd);
	b2->CreateFixture(&shape, 10.0f);
      }
      
      //Bob
      b2Body* b4;
      {
	b2CircleShape shape;
	shape.m_radius = 1.0;

	b2BodyDef bd;
	bd.type = b2_dynamicBody;
	bd.position.Set(25.0f, 12.0f);
	b4 = m_world->CreateBody(&bd);
	b4->CreateFixture(&shape, 0.01f);
      }

      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(25.0f, 26.0f);
      jd.Initialize(b2, b4, anchor);
      m_world->CreateJoint(&jd);
    }

	//vertical wall
	/*! \par Vertical Wall
     * variable bdoo :: <br>
     * Data Type is b2BodyDef <br>
     * Position is (29,7) <br>
     * variable fd2 :: <br>
     * Data Type is b2FixtureDef* <br>
     * Density : 10 , friction : 0 , restitution : 0.55 <br>
     * variable :: bs2 <br>
     * Data Type is b2PolygonShape <br>
     * length : 3 , width : 0.2 , position is set to be (0,2.5) according to the position of body , Angle 1.57 radians <br> <br>
     * variable box1 :: <br>
     * Data Type is b2Body* <br>
     */
	
    {
      b2BodyDef bdoo;
      bdoo.position.Set(29,7);

      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 10.0;
      fd2->friction = 0.0;
      fd2->restitution = 0.55;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(3,0.2, b2Vec2(0.0f,2.5f), 1.57);
      fd2->shape = &bs2;

      b2Body* box1 = m_world->CreateBody(&bdoo);
      box1->CreateFixture(fd2);
    }

    // Small ball near light box
    /*! \par Small ball near light box
     * variable :: shape <br>
     * Data Type is b2CircleShape <br>
     * radius : 0.2 <br>
     * variable bd :: <br>
     * Data Type is b2BodyDef <br>
     * Type is set to be Dynamic <br>
     * Position is (37,2) <br>
     * variable b4 :: <br>
     * Data Type is b2Body* <br>
     * density : 0.1 <br><br>
     */
	b2Body* b4;
    {
		b2CircleShape shape;
		shape.m_radius = 0.2f;

		b2BodyDef bd;
		bd.type = b2_dynamicBody;
		bd.position.Set(37.0f, 2.0f);
		b4 = m_world->CreateBody(&bd);
		b4->CreateFixture(&shape, 0.1f);
     }

     // Ball upon the bar of Pulley
     /*! \par Ball upon the bar of Pulley
     * variable :: shape <br>
     * Data Type is b2CircleShape <br>
     * radius : 1 <br>
     * variable bd :: <br>
     * Data Type is b2BodyDef <br>
     * Type is set to be Dynamic <br>
     * Position is (10,15) <br>
     * variable b5 :: <br>
     * Data Type is b2Body* <br>
     * density : 0.01 <br><br>
     */
     b2Body* b5;
    {
		b2CircleShape shape;
		shape.m_radius = 1.0f;

		b2BodyDef bd;
		bd.type = b2_dynamicBody;
		bd.position.Set(10.0f, 15.0f);
		b5 = m_world->CreateBody(&bd);
		b5->CreateFixture(&shape, 0.01f);
     }

     //Stopper
     /*! \par Vertical Wall
     * variable bdoo :: <br>
     * Data Type is b2BodyDef <br>
     * Position is (2.7,10.785) <br>
     * variable fd2 :: <br>
     * Data Type is b2FixtureDef* <br>
     * Density : 10 , friction : 0 , restitution : 0.55 <br>
     * variable :: bs2 <br>
     * Data Type is b2PolygonShape <br>
     * length : 2 , width : 0.2 , position is set to be (0,2.5) according to the position of body , Angle 1.57 radians <br> <br>
     * variable box1 :: <br>
     * Data Type is b2Body* <br>
     */
    {
      b2BodyDef bdoo;
      bdoo.position.Set(2.7,10.785);

      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 10.0;
      fd2->friction = 0.0;
      fd2->restitution = 0.55;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(2,0.2, b2Vec2(0.0f,2.5f), 1.57);
      fd2->shape = &bs2;

      b2Body* box1 = m_world->CreateBody(&bdoo);
      box1->CreateFixture(fd2);
    }

     //Rotational Horizontal plank
     /*! \par Rotational Horizontal plank
     * variable bdoo :: <br>
     * Data Type is b2BodyDef <br>
     * Position is (0,15.5) <br>
     * variable fd2 :: <br>
     * Data Type is b2FixtureDef* <br>
     * Density : 0.1 , friction : 0 , restitution : 0 <br>
     * variable :: bs2 <br>
     * Data Type is b2PolygonShape <br>
     * length : 0.2 , width : 3 , position is set to be (0,0) according to the position of body , Angle 1.57 radians <br> <br>
     * variable box1 :: <br>
     * Data Type is b2Body* <br>
     * variable bd2 :: <br>
     * Data Type is b2BodyDef <br>
     * Position is (0,15.5) <br>
     * variable box2 :: <br>
     * Data Type is b2Body* <br>
     * variable jointDef; <br>
     * Data Type is b2RevoluteJointDef <br>
     * It is used to connect two bodies from each other <br>
     * collideConnected is set to be false so that there is no collision between them <br>
     */
    {
      b2BodyDef bdoo;
      bdoo.position.Set(0,15.5);
      bdoo.type = b2_dynamicBody;

      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 0.1;
      fd2->friction = 0.0;
      fd2->restitution = 0.0;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(0.2,3, b2Vec2(0.0f,0.0f), 1.57);
      fd2->shape = &bs2;

      b2Body* box1 = m_world->CreateBody(&bdoo);
      box1->CreateFixture(fd2);

      b2BodyDef bd2;
      bd2.position.Set(0.0f, 15.5f);
      b2Body* box2 = m_world->CreateBody(&bd2);

      b2RevoluteJointDef jointDef;
      jointDef.bodyA = box1;
      jointDef.bodyB = box2;
      jointDef.localAnchorA.Set(0,0);
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);
    }
    
    /*! \par Ball upon Stopper
     * variable :: shape <br>
     * Data Type is b2CircleShape <br>
     * radius : 1 <br>
     * variable bd :: <br>
     * Data Type is b2BodyDef <br>
     * Type is set to be Dynamic <br>
     * Position is (2,17.5) <br>
     * variable b9 :: <br>
     * Data Type is b2Body* <br>
     * density : 0.01 <br><br>
     */
     
    b2Body* b9;
    {
		b2CircleShape shape;
		shape.m_radius = 1.0f;

		b2BodyDef bd;
		bd.type = b2_dynamicBody;
		bd.position.Set(2.0f, 17.5f);
		b9 = m_world->CreateBody(&bd);
		b9->CreateFixture(&shape, 0.01f);
     }
  }

  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
