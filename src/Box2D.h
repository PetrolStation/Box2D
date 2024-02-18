#pragma once

#include <Core/Aliases.h>
#include "Core/Components/Component.h"
#include "Core/Components/Properties.h"
#include "LinearMath/btVector3.h"
#include "box2d/b2_body.h"
#include "box2d/b2_math.h"
#include "box2d/b2_polygon_shape.h"
#include "box2d/b2_world.h"
#include "box2d/b2_world_callbacks.h"
#include "glm/fwd.hpp"
#include <Static/Window/Window.h>
#include <Core/Physics/ColliderApi.h>
#include <Core/Physics/PhysicsController.h>
#include <box2d/box2d.h>
#include <Core/Components/Transform.h>
#include <Core/Scene.h>
#include <Core/Components/Entity.h>
//#include "Core/Physics/Collider.h"
#include <Core/Physics/ColliderApi.h>
#include "/home/samuel/Desktop/projects/EnchantedRealms/EnchantedRealms/src/Enemy.h"

namespace PetrolEngine{
  class Listener: public b2ContactListener{
    public:
      void BeginContact(b2Contact* contact) {
        Entity* entityA = (Entity*)contact->GetFixtureA()->GetUserData().pointer;
        Entity* entityB = (Entity*)contact->GetFixtureB()->GetUserData().pointer;
        
        Entity* bullet = nullptr;
        Entity* enemy = nullptr;

        if(entityA->hasComponent<Properties>()){
          if(strcmp(entityA->getComponent<Properties>().name, "enemy" ) == 0){ enemy  = entityA; }
          if(strcmp(entityA->getComponent<Properties>().name, "bullet") == 0){ bullet = entityA; }
        }

        if(entityB->hasComponent<Properties>()){
          if(strcmp(entityB->getComponent<Properties>().name, "enemy" ) == 0){ enemy  = entityB; }
          if(strcmp(entityB->getComponent<Properties>().name, "bullet") == 0){ bullet = entityB; }
        }
        if(bullet != nullptr && enemy != nullptr){
          //enemy->getComponent<Enemy>().hp -=100;
          enemy->destroy();
          bullet->destroy();
        }
      }
  };


  class B2Controller: public Component{
    public:
      b2World* world;
      Listener listener;
      B2Controller(glm::vec2 gravity={0,0}){
        world = new b2World(b2Vec2(gravity.x, gravity.y));
        listener = Listener();
        world->SetContactListener(&listener);
      }

      void onUpdate(){
        world->Step(deltaTime, 6, 4);
      }
  };

  class B2Collider: public Collider2DApi {
    public:
      b2PolygonShape* shape;
      b2Body* body;

      B2Collider(int mass, bool ci, bool isSensor = false){
        this->mass = mass;
        this->localInertia = ci;
        this->sensor = isSensor;
      }
      

      void setTransform(glm::vec2 pos, float angle){
        body->SetTransform(b2Vec2(pos.x, pos.y), angle);
      }

      void applyForce(glm::vec2 force) {
        body->ApplyForceToCenter(b2Vec2(force.x, force.y), true);
      }

      void setLinearVelocity(glm::vec2 vel){
        body->SetLinearVelocity(b2Vec2(vel.x, vel.y));
      }
      glm::vec2 getLinearVelocityFromWorldPoint(glm::vec2 point){
        auto v = body->GetLinearVelocityFromWorldPoint(b2Vec2(point.x, point.y));
        return {v.x, v.y};
      }
      glm::vec2 getLinearVelocity(){
        auto v = body->GetLinearVelocity();
        return {v.x, v.y};
      }
      
      virtual b2PolygonShape* getShape() = 0;
      void onStart(){
        shape = getShape();
        b2BodyDef bodyDef;
        bodyDef.type = b2_dynamicBody;

        bodyDef.position.Set(transform->position.x, transform->position.y);

	      body = ((B2Controller*)entity->getScene()->getEntityByComponent<PhysicsController2D>()->getComponent<PhysicsController2D>().component)->world->CreateBody(&bodyDef);

        b2FixtureDef bodyFixtureDef;
        bodyFixtureDef.shape = shape;
        bodyFixtureDef.isSensor = this->sensor;
        bodyFixtureDef.density = 1;
        bodyFixtureDef.userData.pointer = (uintptr_t)this->entity;
        //body->sen = this->sensor;
	      body->CreateFixture(&bodyFixtureDef);
      }
      void onUpdate(){
        b2Vec2 pos = body->GetPosition();
        glm::float32 angle = body->GetAngle();

        transform->position.x = pos.x;
        transform->position.y = pos.y;

        //transform->setRotationZ(angle);
      }

      ~B2Collider(){
        body->GetWorld()->DestroyBody(body);
      }

  };

  class B2BoxCollider: public B2Collider{
  public:
      b2PolygonShape* getShape(){b2PolygonShape* x=new b2PolygonShape();x->SetAsBox(transform->scale.x/2.f, transform->scale.y/2.f); return x;}
      B2BoxCollider(int mass, bool c, bool sensor): B2Collider(mass, c, sensor){}
      ~B2BoxCollider(){}
  };

  class B2Creator: public PhysicsCreator2D {
    public:
        Component* newPhysicsController() { return new B2Controller(); }
        Collider2DApi* newPlaneCollider(int mass, bool localInertia, bool sensor) { return nullptr; }
        Collider2DApi*  newMeshCollider(int mass, bool localInertia, bool sensor) { return nullptr; }
        Collider2DApi*   newBoxCollider(int mass, bool localInertia, bool sensor) { return new B2BoxCollider(mass, localInertia, sensor); }
    };

}
