#pragma once

#include <Core/Aliases.h>
#include "Core/Components/Component.h"
#include "box2d/b2_body.h"
#include "box2d/b2_math.h"
#include "box2d/b2_polygon_shape.h"
#include "box2d/b2_world.h"
#include "glm/fwd.hpp"
#include <Static/Window/Window.h>
#include <Core/Physics/ColliderApi.h>
#include <Core/Physics/PhysicsController.h>
#include <box2d/box2d.h>
#include <Core/Components/Transform.h>
#include <Core/Scene.h>
#include <Core/Components/Entity.h>

namespace PetrolEngine{
  class B2Controller: public Component{
    public:
      b2World* world;
      B2Controller(glm::vec2 gravity={0,0}){
        world = new b2World(b2Vec2(gravity.x, gravity.y));
      }

      void onUpdate(){
        world->Step(1.f/50.f, 6, 2);
      }
  };

  class B2Collider: public Component{
    public:
      b2PolygonShape* shape;
      b2Body* body;
      int mass;

      B2Collider(int mass, bool ci){
        this->mass = mass;
      }

      virtual b2PolygonShape* getShape() = 0;
      void onStart(){
        shape = getShape();
        b2BodyDef bodyDef;
        bodyDef.type = b2_dynamicBody;
        bodyDef.position.Set(transform->position.x, transform->position.y);

	      body = ((B2Controller*)entity->getScene()->getEntityByComponent<PhysicsController>()->getComponent<PhysicsController>().component)->world->CreateBody(&bodyDef);

	      body->CreateFixture(shape, 1);
      }
      void onUpdate(){
        b2Vec2 pos = body->GetPosition();
        glm::float32 angle = body->GetAngle();
        
        transform->position.x = pos.x;
        transform->position.y = pos.y;

        transform->setRotationZ(angle);
      }

  };

  class B2BoxCollider: public B2Collider{
  public:
      b2PolygonShape* getShape(){b2PolygonShape* x=new b2PolygonShape();x->SetAsBox(transform->scale.x, transform->scale.y); return x;}
      B2BoxCollider(int mass, bool c): B2Collider(mass, c){}
  };

  class B2Creator: public PhysicsCreator {
    public:
        Component* newPhysicsController() { return new B2Controller(); }
        Component* newPlaneCollider(int mass, bool localInertia, glm::vec3 inertia) { return nullptr; }
        Component*  newMeshCollider(int mass, bool localInertia, glm::vec3 inertia) { return nullptr; }
        Component*   newBoxCollider(int mass, bool localInertia, glm::vec3 inertia) { return new B2BoxCollider(mass, localInertia); }
    };

}
