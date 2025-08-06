#include "FlatWorld.h"
#include "Collisions.h"
#include "FlatMath.h"
#include <iostream>
#include <algorithm>

const float FlatWorld::MIN_BODY_SIZE = 0.01f * 0.01f;
const float FlatWorld::MAX_BODY_SIZE = 64.0f * 64.0f;

const float FlatWorld::MIN_DENSITY = 0.5f;
const float FlatWorld::MAX_DENSITY = 21.4f;

FlatWorld::FlatWorld() {
	gravity = { 0.0f, 9.81f };
}

void FlatWorld::AddBody(std::unique_ptr<FlatBody> body) {
    bodyList.push_back(std::move(body));
}

void FlatWorld::RemoveBody(const FlatBody* body) {
    bodyList.erase(std::remove_if(bodyList.begin(), bodyList.end(),
        [&](const std::unique_ptr<FlatBody>& ptr) { return ptr.get() == body; }),
        bodyList.end()
    );
}

bool FlatWorld::GetBody(int id, FlatBody*& body) {
    if (id < 0 || id >= bodyList.size()) {
        body = nullptr;
        return false;
    }

    body = bodyList[id].get(); 
    return true;
}

int FlatWorld::BodyCount() const {
    return bodyList.size();
}

void FlatWorld::Step(int iterations, float dt) { 
    iterations = FlatMath::Clamp(iterations, MIN_ITERATIONS, MAX_ITERATIONS);

    for (int it = 0; it < iterations; it++) {
        // Moverment
        for (int i = 0; i < bodyList.size(); i++) {
            bodyList[i]->Step(gravity, iterations, dt);
        }

        // Collision
        if(BodyCount() > 1) // only do collisions when there more than one body
        for (int i = 0; i < bodyList.size() - 1; i++) {
            FlatBody* bodyA = bodyList[i].get();
            
            for (int j = i + 1; j < bodyList.size(); j++) {
                FlatBody* bodyB = bodyList[j].get();

                if (bodyA->b_IsStatic && bodyB->b_IsStatic) {
                    continue;
                }

                FlatVector normal;
                float depth;

                if (Collide(bodyA, bodyB, normal, depth)) {
                    if (bodyA->b_IsStatic) {
                        bodyB->Move(normal * depth);
                    }
                    else if (bodyB->b_IsStatic) {
                        bodyA->Move(-normal * depth);
                    }
                    else {
                        bodyA->Move(-normal * depth * 0.5f);
                        bodyB->Move(normal * depth * 0.5f);
                    }
                    ResolveCollision(bodyA, bodyB, normal, depth);
                }
            }
        }
    }
}

void FlatWorld::ResolveCollision(FlatBody*& bodyA, FlatBody*& bodyB, FlatVector& normal, float& depth) {
    float restitution = std::min(bodyA->restitution, bodyB->restitution);
    FlatVector relativeVelocity = bodyB->GetLinearVelocity() - bodyA->GetLinearVelocity();

    if (FlatMath::Dot(relativeVelocity, normal) > 0.0f) {
        return;
    }

    float magnitude = -(1.0f + restitution) * FlatMath::Dot(relativeVelocity, normal);
    magnitude /= bodyA->invMass + bodyB->invMass;

    FlatVector impulse = magnitude * normal;

    FlatVector linearVelocityA = bodyA->GetLinearVelocity() - impulse * bodyA->invMass ;
    FlatVector linearVelocityB = bodyB->GetLinearVelocity() + impulse * bodyB->invMass;
    bodyA->SetLinearVelocity(linearVelocityA);
    bodyB->SetLinearVelocity(linearVelocityB);
}

bool FlatWorld::Collide(FlatBody*& bodyA, FlatBody*& bodyB, FlatVector& normal, float& depth) {
    normal = FlatVector();
    depth = 0.0f;

    FlatBody::ShapeType shapeTypeA = bodyA->shapeType;
    FlatBody::ShapeType shapeTypeB = bodyB->shapeType;

    if (shapeTypeA == FlatBody::ShapeType::Box) {
        if (shapeTypeB == FlatBody::ShapeType::Box) {
            return Collisions::IntersectPolygons(bodyA->GetPosition(), bodyA->GetTransformVertices(), 
                bodyB->GetPosition(), bodyB->GetTransformVertices(), normal, depth);
        }
        else if (shapeTypeB == FlatBody::ShapeType::Circle) {
            bool result = Collisions::IntersectCirclePolygon(bodyB->GetPosition(), bodyB->radius,
                bodyA->GetPosition(), bodyA->GetTransformVertices(), normal, depth);

            normal = -normal;
            return result;
        }
    }
    else if (shapeTypeA == FlatBody::ShapeType::Circle) {
        if (shapeTypeB == FlatBody::ShapeType::Box) {
            return Collisions::IntersectCirclePolygon(bodyA->GetPosition(), bodyA->radius,
                bodyB->GetPosition(), bodyB->GetTransformVertices(), normal, depth);
        }
        else if (shapeTypeB == FlatBody::ShapeType::Circle) {
            return Collisions::IntersectCircles(bodyA->GetPosition(), bodyA->radius,
                bodyB->GetPosition(), bodyB->radius, normal, depth);
        }
    }

    return false;
}
