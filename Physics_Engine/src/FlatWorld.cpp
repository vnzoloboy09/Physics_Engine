#include "FlatWorld.h"
#include "Collisions.h"
#include "FlatMath.h"
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

bool FlatWorld::GetBody(const int& id, FlatBody*& body) {
    if (id < 0 || id >= bodyList.size()) {
        body = nullptr;
        return false;
    }

    body = bodyList[id].get(); 
    return true;
}

size_t FlatWorld::BodyCount() const {
    return bodyList.size();
}

void FlatWorld::Step(int iterations, float dt) { 
    iterations = FlatMath::Clamp(iterations, MIN_ITERATIONS, MAX_ITERATIONS);

    contactPointsList.clear();

    for (int it = 0; it < iterations; it++) {
        // Moverment
        for (auto &body : bodyList) {
            body->Step(gravity, iterations, dt);
        }

        contactList.clear();

        // Collision
        if (BodyCount() > 1) { // only do collisions when there more than one body
            for (int i = 0; i < bodyList.size() - 1; i++) {
                FlatBody* bodyA = bodyList[i].get();
                FlatAABB bodyA_aabb = bodyA->GetAABB();

                for (int j = i + 1; j < bodyList.size(); j++) {
                    FlatBody* bodyB = bodyList[j].get();
                    FlatAABB bodyB_aabb = bodyB->GetAABB();

                    if (bodyA->b_IsStatic && bodyB->b_IsStatic) {
                        continue;
                    }

                    if (!Collisions::IntersectAABB(bodyB_aabb, bodyA_aabb)) {
                        continue;
                    }

                    FlatVector normal;
                    float depth;

                    if (Collisions::Collide(bodyA, bodyB, normal, depth)) {
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

                        FlatVector contact1, contact2;
                        int contactCount;

                        Collisions::FindContactPoints(bodyA, bodyB, contact1, contact2, contactCount);
                        contactList.push_back(std::make_unique<FlatManifold>(
                            bodyA, bodyB, normal, depth, contact1, contact2, contactCount
                        ));
                    }
                }
            }

            for (auto& contact : contactList) {
                ResolveCollision(contact);

                if (contact->contactCount > 0) {
                    if (!(std::find(contactPointsList.begin(), contactPointsList.end(), contact->contact1) != contactPointsList.end())) {
                        contactPointsList.push_back(contact->contact1);
                    }

                    if (contact->contactCount > 1) {
                        if (!(std::find(contactPointsList.begin(), contactPointsList.end(), contact->contact2) != contactPointsList.end())) {
                            contactPointsList.push_back(contact->contact2);
                        }
                    }
                }
            }
        } 
    }
}

void FlatWorld::ResolveCollision(const std::unique_ptr<FlatManifold>& contact) {
    float restitution = std::min(contact->bodyA->restitution, contact->bodyB->restitution);
    FlatVector relativeVelocity = contact->bodyB->GetLinearVelocity() - contact->bodyA->GetLinearVelocity();

    if (FlatMath::Dot(relativeVelocity, contact->normal) > 0.0f) {
        return;
    }

    float magnitude = -(1.0f + restitution) * FlatMath::Dot(relativeVelocity, contact->normal);
    magnitude /= contact->bodyA->invMass + contact->bodyB->invMass;

    FlatVector impulse = magnitude * contact->normal;

    contact->bodyA->linearVelovity -= impulse * contact->bodyA->invMass;
    contact->bodyB->linearVelovity += impulse * contact->bodyB->invMass;
}