#include "FlatWorld.h"
#include "Collisions.h"
#include "FlatMath.h"

const float FlatWorld::MIN_BODY_SIZE = 0.01f * 0.01f;
const float FlatWorld::MAX_BODY_SIZE = 64.0f * 64.0f;

const float FlatWorld::MIN_DENSITY = 0.5f;
const float FlatWorld::MAX_DENSITY = 21.4f;

FlatWorld::FlatWorld() {
	gravity = { 0.0f, 9.81f };
}

void FlatWorld::AddBody(FlatBody*& body) {
    bodyList.push_back(std::move(body));
}

void FlatWorld::RemoveBody(FlatBody*& body) {
    bodyList.erase(
        std::remove(bodyList.begin(), bodyList.end(), body), bodyList.end()
    );

}

bool FlatWorld::GetBody(const int& id, FlatBody*& body) {
    if (id < 0 || id >= bodyList.size()) {
        body = nullptr;
        return false;
    }

    body = bodyList[id]; 
    return true;
}

size_t FlatWorld::BodyCount() const {
    return bodyList.size();
}

void FlatWorld::Step(int totalIterations, float dt) { 
    totalIterations = FlatMath::Clamp(totalIterations, MIN_ITERATIONS, MAX_ITERATIONS);

    for (int currentItertation = 0; currentItertation  < totalIterations; currentItertation++) {
        contactPair.clear();

        StepBodies(totalIterations, dt);

        if (BodyCount() > 1) { // only do collisions when there more than one body
            BroadPhase();
            NarrowPhase();
        }
    }
}

void FlatWorld::StepBodies(const int& totalIterations, const float& dt) {
    for (auto& body : bodyList) {
        body->Step(gravity, totalIterations, dt);
    }
}

void FlatWorld::BroadPhase() {
    for (int i = 0; i < bodyList.size() - 1; i++) {
        FlatBody*& bodyA = bodyList[i];
        FlatAABB bodyA_aabb = bodyA->GetAABB();

        for (int j = i + 1; j < bodyList.size(); j++) {
            FlatBody*& bodyB = bodyList[j];
            FlatAABB bodyB_aabb = bodyB->GetAABB();

            if (bodyA->b_IsStatic && bodyB->b_IsStatic) {
                continue;
            }

            if (!Collisions::IntersectAABB(bodyB_aabb, bodyA_aabb)) {
                continue;
            }

            contactPair.emplace_back(i, j);
        }
    }
}

void FlatWorld::NarrowPhase() {
    for (auto& pair : contactPair) {
        FlatVector normal;
        float depth;
        FlatBody* bodyA = bodyList[get<0>(pair)];
        FlatBody* bodyB = bodyList[get<1>(pair)];

        if (Collisions::Collide(bodyA, bodyB, normal, depth)) {
            SeparateBodies(bodyA, bodyB, normal * depth);

            FlatVector contact1, contact2;
            int contactCount;

            Collisions::FindContactPoints(bodyA, bodyB, contact1, contact2, contactCount);
            FlatManifold contact(bodyA, bodyB, normal, depth, contact1, contact2, contactCount);
            ResolveCollision(contact);
        }

    }
}

void FlatWorld::SeparateBodies(FlatBody*& bodyA, FlatBody*& bodyB, const FlatVector& mtv) {
    if (bodyA->b_IsStatic) {
        bodyB->Move(mtv);
    }
    else if (bodyB->b_IsStatic) {
        bodyA->Move(-mtv);
    }
    else {
        bodyA->Move(-mtv * 0.5f);
        bodyB->Move(mtv * 0.5f);
    }
}

void FlatWorld::ResolveCollision(FlatManifold& contact) {
    float restitution = std::min(contact.bodyA->restitution, contact.bodyB->restitution);
    FlatVector relativeVelocity = contact.bodyB->GetLinearVelocity() - contact.bodyA->GetLinearVelocity();

    if (FlatMath::Dot(relativeVelocity, contact.normal) > 0.0f) {
        return;
    }

    float magnitude = -(1.0f + restitution) * FlatMath::Dot(relativeVelocity, contact.normal);
    magnitude /= contact.bodyA->invMass + contact.bodyB->invMass;

    FlatVector impulse = magnitude * contact.normal;

    contact.bodyA->linearVelovity -= impulse * contact.bodyA->invMass;
    contact.bodyB->linearVelovity += impulse * contact.bodyB->invMass;
}