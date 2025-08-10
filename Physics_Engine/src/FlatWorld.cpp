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

FlatWorld::~FlatWorld() {
    for (auto& body : bodyList) {
        delete body;
    }
    bodyList.clear();
    contactPair.clear();
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
        FlatBody*& bodyA = bodyList[get<0>(pair)];
        FlatBody*& bodyB = bodyList[get<1>(pair)];

        if (Collisions::Collide(bodyA, bodyB, normal, depth)) {
            SeparateBodies(bodyA, bodyB, normal * depth);

            FlatVector contact1, contact2;
            int contactCount;

            Collisions::FindContactPoints(bodyA, bodyB, contact1, contact2, contactCount);
            FlatManifold contact(bodyA, bodyB, normal, depth, contact1, contact2, contactCount);
            ResolveCollisionWithRotationAndFriction(contact);
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

void FlatWorld::ResolveCollisionBasic(FlatManifold& contact) {
    float restitution = std::min(contact.bodyA->restitution, contact.bodyB->restitution);
    FlatVector relativeVelocity = contact.bodyB->GetLinearVelocity() - contact.bodyA->GetLinearVelocity();

    if (FlatMath::Dot(relativeVelocity, contact.normal) > 0.0f) {
        return;
    }

    float magnitude = -(1.0f + restitution) * FlatMath::Dot(relativeVelocity, contact.normal);
    magnitude /= contact.bodyA->invMass + contact.bodyB->invMass;

    FlatVector impulse = magnitude * contact.normal;

    contact.bodyA->linearVelocity -= impulse * contact.bodyA->invMass;
    contact.bodyB->linearVelocity += impulse * contact.bodyB->invMass;
}

void FlatWorld::ResolveCollisionWithRotation(FlatManifold& contact) {
    float restitution = std::min(contact.bodyA->restitution, contact.bodyB->restitution);

    FlatVector contactList[2] = {contact.contact1, contact.contact2};
    FlatVector impulseList[2];
    FlatVector raList[2];
    FlatVector rbList[2];

    for (int i = 0; i < contact.contactCount; i++) {
        FlatVector ra = contactList[i] - contact.bodyA->GetPosition();
        FlatVector rb = contactList[i] - contact.bodyB->GetPosition();

        raList[i] = ra;
        rbList[i] = rb;

        FlatVector raPerp(-ra.y, ra.x);
        FlatVector rbPerp(-rb.y, rb.x);

        FlatVector angularLinearVelocityA = raPerp * contact.bodyA->GetAngularVelocity();
        FlatVector angularLinearVelocityB = rbPerp * contact.bodyB->GetAngularVelocity();

        FlatVector relativeVelocity = 
            (contact.bodyB->GetLinearVelocity() + angularLinearVelocityB) - 
            (contact.bodyA->GetLinearVelocity() + angularLinearVelocityA);

        float contactVelocityMagnitue = FlatMath::Dot(relativeVelocity, contact.normal);

        if (contactVelocityMagnitue > 0.0f) {
            continue;
        }

        float raPerpDotNormal = FlatMath::Dot(raPerp, contact.normal);
        float rbPerpDotNormal = FlatMath::Dot(rbPerp, contact.normal);

        float denominator = contact.bodyA->invMass + contact.bodyB->invMass + 
            (raPerpDotNormal * raPerpDotNormal) * contact.bodyA->invInertia +
            (rbPerpDotNormal * rbPerpDotNormal) * contact.bodyB->invInertia;

        float magnitude = -(1.0f + restitution) * contactVelocityMagnitue;
        magnitude /= denominator * (float)contact.contactCount;

        FlatVector impulse = magnitude * contact.normal;
        impulseList[i] = impulse;
    }

    for (int i = 0; i < contact.contactCount; i++) {
        FlatVector impulse = impulseList[i];

        contact.bodyA->linearVelocity += -impulse * contact.bodyA->invMass;
        contact.bodyA->angularVelocity += -FlatMath::Cross(raList[i], impulse) * contact.bodyA->invInertia;
        
        contact.bodyB->linearVelocity += impulse * contact.bodyB->invMass;
        contact.bodyB->angularVelocity += FlatMath::Cross(rbList[i], impulse) * contact.bodyB->invInertia;

    }
}

void FlatWorld::ResolveCollisionWithRotationAndFriction(FlatManifold& contact) {
    float restitution = std::min(contact.bodyA->restitution, contact.bodyB->restitution);

    float staticFriction = (contact.bodyA->staticFriction + contact.bodyA->staticFriction) / 2.0f;
    float dynamicFriction = (contact.bodyA->dynamicFriction + contact.bodyA->dynamicFriction) / 2.0f;

    FlatVector contactList[2] = { contact.contact1, contact.contact2 };
    FlatVector impulseList[2] = { { 0.0f, 0.0f }, { 0.0f, 0.0f } };
    FlatVector frictionImpulseList[2] = {{ 0.0f, 0.0f }, { 0.0f, 0.0f }};
    FlatVector raList[2] = {{ 0.0f, 0.0f }, { 0.0f, 0.0f }};
    FlatVector rbList[2] = {{ 0.0f, 0.0f }, { 0.0f, 0.0f }};
    float magnitudeList[2] = { 0.0f, 0.0f };

    for (int i = 0; i < contact.contactCount; i++) {
        FlatVector ra = contactList[i] - contact.bodyA->GetPosition();
        FlatVector rb = contactList[i] - contact.bodyB->GetPosition();

        raList[i] = ra;
        rbList[i] = rb;

        FlatVector raPerp(-ra.y, ra.x);
        FlatVector rbPerp(-rb.y, rb.x);

        FlatVector angularLinearVelocityA = raPerp * contact.bodyA->GetAngularVelocity();
        FlatVector angularLinearVelocityB = rbPerp * contact.bodyB->GetAngularVelocity();

        FlatVector relativeVelocity =
            (contact.bodyB->GetLinearVelocity() + angularLinearVelocityB) -
            (contact.bodyA->GetLinearVelocity() + angularLinearVelocityA);

        float contactVelocityMagnitue = FlatMath::Dot(relativeVelocity, contact.normal);

        if (contactVelocityMagnitue > 0.0f) {
            continue;
        }

        float raPerpDotNormal = FlatMath::Dot(raPerp, contact.normal);
        float rbPerpDotNormal = FlatMath::Dot(rbPerp, contact.normal);

        float denominator = contact.bodyA->invMass + contact.bodyB->invMass +
            (raPerpDotNormal * raPerpDotNormal) * contact.bodyA->invInertia +
            (rbPerpDotNormal * rbPerpDotNormal) * contact.bodyB->invInertia;

        float magnitude = -(1.0f + restitution) * contactVelocityMagnitue;
        magnitude /= denominator * (float)contact.contactCount;

        magnitudeList[i] = magnitude;

        FlatVector impulse = magnitude * contact.normal;
        impulseList[i] = impulse;
    }

    for (int i = 0; i < contact.contactCount; i++) {
        FlatVector impulse = impulseList[i];

        contact.bodyA->linearVelocity += -impulse * contact.bodyA->invMass;
        contact.bodyA->angularVelocity += -FlatMath::Cross(raList[i], impulse) * contact.bodyA->invInertia;

        contact.bodyB->linearVelocity += impulse * contact.bodyB->invMass;
        contact.bodyB->angularVelocity += FlatMath::Cross(rbList[i], impulse) * contact.bodyB->invInertia;
    }

    // Friction
    for (int i = 0; i < contact.contactCount; i++) {
        FlatVector ra = contactList[i] - contact.bodyA->GetPosition();
        FlatVector rb = contactList[i] - contact.bodyB->GetPosition();

        raList[i] = ra;
        rbList[i] = rb;

        FlatVector raPerp(-ra.y, ra.x);
        FlatVector rbPerp(-rb.y, rb.x);

        FlatVector angularLinearVelocityA = raPerp * contact.bodyA->GetAngularVelocity();
        FlatVector angularLinearVelocityB = rbPerp * contact.bodyB->GetAngularVelocity();

        FlatVector relativeVelocity =
            (contact.bodyB->GetLinearVelocity() + angularLinearVelocityB) -
            (contact.bodyA->GetLinearVelocity() + angularLinearVelocityA);

        FlatVector tangent = relativeVelocity - FlatMath::Dot(relativeVelocity, contact.normal) * contact.normal;

        if (FlatMath::NearlyEqual(tangent, { 0.0f, 0.0f })) continue;
        tangent = FlatMath::Normalize(tangent);

        float raPerpDotTangent = FlatMath::Dot(raPerp, tangent);
        float rbPerpDotTangent = FlatMath::Dot(rbPerp, tangent);

        float denominator = contact.bodyA->invMass + contact.bodyB->invMass +
            (raPerpDotTangent * raPerpDotTangent) * contact.bodyA->invInertia +
            (rbPerpDotTangent * rbPerpDotTangent) * contact.bodyB->invInertia;

        float tangentMagnitude = -FlatMath::Dot(relativeVelocity, tangent);
        tangentMagnitude /= denominator * (float)contact.contactCount;
        
        FlatVector frictionImpulse;
        float magnitude = magnitudeList[i];

        if (abs(tangentMagnitude) <= magnitude * staticFriction) {
            frictionImpulse = tangentMagnitude * tangent;
        }
        else {
            frictionImpulse = -magnitude * tangent * dynamicFriction;
        }

        frictionImpulseList[i] = frictionImpulse;
    }

    for (int i = 0; i < contact.contactCount; i++) {
        FlatVector frictionImpulse = frictionImpulseList[i];

        contact.bodyA->linearVelocity += -frictionImpulse * contact.bodyA->invMass;
        contact.bodyA->angularVelocity += -FlatMath::Cross(raList[i], frictionImpulse) * contact.bodyA->invInertia;

        contact.bodyB->linearVelocity += frictionImpulse * contact.bodyB->invMass;
        contact.bodyB->angularVelocity += FlatMath::Cross(rbList[i], frictionImpulse) * contact.bodyB->invInertia;
    }
}