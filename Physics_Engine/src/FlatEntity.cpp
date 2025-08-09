#include "FlatEntity.h"
#include "Graphics.h"
#include "FlatConverter.h"

FlatEntity::FlatEntity(FlatBody*& _body) :
    body(_body),
    color(Graphics::GetRandomColor())
{}

FlatEntity::FlatEntity(FlatBody*& _body, const Color& _color) :
    body(_body),
    color(_color)
{}

FlatEntity::FlatEntity(FlatWorld*& world, const float& radius, const bool& isStatic, const FlatVector& position) {
    FlatBody::CreateCircleBody(radius, 1.0f, isStatic, 0.5f, body);
    if (!body) {
        __debugbreak();
    }
    body->MoveTo(position);
    color = Graphics::GetRandomColor();
    world->AddBody(body);
}

FlatEntity::FlatEntity(FlatWorld*& world, const float& width, const float& height, const bool& isStatic, const FlatVector& position) {
    FlatBody::CreateBoxBody(width, height, 1.0f, isStatic, 0.5f, body);
    if (!body) {
        __debugbreak(); 
    }
    body->MoveTo(position);
    color = Graphics::GetRandomColor();
    world->AddBody(body);
}


void FlatEntity::Render(FlatBody::ShapeType shape) {
    Vector2 pos = FlatConverter::ToVector2(body->GetPosition());
    
    if (body->shapeType == FlatBody::Box) {
        Graphics::DrawBoxFill(pos, body->width, body->height, body->angle, color);
        Graphics::DrawPolygonOutline(FlatConverter::ToVector2List(body->GetTransformVertices()), BLUE);
    }
    else if(body->shapeType == FlatBody::Circle) {
        Graphics::DrawCircleFull(pos, body->radius, color, BLUE);
    }
}