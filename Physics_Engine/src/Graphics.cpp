#include "Graphics.h"

void Graphics::DrawPolygonFill(const std::vector<Vector2>& vertices, const std::vector<int>& triangles, const Color color) {
    for (size_t i = 0; i + 2 < triangles.size(); i += 3) {
        Vector2 v0 = vertices[triangles[i]];
        Vector2 v1 = vertices[triangles[i + 1]];
        Vector2 v2 = vertices[triangles[i + 2]];

        DrawTriangle(v2, v1, v0, color);
    }
}

void Graphics::DrawPolygonOutline(const std::vector<Vector2>& vertices, const Color color) {
    for (size_t i = 0; i < vertices.size(); ++i) {
        Vector2 v1 = vertices[i];
        Vector2 v2 = vertices[(i + 1) % vertices.size()]; 
        DrawLineV(v1, v2, color);
    }
}

void Graphics::DrawPolygonFull(const std::vector<Vector2>& vertices, const std::vector<int>& triangles, 
    const Color fillColor, const Color outlineColor) 
{
    DrawPolygonFill(vertices, triangles, fillColor);
    DrawPolygonOutline(vertices, outlineColor);
}

void Graphics::DrawBoxFill(const Vector2& position, const float& width, const float& height, const float& angle, const Color& fillColor) {
    Rectangle rect = { position.x, position.y, width, height }; 
    DrawRectanglePro(rect, { width / 2.0f, height / 2.0f }, angle * 180.0f / PI, fillColor);
}

void Graphics::DrawBoxOutline(const Vector2& position, const float& width, const float& height, const Color& outlineColor) {
    DrawRectangleLinesEx({ position.x - width / 2.0f, position.y - height / 2.0f, width, height }, 0.1f, outlineColor);
}

void Graphics::DrawBoxFull(const Vector2& position, const float& width, const float& height, 
    const float& angle, const Color& fillColor, const Color& outlineColor) {
    DrawBoxFill(position, width, height, angle, fillColor);
    DrawBoxOutline(position, width, height, outlineColor);
}

void Graphics::DrawCircleFill(const Vector2 position, const float radius, const Color color) {
    DrawCircleV(position, radius, color);
}

void Graphics::DrawCircleOutline(const Vector2 position, const float radius, const Color color) {
    DrawCircleLinesV(position, radius, color);
}

void Graphics::DrawCircleFull(const Vector2 position, const float radius, const Color fillColor, const Color outlineColor) {
    DrawCircleFill(position, radius, fillColor);
    DrawCircleOutline(position, radius, outlineColor);
}

Color Graphics::GetRandomColor() {
    return {
        (unsigned char)GetRandomValue(0, 255),  // Red
            (unsigned char)GetRandomValue(0, 255),  // Green
            (unsigned char)GetRandomValue(0, 255),  // Blue
            255                                     // Alpha (fully opaque)
    };
}