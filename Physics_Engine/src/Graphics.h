#pragma once

#include "raylib.h"
#include "FlatVector.h"
#include <vector>

class Graphics {
public:
    static Color GetRandomColor();
    static void DrawPolygonFill(const std::vector<Vector2>& vertices, const std::vector<int>& triangles, 
        const Color color);
    static void DrawPolygonOutline(const std::vector<Vector2>& vertices, const Color color);
    static void DrawPolygonFull(const std::vector<Vector2>& vertices, const std::vector<int>& triangles, 
        const Color fillColor, const Color outlineColor);
    static void DrawCircleFill(const Vector2 position, const float radius, const Color color);
    static void DrawCircleOutline(const Vector2 position, const float radius, const Color color);
    static void DrawCircleFull(const Vector2 position, const float radius, const Color fillColor, const Color outlineColor);
    static void DrawBoxFill(const Vector2& position, const float& width, const float& height, const float& angle, const Color& fillColor);
    static void DrawBoxOutline(const Vector2& position, const float& width, const float& height, const Color& outlineColor);
    static void DrawBoxFull(const Vector2& position, const float& width, const float& height, 
        const float& angle, const Color& fillColor, const Color& outlineColor);
};