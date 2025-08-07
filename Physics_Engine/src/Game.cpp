#include "Game.h"
#include "Def.h"
#include "raymath.h"
#include "FlatConverter.h" 
#include "FlatMath.h"
#include "Collisions.h"
#include "Graphics.h"

#include <iostream>
#include <chrono>

std::vector<Color> colors;
std::vector<Color> outlineColors;

auto sampleTimer = std::chrono::high_resolution_clock::now();

Game::Game() = default;
Game::~Game() = default;

void Game::Init() {
    srand(time(0));
    SetConfigFlags(FLAG_VSYNC_HINT | FLAG_MSAA_4X_HINT);
    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Physics");
    SetTargetFPS(60);
    //SetWindowState(FLAG_WINDOW_RESIZABLE);

    camera.zoom = 24.0f;
    camera.target = { 0.0f, 0.0f };
    camera.offset = { SCREEN_WIDTH / 2.0f, SCREEN_HEIGHT / 2.0f };
    minCam = GetScreenToWorld2D({ 0, 0 }, camera);
    maxCam = GetScreenToWorld2D({ SCREEN_WIDTH, SCREEN_HEIGHT }, camera);
    world = new FlatWorld();

    float paddingY = (maxCam.x - minCam.x) * 0.1f;
    float paddingX = (maxCam.y - minCam.y) * 0.1f;

    auto groundBody = FlatBody::CreateBoxBody(maxCam.x - minCam.x - paddingX * 2, 3.0f, FlatVector(0, 10), 1.0f, true, 0.5f);

    if (!groundBody) {
        std::cerr << "errorMsg";
    }
    world->AddBody(std::make_unique<FlatBody>(std::move(*groundBody)));
    outlineColors.emplace_back(BLUE);
    colors.emplace_back(DARKGRAY);
}

void Game::Update(float dt) { 
    minCam = GetScreenToWorld2D({ 0, 0 }, camera);
    maxCam = GetScreenToWorld2D({ SCREEN_WIDTH, SCREEN_HEIGHT }, camera);
    
    HandleInput();

    auto st = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = st - sampleTimer;

    if (elapsed.count() > 1) {
        bodyCountString = "Body count: " + std::to_string(totalBodyCount / totalSampleCount);
        worldStepTimeString = "Step time: " + std::to_string(std::round(totalWorldTimeStep / totalSampleCount * 10000.0) / 10000.0);

        totalWorldTimeStep = 0;
        totalBodyCount = 0;
        totalSampleCount = 0;
        sampleTimer = std::chrono::high_resolution_clock::now();
    }
    
    world->Step(20, dt);
    auto ed = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = ed - st;
    
    totalWorldTimeStep += duration.count();
    totalBodyCount += world->BodyCount();
    totalSampleCount++;

    for (int i = 0; i < world->BodyCount(); i++) {
        FlatBody* body;
        if (!world->GetBody(i, body)) {
            std::cerr << "falied to get body in game::update!\n";
        }
        FlatAABB box = body->GetAABB();
        if (box.max.y > maxCam.y) {
            world->RemoveBody(body);
            colors.erase(colors.begin() + i);
            outlineColors.erase(outlineColors.begin() + i);
        }
    }
}

void Game::HandleKeyInput() {
#if false
    float dx = 0.0f;
    float dy = 0.0f;
    float forceMagnitude = 48.0f;

    FlatBody* body = nullptr;
    if (!world->GetBody(0, body)) {
        std::cerr << "failed to get body int Game::HandleKeyInput!\n";
    }

    if (IsKeyDown(KEY_A)) dx--;
    if (IsKeyDown(KEY_D)) dx++;
    if (IsKeyDown(KEY_W)) dy--;
    if (IsKeyDown(KEY_S)) dy++;
    if (IsKeyDown(KEY_R)) body->Rotate(PI / 2.0f * GetFrameTime());

    if (dx != 0.0f || dy != 0.0f) {
        FlatVector forceDirection = FlatMath::Normalize(FlatVector(dx, dy));
        FlatVector force = forceDirection * forceMagnitude;
        body->AddForce(force);
    }
    dx = 0.0f;
    dy = 0.0f;
#endif
}

void Game::HandleMouseInput() {
    /*if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        Vector2 delta = GetMouseDelta();
        delta = Vector2Scale(delta, -1.0f / camera.zoom);
        camera.target = Vector2Add(camera.target, delta);
    }*/

    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        float width = 1.0f + rand() % 2;
        float height = 1.0f + rand() % 2;

        auto box = FlatBody::CreateBoxBody(width, height, FlatConverter::ToFlatVector(GetScreenToWorld2D(GetMousePosition(), camera)),
            1.0f, false, 0.5f);
        if (!box) {
            std::cerr << "errorMsg";
        }
        world->AddBody(std::make_unique<FlatBody>(std::move(*box)));
        colors.emplace_back(Graphics::GetRandomColor());
        outlineColors.emplace_back(BLUE);
    }

    if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
        float radius = 0.75f + rand() % 1;
        auto circle = FlatBody::CreateCircleBody(radius, FlatConverter::ToFlatVector(GetScreenToWorld2D(GetMousePosition(), camera)),
            1.0f, false, 0.5f);
        if(!circle) {
            std::cerr << "errorMsg";
        }
        world->AddBody(std::make_unique<FlatBody>(std::move(*circle)));
        colors.emplace_back(Graphics::GetRandomColor());
        outlineColors.emplace_back(BLUE);
    }

    float wheel = GetMouseWheelMove();
    if (wheel != 0) {
        Vector2 mouseWorldPos = GetScreenToWorld2D(GetMousePosition(), camera);

        camera.offset = GetMousePosition();
        camera.target = mouseWorldPos;

        float scale = 0.2f * wheel;
        camera.zoom = Clamp(expf(logf(camera.zoom) + scale), 0.125f, 64.0f);
    }
}

void Game::HandleInput() {
    HandleKeyInput();
    HandleMouseInput();
}

void Game::Redner() {
    BeginDrawing();
    ClearBackground(RAYWHITE);
    BeginMode2D(camera); 

    for (int i = 0; i < world->BodyCount(); i++) {
        FlatBody* body = nullptr;
        if (!world->GetBody(i, body)) {
            std::cerr << "failed to get body in Game::render!\n";
        }
        if (body->shapeType == FlatBody::ShapeType::Box) {
            Graphics::DrawPolygonFull(FlatConverter::ToVector2List(body->GetTransformVertices()),
                body->triangles, colors[i], outlineColors[i]);
        }
        else {
            Graphics::DrawCircleFull(FlatConverter::ToVector2(body->GetPosition()), 
                body->radius, colors[i], outlineColors[i]);
        }
    }

    for (auto& point : world->contactPointsList) {
        DrawCircleV(FlatConverter::ToVector2(point), 0.5f, RED);
    }

    EndMode2D();
    DrawText(worldStepTimeString.c_str(), 20, 20, 20, BLACK);
    DrawText(bodyCountString.c_str(), 20, 40, 20, BLACK);
    EndDrawing();
}

void Game::Quit() {
    CloseWindow();
}

void Game::WrapScreen() {
    float viewWidth = maxCam.x - minCam.x;
    float viewHeight = maxCam.y - minCam.y;

    for (int i = 0; i < world->BodyCount(); i++) {
        FlatBody* body;
        if (!world->GetBody(i, body)) {
            std::cerr << "failed to get body in Game::WrapScreen!\n";
        }
        if (body->GetPosition().x < minCam.x) {
            body->MoveTo(body->GetPosition() + FlatVector{ viewWidth, 0.0f });
        }
        if (body->GetPosition().x > maxCam.x) {
            body->MoveTo(body->GetPosition() - FlatVector{ viewWidth, 0.0f });
        }
        if (body->GetPosition().y < minCam.y) {
            body->MoveTo(body->GetPosition() + FlatVector{ 0.0f, viewHeight});
        }
        if (body->GetPosition().y > maxCam.y) {
            body->MoveTo(body->GetPosition() - FlatVector{ 0.0f, viewHeight});
        }
    }
}