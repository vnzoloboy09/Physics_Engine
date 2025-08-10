#include "Game.h"
#include "Def.h"
#include "raymath.h"
#include "FlatConverter.h" 
#include "FlatMath.h"
#include "Collisions.h"
#include "Graphics.h"
#include "Random.h"

#include <chrono>
//#include <iostream>

auto sampleTimer = std::chrono::high_resolution_clock::now();

Game::Game() = default;
Game::~Game() {
    for (auto& e : entities) {
        delete e;
    }
    entities.clear();

    for (auto& e : removalEntities) {
        delete e;
    }
    removalEntities.clear();

    delete world;
}

void Game::Init() {
    srand(time(0));
    SetConfigFlags(FLAG_VSYNC_HINT | FLAG_MSAA_4X_HINT);
    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Physics");
    SetTargetFPS(60);
    //SetWindowState(FLAG_WINDOW_RESIZABLE);

    camera.zoom = 20.0f;
    camera.target = { 0.0f, 0.0f };
    camera.offset = { SCREEN_WIDTH / 2.0f, SCREEN_HEIGHT / 2.0f };
    minCam = GetScreenToWorld2D({ 0, 0 }, camera);
    maxCam = GetScreenToWorld2D({ SCREEN_WIDTH, SCREEN_HEIGHT }, camera);
    world = new FlatWorld();

    float paddingY = (maxCam.x - minCam.x) * 0.1f;
    float paddingX = (maxCam.y - minCam.y) * 0.1f;

    FlatBody* groundBody = nullptr;
    FlatBody::CreateBoxBody(maxCam.x - minCam.x - paddingX * 2, 3.0f, 1.0f, true, 0.5f, groundBody);
    if (!groundBody) {
        __debugbreak();
    }
    groundBody->MoveTo({0, 10});
    world->AddBody(groundBody);
    entities.emplace_back(new FlatEntity(groundBody, DARKGRAY));

    FlatBody* ledgeBody1 = nullptr;
    FlatBody::CreateBoxBody(20.0f, 2.0f, 0.5f, true, 0.5f, ledgeBody1);
    if (!ledgeBody1) {
        __debugbreak();
    }
    ledgeBody1->MoveTo({ -13, -3 });
    ledgeBody1->Rotate(2 * PI / 20.0f);
    world->AddBody(ledgeBody1);
    entities.emplace_back(new FlatEntity(ledgeBody1, DARKGREEN));

    FlatBody* ledgeBody2 = nullptr;
    FlatBody::CreateBoxBody(15.0f, 2.0f, 0.5f, true, 0.5f, ledgeBody2);
    if (!ledgeBody2) {
        __debugbreak();
    }
    ledgeBody2->MoveTo({ 10, -10 });
    ledgeBody2->Rotate(-2 * PI / 20.0f);
    world->AddBody(ledgeBody2);
    entities.emplace_back(new FlatEntity(ledgeBody2, DARKBROWN));
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

    removalEntities.clear();

    for (auto& e : entities) {
        if (e->body->b_IsStatic) continue;

        FlatAABB box = e->body->GetAABB();
        if (box.max.x < minCam.x || box.min.x > maxCam.x ||
            box.max.y < minCam.y || box.min.y > maxCam.y) {
            removalEntities.push_back(e);
        }
    }

    for (auto& e : removalEntities) {
        world->RemoveBody(e->body);
        entities.erase(std::remove(entities.begin(), entities.end(), e), entities.end());
    }
}

void Game::HandleKeyInput() {

}

void Game::HandleMouseInput() {
    if (IsMouseButtonDown(MOUSE_BUTTON_MIDDLE)) {
        Vector2 delta = GetMouseDelta();
        delta = Vector2Scale(delta, -1.0f / camera.zoom);
        camera.target = Vector2Add(camera.target, delta);
    }

    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        float width = Random::Float(2.0f, 3.5f);
        float height = Random::Float(2.0f, 3.5f);

        entities.emplace_back(new FlatEntity(world, width, height, false, 
            FlatConverter::ToFlatVector(GetScreenToWorld2D(GetMousePosition(), camera))));
    }

    if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
        float radius = Random::Float(0.5f, 2.0f);

        entities.emplace_back(new FlatEntity(world, radius, false, 
            FlatConverter::ToFlatVector(GetScreenToWorld2D(GetMousePosition(), camera))));
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

void Game::Render() {
    BeginDrawing();
    ClearBackground(RAYWHITE);
    BeginMode2D(camera); 

    for (auto& e : entities) {
        e->Render(e->body->shapeType);
    }

    EndMode2D();
    DrawText(worldStepTimeString.c_str(), 20, 20, 20, BLACK);
    DrawText(bodyCountString.c_str(), 20, 40, 20, BLACK);
    EndDrawing();
}

void Game::Quit() {
    CloseWindow();
}
