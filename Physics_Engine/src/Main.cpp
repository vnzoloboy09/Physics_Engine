#include "Game.h"

int main(void) {
    Game* game = new Game();
    game->Init();

    float dt = 1.0f / 60.0f;
    while (!WindowShouldClose()) {
        game->Update(dt);
        game->Render(); 
    }

    game->Quit();

    return 0;
}