#include "Game.h"

int main(void) {
    Game* game = new Game();
    game->Init();

    while (!WindowShouldClose()) {
        game->Update(GetFrameTime());
        game->Redner(); 
    }

    game->Quit();

    return 0;
}