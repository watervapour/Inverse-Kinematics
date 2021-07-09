#include "watervapourMath.h"
#include "ikmethods.h"
#include <cstdio>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>


// SDL settings
const int windowWidth = 600;
const int windowHeight = 600;
SDL_Surface* gWindowSurface = NULL;
SDL_Renderer* gRenderer = NULL;
SDL_Window* gWindow = NULL;
SDL_Rect goalShape;
TTF_Font* gMessageFont;
SDL_Color gMessageFontColour = {200, 200, 255};
SDL_Surface* gMessageSurface = NULL;
SDL_Texture* gMessageTexture = NULL;
// SDL functions
bool setupGraphics();
void close();

int main(){
	if(!setupGraphics()){
		return 1;
	}

	// create stuff
	trainMethod trainA = trainMethod({200, 50}, 100, 20, {255, 10, 150, 255}, gRenderer, windowHeight);
	fabrikMethod trainB = fabrikMethod({200, 50}, 100, 20, {25, 220, 15, 255}, gRenderer, windowHeight);

	coords goal = {400, 400};
	goalShape.x = 0;
	goalShape.y = 0;
	goalShape.w = 10;
	goalShape.h = 10;

	bool running = true;
	SDL_Event e;
	while(running){
		while(SDL_PollEvent(&e) != 0 ){
			if(e.type == SDL_QUIT){
				running = false;
				break;	
			} 
			else if(e.type == SDL_MOUSEMOTION)
			{
				goal.x = e.motion.x;
				goal.y = e.motion.y;
				
			}
			else if(e.type == SDL_KEYDOWN){
				switch(e.key.keysym.sym){
					case(SDLK_q):
						running = false;
						break;
					case(SDLK_SPACE):
						// switch between mouse or bouncing ball
						break;
				} // end keysym switch
			}
		} // end event polling

		SDL_SetRenderDrawColor(gRenderer, 0x20, 0x20, 0x20, 0xFF);
		SDL_RenderClear(gRenderer);

		SDL_SetRenderDrawColor(gRenderer, 200, 200, 200, 255);
		goalShape.x = goal.x;
		goalShape.y = goal.y;
		SDL_RenderFillRect(gRenderer, &goalShape);

		trainA.calculate({goal.x, windowHeight-goal.y});
		trainA.draw();	
		trainB.calculate({goal.x, windowHeight-goal.y});
		trainB.draw();	

		SDL_RenderPresent(gRenderer);
		SDL_Delay(10); // hacky timing
	} // end while(running) loop
	close();
	return 0;
}

bool setupGraphics(){
	bool success = true;

	if(SDL_Init(SDL_INIT_VIDEO) < 0){
		printf("Could not initialise SDL: %S\n", SDL_GetError());
		success = false;
	}
	else {
		gWindow = SDL_CreateWindow("Inverse Kinematics", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, windowWidth, windowHeight, SDL_WINDOW_SHOWN);
		if(gWindow == NULL){
			printf("Could not create window: %S\n", SDL_GetError());
			success = false;
		}
		else {
			gRenderer = SDL_CreateRenderer(gWindow, -1, SDL_RENDERER_ACCELERATED);
			if(gRenderer == NULL){
				printf("Could not create renderer: %S\n", SDL_GetError());	
				success = false;
			}
		}
	}
	return success;
}

void close(){
	SDL_DestroyWindow(gWindow);
	gWindow = NULL;
	SDL_DestroyRenderer(gRenderer);
	gRenderer = NULL;
	SDL_Quit();
}
