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

class pendulum {
public:
	pendulum();
	pendulum(double, double, double, double, double, double, int);
	void draw();
int colour;
	int x1=0, y1=0, x2 = 200, y2 = 200;
	double m1, m2, l1, l2, a1, a2, g;

	double a1_p = 0, a2_p = 0, a1_dp = 0, a2_dp = 0;

};

int main(){
	if(!setupGraphics()){
		return 1;
	}


	// goal related 
	pendulum pendulumGoal = pendulum(200, 200, 70, 70, 0.25*PI, 0, 0xBB);
	coords goal = {400, 400};
	goalShape.x = 0;
	goalShape.y = 0;
	goalShape.w = 6;
	goalShape.h = 6;

	// IK method objects
	trainMethod ikA = trainMethod({200, 50}, 75, 20, {255, 10, 150, 255}, gRenderer, windowHeight);
	fabrikMethod ikB = fabrikMethod({200, 50}, 75, 20, {25, 220, 15, 255}, gRenderer, windowHeight);
	ccdMethod ikC = ccdMethod({200, 50}, 75, 20, {255, 255, 0}, gRenderer, windowHeight);

	// kinematic objects
	htmMethod kA = htmMethod({200, 50}, 75, 20, {25, 25, 255}, gRenderer, windowHeight);

	double t;

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
				//goal.x = e.motion.x;
				//goal.y = e.motion.y;
				
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
		pendulumGoal.draw();
		goal.x = pendulumGoal.x2;
		goal.y = pendulumGoal.y2;
		goalShape.x = goal.x-3;
		goalShape.y = goal.y-3;
		SDL_RenderFillRect(gRenderer, &goalShape);


		ikA.calculate({goal.x, windowHeight-goal.y});
		ikA.draw();	
		// this is the beginning of displaying this info on screen
		//printf("IKA: %f, %f, %f\n", ikA.ikInfo.values[0],ikA.ikInfo.values[1],ikA.ikInfo.values[2]);
		ikB.calculate({goal.x, windowHeight-goal.y});
		ikB.draw();	
		ikC.calculate({goal.x, windowHeight-goal.y});
		ikC.draw();	

		t += 0.01;
		kA.calculate(2*t, t, -t*0.7, t+0.4, -t);
		kA.draw();
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
pendulum::pendulum() {
	m1 = 40; m2 = 40;
	l1 = l2 = 150;
	a1 = a2 = PI;
	g = -0.0005;
	colour = 0xFF;
}

pendulum::pendulum(double m1_, double m2_, double l1_, double l2_, double a1_, double a2_, int colour_){
	m1 = m1_;
	m2 = m2_;
	l1 = l1_;
	l2 = l2_;
	a1 = a1_;
	a2 = a2_;
	colour = colour_;
	g = -0.005;
}

void pendulum::draw() {
	SDL_SetRenderDrawColor(gRenderer, colour, colour, colour, 0xFF);
	//SDL_RenderDrawPoint(renderer, 30, 30);
	
	double term1 = -g * (2 * m1 + m2) * sin(a1);
	double term2 = -m2 * g * sin(a1 - 2 * a2);
	double term3 = -2 * sin(a1 - a2) * m2 * (a2_p*a2_p*l2+a1_p*a1_p*l1*cos(a1-a2));
	double term4 = l1 * (2 * m1 + m2 - m2 * cos(2 * a1 - 2 * a2));
	a1_dp = (term1+term2+term3)/term4;

	
	term1 = 2 * sin(a1 - a2);
	term2 = a1_p * a1_p * l1 * (m1 + m2);
	term3 = g * (m1 + m2) * cos(a1) + a2_p * a2_p * l2 * m2 * cos(a1 - a2);
	term4 = l2 * (2 * m1 + m2 - m2 * cos(2 * a1 - 2 * a2));
	a2_dp = (term1 * (term2 + term3)) / term4;
	
	a1_p += a1_dp;
	a2_p += a2_dp;

	a1 += a1_p;
	a2 += a2_p;

	x1 = l1 * sin(a1) + windowWidth / 2;
	y1 = -l1 * cos(a1) + windowHeight / 2;
	x2 = x1 + l2 * sin(a2);
	y2 = y1 -l1 * cos(a2);
	SDL_RenderDrawLine(gRenderer, windowWidth / 2, windowHeight / 2, x1, y1);
	SDL_RenderDrawLine(gRenderer, x1, y1, x2, y2);
}
