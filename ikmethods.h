#include <SDL2/SDL.h>

class ikLimb{
public:

	vector2d self;

	ikLimb();
	ikLimb(coords, double, double, double, SDL_Renderer*, SDL_Colour);
	void draw();
private:
	double limbWidth;
	int winHeight;	
	SDL_Renderer* renderer;
	SDL_Colour limbColour;	
	SDL_Point bl, br, tl, tr;
	SDL_Point lines[5];
};

ikLimb::ikLimb(){
	self.base = {0,0};
	self.angle = 0;
	self.magnitude = 1;
	renderer = nullptr;
	limbColour = {255, 255, 255, 255};
	limbWidth = 5;
	winHeight = 50;
}

ikLimb::ikLimb(coords pos, double angle, double length, double width,
		SDL_Renderer* _renderer, SDL_Colour _limbColour){
	self.base = pos;
	self.angle = angle;
	self.magnitude = length;
	limbWidth = width;

	renderer = _renderer;
	limbColour = _limbColour;
	int unused;
	SDL_GetRendererOutputSize(renderer, &unused, &winHeight);

}

void ikLimb::draw(){
	/* diagram of what each side of the limb relates to
		corners are indicated with the {bl, br, tl, tr} notation
			bl		'left' side		tl
				_____________________
				|					|
	base/bottom	|					| head/top
				|___________________|	
			br		'right' side	tr
	*/	
	
	// perpAngle is the angle the top and bottom edges travel in
	double perpAngle = self.angle + PI/2;
	// halfWidth is the distance from the centre of the bottom face to the left 
	// face or any equivalent, this is needed as the self vector represents the 
	// ideal point, down the middle from base to head
	double halfWidth = limbWidth/2;

	// represent the amound that the corners will stray from the central vector
	double limbDX = cos(perpAngle) * halfWidth;
	double limbDY = sin(perpAngle) * halfWidth;
	
	bl.x = (int)(self.base.x + limbDX);
	bl.y = winHeight - (int)(self.base.y + limbDY);

	br.x = (int)(self.base.x - limbDX);
	br.y = winHeight - (int)(self.base.y - limbDY);

	coords head = self.getHead();
	tl.x = (int)(head.x + limbDX);
	tl.y = winHeight - (int)(head.y + limbDY);

	tr.x = (int)(head.x - limbDX);
	tr.y = winHeight - (int)(head.y - limbDY);
	
	/*
	printf("\n ====== \n");
	printf("deltas n such:\n angle = %f\nperpAngle = %f\n halfWidth = %f\n DX/DY = %f, %f",
		self.angle, perpAngle, halfWidth, limbDX, limbDY);
	printf("base xy = %f, %f\n bl = %d, %d\n br = %d, %d\n",
		self.base.x, self.base.y, bl.x, bl.y, br.x, br.y);
	printf("head xy = %f, %f\n tl = %d, %d\n tr = %d, %d\n",
		head.x, head.y, tl.x, tl.y, tr.x, tr.y);
	*/
	
	SDL_SetRenderDrawColor(renderer, limbColour.r, limbColour.g, limbColour.b, limbColour.a);
	lines[0]=bl;lines[1]=tl;lines[2]=tr;lines[3]=br;lines[4]=bl;
	SDL_RenderDrawLines(renderer, lines, 5);
}

/*
	This is a base class for the various systems.

	Public:
		- constructor: class constructor
		- calculate(): receives a pair of coordinates, used to calculate angles
			for the segments of the system
		- draw(): calls the draw function for each limb segment
	Private:
		- baseCoords: coordinates for the base of the system (where it is mounted
			to the outer world)
		- segments[]: an array of ikLimbs, represents the system
*/
class baseIKSystem{
public:
	baseIKSystem() = default;
	virtual void calculate(coords);
	virtual void draw();
protected:
	coords baseCoords;
	const int segmentCount = 3;
	ikLimb segments[3];
	
};

void baseIKSystem::draw(){
	for(int N = 0; N < segmentCount; N++){
		segments[N].draw();
	}
}

void baseIKSystem::calculate(coords){
	return;
}
// =============================================================
/*
	Below are the various alrogithm/methods of solving inverse kinematics.
	Currently, these only handle segments that can roate around the Z axis (in and
	out of the screen).

	With a system of N segments, any numbering will dicate the base/root element
	as segment 0, with the end affecter/tip segment being labeled N-1.
	
	For the time being, each system will use a fixed array for the segments,
	as a future goal, perhaps vectors, or other dynamic storage classes may be
	used. Also may have colours be gradients along segment chain.
*/



/* 
	This algorithm for invsere kinematics is described in a video from
	the youtube channel 'the coding train'.

	The method treats each limb as a seperate object. Beginning at the 
	tip/end affector, each limb follows a recursive process. 

	The end joint turns to point at the 'goal', then, when relevant, moves to that
	spot (this will detach it from the actual system). The next joint; 'end-1' or
	N-2 will follow the same process, only its goal is the tail/base of the tip 
	joint.
	
	This will propogate along each segment until all joints have aimed at and
	moved to the joint further on its length. Finally, the whole system is moved
	so the base of segment 0 (the root of the whole system) is placed at the
	systems origin.
*/
class trainMethod : public baseIKSystem {
public:
	trainMethod(SDL_Point, double, double, SDL_Colour, SDL_Renderer*, int);
	void calculate(coords) override;
};

trainMethod::trainMethod(SDL_Point _baseCoords, double segmentLength, 
	double segmentWidth, SDL_Colour _systemColour, SDL_Renderer* _renderer,
	int _windowHeight){
	
	baseCoords.x = _baseCoords.x;
	baseCoords.y = _baseCoords.y;

	for(int N = 0; N < segmentCount; N++){
		segments[N] = ikLimb({baseCoords.x+N*segmentLength, baseCoords.y},
			0, segmentLength, segmentWidth, _renderer, _systemColour);
	}
}

void trainMethod::calculate(coords goal){
	segments[2].self.pointAt(goal);
	double dist = segments[2].self.getDist(goal);
	dist -= segments[2].self.magnitude;
	segments[2].self.move(dist);

	for(int N = segmentCount-2; N>=0; N--){	
		segments[N].self.pointAt(segments[N+1].self.base);
		dist = segments[N].self.getDist(segments[N+1].self.base);
		dist -= segments[N].self.magnitude;
		segments[N].self.move(dist);
	}

	//shift system back to origin
	// rootSegCoords represents how far the system has shifted from its origin
	coords rootSegCoords = segments[0].self.base;
	double deltaX = rootSegCoords.x - baseCoords.x;	
	double deltaY = rootSegCoords.y - baseCoords.y;	
	for(int N = 0; N < segmentCount; N++){
		segments[N].self.base.x = segments[N].self.base.x - deltaX;
		segments[N].self.base.y = segments[N].self.base.y - deltaY;
	}
}

