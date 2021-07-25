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
	tl.x = (int)(head.x + limbDX*0.7);
	tl.y = winHeight - (int)(head.y + limbDY*0.7);

	tr.x = (int)(head.x - limbDX*0.7);
	tr.y = winHeight - (int)(head.y - limbDY*0.7);
	
	
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
		- ikInfo: a matrix providing values for text on screen
	Private:
		- baseCoords: coordinates for the base of the system (where it is mounted
			to the outer world)
		- segments[]: an array of ikLimbs, represents the system
*/
class baseIKSystem{
public:
	baseIKSystem() = default;
	baseIKSystem(SDL_Point, double, double, SDL_Colour, SDL_Renderer*, int);
	virtual void calculate(coords);
	void draw();

	mat31 ikInfo;
protected:
	coords baseCoords;
	const int segmentCount = 3;
	ikLimb segments[3];
	
};

baseIKSystem::baseIKSystem(SDL_Point _baseCoords, double segmentLength, 
	double segmentWidth, SDL_Colour _systemColour, SDL_Renderer* _renderer,
	int _windowHeight){
		printf("custom constructor for basIKSystem\n");
}

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

	With a system of N points, any numbering will dicate the base/root element as
	segment 1, with the end affecter/tip point being labeled N. In the case that 
	a reference to a specifiv 'segment', or the arm/line between points, the
	number used will be that of the lower numbered joint. For example, the segment
	between point 1 and 2 will be segment 1.
	
	As arrays are zero indexed, point N will have the position N-1, and point 1
	will be array[0]. The numbering beginning from one is a convention found in
	research papers on the material.
	
	For the time being, each system will use a fixed array for the segments,
	as a future goal, perhaps vectors, or other dynamic storage classes may be
	used. Also may have colours be gradients along segment chain.
*/



/* 
	This algorithm for invsere kinematics is described in a video from
	the youtube channel 'the coding train'.

	The method treats each limb as a seperate object. Beginning at the 
	tip/end affector, each limb follows a recursive process. 

	The end segment turns to aim at the 'goal', then, when relevant, moves to that
	spot (this will detach it from the actual system). The next joint; 'end-1' or
	N-1 will follow the same process, only its goal is the tail/base of the tip 
	joint.
	
	This will propogate along each segment until all joints have aimed at and
	moved to the joint further on its length. Finally, the whole system is moved
	so the base of segment 1 (point 1, the root of the whole system) is placed at
	the systems origin.
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
	segments[segmentCount-1].self.pointAt(goal);
	double dist = segments[segmentCount-1].self.getDist(goal);
	dist -= segments[segmentCount-1].self.magnitude;
	segments[segmentCount-1].self.move(dist);
	ikInfo.values[segmentCount-1] = segments[segmentCount-1].self.angle;

	for(int N = segmentCount-2; N>=0; N--){	
		segments[N].self.pointAt(segments[N+1].self.base);
		dist = segments[N].self.getDist(segments[N+1].self.base);
		dist -= segments[N].self.magnitude;
		segments[N].self.move(dist);
		ikInfo.values[N] = segments[N].self.angle;
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

/* 
	Fowards And Backward Reaching Inverse Kinematics is an algorithm that boasts
	fast resolve times, and high accuracy. The also supports limits on how much
	joints can turn, aswell as multiple end effectors.

	The system begins by setting the position of point N to the same location as
	the goal. The point will now be referred to as p'N. A ' is used to indicate
	that a point has receive an update (in each iteration this would be increased
	to '' and then ''', however, only 2 iterations of the algorithm needs to be
	described).

	Next, a line is drawn from p'N, to p(N-1). The point p(N-1) is then placed at
	its new home of p'(N-1). This is found to be along the line just created, with
	a distance of d(N-1). This length is the intended length of the segment
	connecting points p(N-1) and pN.

	The system then continues along the rest of the chain, now creating a line
	between p'(N-1) and p(N-2). p'(N-2) is then also found as a distance along
	this length. 

	This concludes the 'backwards' portion of the algorithm. The process then 
	begins again in a 'forwards' pass. First, point 1 (p'1 is the point affected,
	as the backwards routing has updated its position once so far) has its 
	location set to the intended base of the system, making it now p''1. A line is
	made to the point p'2. p'2 is then moved along this, to its new home of p''2, 
	at a distance of d1 from the origin point.

	Once all the elements have been affected twice (once in the backwards
	direction, and once in the forwards direction) the algorithm has completed one
	full iteration. 
*/

class fabrikMethod : public baseIKSystem {
public:
	fabrikMethod(SDL_Point, double, double, SDL_Colour, SDL_Renderer*, int);
	/*
	fabrikMethod(SDL_Point _baseCoords, double segmentLength, 
	double segmentWidth, SDL_Colour _systemColour, SDL_Renderer* _renderer,
	int _windowHeight):baseIKSystem(_baseCoords, segmentLength, segmentWidth, _systemColour,
		_renderer, _windowHeight){}
	*/	
	void calculate(coords) override;
};

fabrikMethod::fabrikMethod(SDL_Point _baseCoords, double segmentLength, 
	double segmentWidth, SDL_Colour _systemColour, SDL_Renderer* _renderer,
	int _windowHeight){
	
	baseCoords.x = _baseCoords.x;
	baseCoords.y = _baseCoords.y;

	for(int N = 0; N < segmentCount; N++){
		segments[N] = ikLimb({baseCoords.x+N*segmentLength, baseCoords.y},
			0, segmentLength, segmentWidth, _renderer, _systemColour);
	}
}
void fabrikMethod::calculate(coords goal){
	// this is used for calculating the intercepts/etc
	vector2d line;


	// Backwards pass
	vector2d* currentLimb = &segments[segmentCount-1].self;
	// p'N
	line.base = goal; 
	// find p(N-1)
	line.pointAt(currentLimb->base);
	// we now calculate where p'(N-1) is
	line.magnitude = currentLimb->magnitude; 
	//set our limb to use these new positions
	currentLimb->base = line.getHead();
	currentLimb->setHead(line.base);

	for(int C = segmentCount -2;C >=0; C--){
		currentLimb = &segments[C].self;
		line.base = segments[C+1].self.base;
		line.pointAt(currentLimb->base);
		line.magnitude = currentLimb->magnitude; 
		//set our limb to use these new positions
		currentLimb->base = line.getHead();
		currentLimb->setHead(line.base);
	}
	// forwards pass
	currentLimb = &segments[0].self;
	// p''1
	line.base = baseCoords; 
	// find p'2
	line.pointAt(currentLimb->getHead());
	// we now calculate where p'(N-1) is
	line.magnitude = currentLimb->magnitude; 
	//set our limb to use these new positions
	currentLimb->base = line.base;
	currentLimb->setHead(line.getHead());

	for(int C = 1;C < segmentCount; C++){
		currentLimb = &segments[C].self;
		line.base = segments[C-1].self.getHead();
		line.pointAt(currentLimb->getHead());
		line.magnitude = currentLimb->magnitude; 
		//set our limb to use these new positions
		currentLimb->base = line.base;
		currentLimb->setHead(line.getHead());
	}
}

/*
	HTM method
*/
class htmMethod : public baseIKSystem {
public:
	htmMethod(SDL_Point, double, double, SDL_Colour, SDL_Renderer*, int);
	void draw();
	void calculate(double, double, double);
private:
	SDL_Renderer* renderer;
	int windowHeight;
};

htmMethod::htmMethod(SDL_Point _baseCoords, double segmentLength, 
	double segmentWidth, SDL_Colour _systemColour, SDL_Renderer* _renderer,
	int _windowHeight){
	
	baseCoords.x = _baseCoords.x;
	baseCoords.y = _baseCoords.y;

	renderer = _renderer;
	windowHeight = _windowHeight;

	for(int N = 0; N < segmentCount; N++){
		segments[N] = ikLimb({baseCoords.x+N*segmentLength, baseCoords.y},
			0, segmentLength, segmentWidth, _renderer, _systemColour);
	}
}

void htmMethod::calculate(double a1, double a2, double a3){
	//01
	double ca1 = cos(a1);
	double sa1 = sin(a1);
	double l1 = segments[0].self.magnitude;
	mat33 R01 = {ca1, -1*sa1, 0, 
				sa1, ca1, 0,
				0, 0, 1};
	mat31 D01 = {l1*ca1, l1*sa1, 0};	
	mat44 H01;
	HTMConcat(R01, D01, H01);

	//12
	double ca2 = cos(a2);
	double sa2 = sin(a2);
	double l2 = segments[1].self.magnitude;
	mat33 R12 = {ca2, -1*sa2, 0, 
				sa2, ca2, 0,
				0, 0, 1};
	mat31 D12 = {l2*ca2, l2*sa2, 0};	
	mat44 H12;
	HTMConcat(R12, D12, H12);

	//23
	double ca3 = cos(a3);
	double sa3 = sin(a3);
	double l3 = segments[2].self.magnitude;
	mat33 R23 = {ca3, -1*sa3, 0, 
				sa3, ca3, 0,
				0, 0, 1};
	mat31 D23 = {l3*ca3, l3*sa3, 0};	
	mat44 H23;
	HTMConcat(R23, D23, H23);

	mat44 H02;
	dot44(H01, H12, H02);	
	mat44 H03;
	dot44(H02, H23, H03);	
	
	segments[0].self.angle = a1;	
	segments[1].self.base = segments[0].self.getHead();
	segments[1].self.angle = a2 + a1;	
	segments[2].self.base = segments[1].self.getHead();
	segments[2].self.angle = a3 + a2 + a1;	
}

void htmMethod::draw(){
	coords target = segments[2].self.getHead();
	int htmX = (int)target.x;
	int htmY = windowHeight-(int)target.y;
	SDL_Rect htmDisplacement = {htmX-5, htmY-5, 10, 10};
	SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
	SDL_RenderFillRect(renderer, &htmDisplacement);
	baseIKSystem::draw();
}
