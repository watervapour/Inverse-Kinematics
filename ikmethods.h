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

	double ikInfo[5];
protected:
	coords baseCoords;
	const int segmentCount = 5;
	ikLimb segments[5];
	double angleSum = 0;
	double tempSegmentAngle = 0;
};

baseIKSystem::baseIKSystem(SDL_Point _baseCoords, double segmentLength, 
	double segmentWidth, SDL_Colour _systemColour, SDL_Renderer* _renderer,
	int _windowHeight){
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
	ikInfo[segmentCount-1] = segments[segmentCount-1].self.angle;


	for(int N = segmentCount-2; N>=0; N--){	
		segments[N].self.pointAt(segments[N+1].self.base);
		dist = segments[N].self.getDist(segments[N+1].self.base);
		dist -= segments[N].self.magnitude;
		segments[N].self.move(dist);
		ikInfo[N] = segments[N].self.angle;
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
	Cyclic Coordinate descent

	CCD iterates once per segment, beginning at the 'tip'. At each step, an angle
	and vector are calculated which describe how the segment is rotated.

	The algorithm works with three points:
		- pf the coordinates of the goal
		- pe the position of the end effector
		- pc the base / pivot point of the current segment of the iteration

	Two vectors are created (pce and pcf), which point from pc to either pe or pf.
	Taking a dot product, then dividing by magnitudes, then inverse cos-ing 
	provides the angle between vectors. Whilst the angles could be simply
	subtracted in this particular implementation, that would be less proper, and 
	doesn't result in a full implementation (and also wouldnt scale well to 3D).
	The result of this is stored in the variable 'theta'.

	Next, a cross product is taken between the vectors. In this 2D case, the
	result will simply point 'into' or 'out of' the screen, and describe
	counter-clockwise and clockwise respectively. This is stored in 'd'
	which stands for 'direction', and in 3D would be a vector. This can be
	visualised as following the right-hand curl rule used with electro-magnetism.
	Imagine the pce vector as between wrist and knuckles. Then with pcf as
	wrist to finger tips. Your thumb then points in the direction of d.

	Theta is then modified to.In this 2D case, scalar multiplication of theta and 
	d is sufficient. In 3D the d is used as an axis, around which theta radians
	of rotation occurs.	The pce vector is then rotated by an amount of theta. 
	
	The whole process will then begin from joint C-1. pc and pe are
	recalculated. Then the vectors and all steps follow on. The algorithm has
	finished an iteration once all segments have rotated once. As rotation of
	each joint will bring previous rotations out of their desired spot,
	multiple passes will be required to reach a steady-state.
	
*/
class ccdMethod : public baseIKSystem {
public:
	ccdMethod(SDL_Point, double, double, SDL_Colour, SDL_Renderer*, int);
	void calculate(coords) override;
};

ccdMethod::ccdMethod(SDL_Point _baseCoords, double segmentLength, 
	double segmentWidth, SDL_Colour _systemColour, SDL_Renderer* _renderer,
	int _windowHeight){
	
	baseCoords.x = _baseCoords.x;
	baseCoords.y = _baseCoords.y;

	for(int N = 0; N < segmentCount; N++){
		segments[N] = ikLimb({baseCoords.x+N*segmentLength, baseCoords.y},
			0, segmentLength, segmentWidth, _renderer, _systemColour);
	}
}
void ccdMethod::calculate(coords goal){
	coords pe;
	coords pf = goal;
	vector2d pce;
	vector2d pcf;

	// iterate over each segment
	for(int N = segmentCount -1 ; N >= 0; N--){
		// fetch coordinates of points, then assemble vectors
		coords pc = segments[N].self.base;
		pe = segments[4].self.getHead();
		pce.base = pc;
		pce.setHead(pe);
		pcf.base = pc;
		pcf.setHead(pf);
		
		// calculate theta and d
		double dotResult = pce.dot(pcf);
		double theta = acos(dotResult/(pce.magnitude * pcf.magnitude));
		double d = pce.cross(pcf);	
		// normalise d to 1 or -1
		// if -1 < d < 1 then each turn will be less than the amount required
		// this can be a desirable effect, as the arm will move smoother and
		// slower
		d = (d>0)?1:-1;
		// make theta follow CW/CCW direction based on d
		theta *= d;

		// make sure the angle propogates through, as rotation of each joint
		// is relative to previous.
		// position is also updated
		for(int i = N; i < segmentCount; i++){
			// apply theta to affected limb
			segments[i].self.angle += theta;

			// segments[0] is the base, and does not have a parent joint to 
			// be moved by
		 	if(i != 0)
				// set rotation coordinate as appropriate
		 		segments[i].self.base = segments[i-1].self.getHead();
		}
	}
}

/*
	Homogeneous Transform Matrix method

	HTMs are matrices which can describe the rotation and location of a point
	in a robotic arm system. See video/playlist in README for full details.

	Each section (01, 12, 23, etc.) decribes the relation between reference
	frame {0 and 1} or {1 and 2} or {2 and 3}. The '01' section will be
	highlighted.

	The variables ca1 and sa1 are variables used repeatedly, and refer to the
	X and Y components of rotation being undergone. l1 is the length of the
	segment. R01 is a rotation matrix, due to the 2D nature of the 
	implementations, these rotation matrices are simply Z rotation matrices.
	D01 describes the displacement, basically where frame 1 is after frame 0
	is rotated. An HTM is the created by a special concatenation process.

	As all the HTMs are the same size, they can simply be dot multiplied 
	together. As a final step in the calculate function, the segments are
	updated, so that they can be drawn properly.

	The draw function reads from the final 05 HTM, and draws a square at
	the location specified, this is to visually show how the position is
	the same as where the armature rests.
*/
class htmMethod : public baseIKSystem {
public:
	htmMethod(SDL_Point, double, double, SDL_Colour, SDL_Renderer*, int);
	void draw();
	void calculate(double, double, double, double, double);
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

void htmMethod::calculate(double a1, double a2, double a3, double a4, double a5){
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

	//34
	double ca4 = cos(a4);
	double sa4 = sin(a4);
	double l4 = segments[3].self.magnitude;
	mat33 R34 = {ca4, -1*sa4, 0, 
				sa4, ca4, 0,
				0, 0, 1};
	mat31 D34 = {l4*ca4, l4*sa4, 0};	
	mat44 H34;
	HTMConcat(R34, D34, H34);


	//45
	double ca5 = cos(a5);
	double sa5 = sin(a5);
	double l5 = segments[4].self.magnitude;
	mat33 R45 = {ca5, -1*sa5, 0, 
				sa5, ca5, 0,
				0, 0, 1};
	mat31 D45 = {l5*ca5, l5*sa5, 0};	
	mat44 H45;
	HTMConcat(R45, D45, H45);

	// Create the HTM using dot products
	mat44 H02;
	dot44(H01, H12, H02);	
	mat44 H24;
	dot44(H23, H34, H24);
	mat44 H04;
	dot44(H02, H24, H04);	
	mat44 H05;
	dot44(H04, H45, H05);
	
	// apply the inputted angles to the system
	segments[0].self.angle = a1;	
	segments[1].self.base = segments[0].self.getHead();
	segments[1].self.angle = a2 + a1;	
	segments[2].self.base = segments[1].self.getHead();
	segments[2].self.angle = a3 + a2 + a1;	
	segments[3].self.base = segments[2].self.getHead();
	segments[3].self.angle = a4 + a3 + a2 + a1;	
	segments[4].self.base = segments[3].self.getHead();
	segments[4].self.angle = a5 + a4 + a3 + a2 + a1;	
}

void htmMethod::draw(){
	coords target = segments[4].self.getHead();
	int htmX = (int)target.x;
	int htmY = windowHeight-(int)target.y;
	SDL_Rect htmDisplacement = {htmX-5, htmY-5, 10, 10};
	SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
	SDL_RenderFillRect(renderer, &htmDisplacement);
	baseIKSystem::draw();
}
