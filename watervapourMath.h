#include <math.h>

#define PI 3.14159265358

struct coords{
	double x;
	double y;
};

class vector2d {
public:
	// the base location of object
	coords base;
	double angle;
	double magnitude;

	vector2d();
	// coordinate location of vector tip
	coords getHead();
	void setHead(coords);

	//math
	vector2d add(vector2d);
	vector2d sub(vector2d);
	vector2d dot(vector2d);

	// positional functions
	void pointAt(coords);
	double angleBetween(vector2d);
	double getDist(coords);
	void move(double);
	coords intercept(vector2d);
	void reflect(vector2d);

	// sets to unit length
	void normalise();
};

vector2d::vector2d(){
	base.x = 0;
	base.y = 0;

	angle = 0;
	magnitude = 1;
}

coords vector2d::getHead(){
	coords head = base;
	head.x += cos(angle) * magnitude;
	head.y += sin(angle) * magnitude;
	return head;
}
void vector2d::setHead(coords location){
	double xLength = location.x - base.x;
	double yLength = location.y - base.y;

	angle = atan2(yLength, xLength);
	magnitude = sqrt(xLength * xLength + yLength*yLength);
}


//add, sub, dot
vector2d vector2d::add(vector2d b){
	setHead(b.getHead());
}

// positionals
void vector2d::pointAt(coords goal){
	double originalMagnitude = magnitude;
	setHead(goal);
	magnitude = originalMagnitude;
}

double vector2d::angleBetween(vector2d vectorB){
	return 0;
}
double vector2d::getDist(coords goal){
	double deltaX = goal.x - base.x;
	double deltaY = goal.y - base.y;
	return sqrt((deltaX * deltaX) + (deltaY * deltaY));
}
void vector2d::move(double distance){
	base.x = base.x + cos(angle)*distance;
	base.y = base.y + sin(angle)*distance;
}
coords vector2d::intercept(vector2d vectorB){

}

void vector2d::reflect(vector2d vectorB){

}

//normalise
void vector2d::normalise(){
	magnitude = 1;
}
