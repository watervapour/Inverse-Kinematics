#include <math.h>
#include <cstdio>

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







// =========== MATRICES ==========
class matrix{

};


struct mat31{
	double values[3];
};

struct mat33{
	double values[3][3];
};

struct mat44{
	double values[4][4];
};

void print44(mat44 matrix){
	printf("%f, %f, %f, %f\n %f, %f, %f, %f\n %f, %f, %f, %f\n %f, %f, %f, %f\n",
		matrix.values[0][0], matrix.values[0][1],matrix.values[0][2], matrix.values[0][3],
		matrix.values[1][0], matrix.values[1][1],matrix.values[1][2], matrix.values[1][3],
		matrix.values[2][0], matrix.values[2][1],matrix.values[2][2], matrix.values[2][3],
		matrix.values[3][0], matrix.values[3][1],matrix.values[3][2], matrix.values[3][3]);
}

void HTMConcat(mat33 rotation, mat31 displacement, mat44& result){
	for(int i=0;i<3;i++){
		for(int j = 0; j < 3; j++){
			result.values[i][j] = rotation.values[i][j];
		}
	}
	for(int i = 0; i < 3; i++){
		result.values[i][3] = displacement.values[i];
	}
	for(int j = 0; j < 4; j++){
		result.values[3][j] = (j!=3)?0:1;
	}
}

mat33 dot33(mat33 a, mat33 b, mat33& result){
	for(int i = 0; i < 3; i++){
		for(int j = 0; j < 3; j++){
			double x = 0;
			for(int o = 0; o < 3; o++){
				x += a.values[i][o]*b.values[o][j];
			}
			result.values[i][j] = x;	
		}
	}
}
mat44 dot44(mat44 a, mat44 b, mat44& result){
	for(int i = 0; i < 4; i++){
		for(int j = 0; j < 4; j++){
			double x = 0;
			for(int o = 0; o < 4; o++){
				x += a.values[i][o]*b.values[o][j];
			}
			result.values[i][j] = x;	
		}
	}
}
