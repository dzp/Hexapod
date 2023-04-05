#include "my_math.h"


void Position3::zero()
{
    x = 0;
    y = 0;
    z = 0;
}

Position3 operator+(const Position3& pos1,const Position3& pos2)
{
    Position3 pos;
    pos.x = pos1.x + pos2.x;
    pos.y = pos1.y + pos2.y;
    pos.z = pos1.z + pos2.z;
    return pos;
}

Position3 operator-(const Position3& pos1,const Position3& pos2)
{
    Position3 pos;
    pos.x = pos1.x - pos2.x;
    pos.y = pos1.y - pos2.y;
    pos.z = pos1.z - pos2.z;
    return pos;
}

Thetas operator+(const Thetas& theta1, const Thetas& theta2)
{
    Thetas theta;
    theta.angle[0] = theta1.angle[0] + theta2.angle[0];
    theta.angle[1] = theta1.angle[1] + theta2.angle[1];
    theta.angle[2] = theta1.angle[2] + theta2.angle[2];
    return theta;
}

Thetas operator-(const Thetas& theta1, const Thetas& theta2)
{
    Thetas theta;
    theta.angle[0] = theta1.angle[0] - theta2.angle[0];
    theta.angle[1] = theta1.angle[1] - theta2.angle[1];
    theta.angle[2] = theta1.angle[2] - theta2.angle[2];
    return theta;
}

Thetas& Thetas::operator=(const float angles[3])
{
    this->angle[0] = angles[0];
    this->angle[1] = angles[1];
    this->angle[2] = angles[2];
    return *this;
}

Thetas::Thetas(const float angles[3])
{
    this->angle[0] = angles[0];
    this->angle[1] = angles[1];
    this->angle[2] = angles[2];
}

void value_limit(float &val,float min,float max)
{
	if(val>max)
		val = max;
	if(val<min)
		val = min;
}
