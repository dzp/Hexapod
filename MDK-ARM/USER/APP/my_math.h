#ifndef MY_MATH_H
#define MY_MATH_H



#define PI 3.14159f // Ô²ÖÜÂÊ

class Thetas
{
public:
    float angle[3];
    Thetas(float angle_0 = 0, float angle_1 = 0, float angle_2 = 0)
    {
        this->angle[0] = angle_0;
        this->angle[1] = angle_1;
        this->angle[2] = angle_2;
    }
    Thetas& operator=(const float angles[3]);
    Thetas(const float angles[3]);
};

Thetas operator+(const Thetas& theta1, const Thetas& theta2);
Thetas operator-(const Thetas& theta1, const Thetas& theta2);

class Position3
{
public:
    float x;
    float y;
    float z;
    Position3(float x = 0, float y = 0, float z = 0)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }
      
};
Position3 operator+(const Position3& pos1,const Position3& pos2);
Position3 operator-(const Position3& pos1,const Position3& pos2);


#endif
