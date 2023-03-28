#include "my_math.h"


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
