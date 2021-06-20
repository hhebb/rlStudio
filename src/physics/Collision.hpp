# ifndef COLLISION
# define COLLISION

# include "Body.hpp"
# include "../Helper.hpp"

class Collision
{
private:
    Body* b1;
    Body* b2;
    Vector2 collisionNormal;
    float penetrationDepth;
    VERTEX manifolds;
public:
    void FindCollisioninfo(Simplex);
    void FindManifolds();
};

# endif