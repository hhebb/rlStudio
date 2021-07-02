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
    double penetrationDepth;
    ClippedPoints manifolds;
    bool flip;
public:
    Collision(Body*, Body*);
    void FindCollisioninfo(Simplex);
    void FindManifolds();
    ClippedPoints Clip(Vector2, Vector2, Vector2, double);
    void Solve();
};

# endif