# ifndef JOINT
# define JOINT

# include "Body.hpp"

class Joint
{
protected:
    Body* bodyA;
    Body* bodyB;
    Vector2 offsetA;
    Vector2 offsetB;
public:
    Joint(Body* a, Vector2 offset_a, Body* b, Vector2 offset_b);
    virtual void Solve();
};

# endif