# ifndef DISTANCE_JOINT
# define DISTNACE_JOINT

# include "Joint.hpp"

class DistanceJoint: public Joint
{
public:
    DistanceJoint(Body* a, Vector2 offset_a, Body* b, Vector2 offset_b);
    void InitJoint() override;
    void Solve() override;

};

# endif