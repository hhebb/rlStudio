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
    
    SCALAR m_a;
    SCALAR m_b;
    SCALAR i_a;
    SCALAR i_b;
    
    Vector2 pos_a;
    Vector2 pos_b;
    SCALAR angle_a;
    SCALAR angle_b;
    Vector2 v_a;
    Vector2 v_b;
    SCALAR w_a;
    SCALAR w_b;

    Vector2 r_a;
    Vector2 r_b;
    SCALAR rn_a;
    SCALAR rn_b;

    SCALAR n_impulse;
    SCALAR t_impulse;

    double k_normal;
    SCALAR m_normal;
    Vector2 tangent;
    double rt_a;
    double rt_b;
    
    SCALAR k_tangent;
    SCALAR m_tangent;

    // for restitution
    SCALAR v_rel;
    double e;
    double bias;

    // for friction
    double friction;

    // for position solve
    double bgt = .2;
    double slop = .001;
    double max_correction = .5;
    double separation = 0;

public:
    Collision(Body*, Body*);
    void FindCollisioninfo(Simplex);
    void FindManifolds();
    ClippedPoints Clip(Vector2, Vector2, Vector2, double);
    void Solve();
    void Push();

    void InitCollision();
    void Solve2();
    void SolvePosition();
};

# endif