# ifndef BODY
# define BODY

# include "../namespace.hpp"
# include "Collider.hpp"

// class Body;

class Body
{
private:
    Vector2 position;
    SCALAR rotation;
    Vector2 velocity;
    SCALAR angularVelocity;

    Vector2 force;
    SCALAR torque;
    SCALAR inverseMass;
    SCALAR mass;
    float radius;

    Collider* collider;

public:
    Body(VERTEX, Vector2, SCALAR, SCALAR, float);
    void AddForce(Vector2);
    void ClearForce();
    void CalculateVelocity();
    void CalculatePosition();
    Collider* GetCollider();
    Vector2 GetPosition();
    SCALAR GetRotation();

};

///
Body::Body(VERTEX vertices, Vector2 pos, SCALAR rot, SCALAR m, float rad)
{
    position = pos;
    rotation = rot;
    mass = m;
    inverseMass = 1 / m;
    radius = rad;
    collider = new Collider(this, vertices, radius);
}


void Body::AddForce(Vector2 f)
{
    force.x += f.x;
    force.y += f.y;

}

void Body::CalculateVelocity()
{
    velocity.x += force.x * inverseMass * DELTA_TIME;
    velocity.y += force.y * inverseMass * DELTA_TIME;

}

void Body::ClearForce()
{
    force.x = 0;
    force.y = 0;
}

Collider* Body::GetCollider()
{
    return collider;
}

# endif