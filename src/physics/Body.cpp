# include "Body.hpp"
# include <iostream>
using namespace std;

Body::Body(VERTEX vertices, Vector2 pos, SCALAR rot, SCALAR m, float rad)
{
    position = pos;
    rotation = rot;
    mass = m;
    inverseMass = m != 0 ? 1 / mass : 0;
    radius = rad;
    velocity = {0, 0};
    collider = new Collider(vertices, radius);
}


void Body::AddForce(Vector2 f)
{
    force.x += f.x;
    force.y += f.y;
}

void Body::AddTorque(SCALAR t)
{
    torque += t;
}

void Body::CalculateVelocity()
{
    velocity.x += force.x * inverseMass * DELTA_TIME;
    velocity.y += force.y * inverseMass * DELTA_TIME;

}

void Body::CalculateAngularVelocity()
{
    angularVelocity += torque * DELTA_TIME;
}

void Body::CalculatePosition()
{
    position.x += velocity.x * DELTA_TIME;
    position.y += velocity.y * DELTA_TIME;
}

void Body::CalculateAngle()
{
    rotation += angularVelocity * DELTA_TIME;
}

void Body::ClearForce()
{
    force = {0, 0};
}

void Body::ClearTorque()
{
    torque = 0;
}

Collider* Body::GetCollider()
{
    return collider;
}

Vector2 Body::GetPosition()
{
    return position;
}

SCALAR Body::GetRotation()
{
    return rotation;
}

Vector2 Body::GetVelocity()
{
    return velocity;
}

