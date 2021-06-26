# include "Body.hpp"
# include <iostream>
using namespace std;

Body::Body(POLY_DATA vertices, Vector2 pos, SCALAR rot, float rad, int i, SCALAR density=1)
{
    // set collider
    // calc collider's centroid, move vertices.
    id = i;
    position = pos; // centroid 로 작동한다.
    rotation = rot;
    collider = new Collider(vertices, position, radius);
    radius = rad; // ??? deprecated.
    
    
    // get mass data from created collider.
    mass = collider->CalculateMass(density);
    inverseMass = mass == 0 ? 0 : 1 / mass;
    inertia = collider->CalculateInertia();
    inverseInertia = inertia == 0 ? 0 : 1 / inertia;
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

SCALAR Body::GetAngularVelocity()
{
    return angularVelocity;
}

SCALAR Body::GetMass()
{
    return mass;
}

SCALAR Body::GetInertia()
{
    return inertia;
}

Collider* Body::GetCollider()
{
    return collider;
}

void Body::AddForce(Vector2 f)
{
    // force = force + f;
    force += f;
}

void Body::AddTorque(SCALAR t)
{
    torque += t;
}

void Body::AddImpulseAt(Vector2 impulse, Vector2 pos)
{
    // dynamics for impulse at arbitrary point.
    // for collision solve.
    velocity += impulse * inverseMass;
    angularVelocity += pos.Cross(impulse) * inverseInertia;
}

void Body::CalculateVelocity()
{
    // velocity = velocity + force * inverseMass * DELTA_TIME;
    velocity += force * inverseMass * DELTA_TIME;
}

void Body::CalculateAngularVelocity()
{
    angularVelocity += torque * DELTA_TIME;
}

void Body::CalculatePosition()
{
    // position = position + velocity * DELTA_TIME;
    position += velocity * DELTA_TIME;
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

