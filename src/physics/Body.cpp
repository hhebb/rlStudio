# include "Body.hpp"
# include <iostream>
using namespace std;

Body::Body(POLY_DATA vertices, Vector2 pos, SCALAR rot, int i, SCALAR density, BodyType t)
{
    // set collider
    // calc collider's centroid, move vertices.
    id = i;
    position = pos; // centroid 로 대체된다.
    rotation = rot;
    collider = new Collider(vertices, position, rotation, radius);
    // SetPosition(pos);
    // SetRotation(rot);
    type = t;
    PrintScalar("type", t);
    // radius = rad; // ??? deprecated.
    
    
    // get mass data from created collider.
    mass = type == DYNAMIC ? collider->CalculateMass(density) : 0;
    inverseMass = mass == 0 ? 0 : 1 / mass;
    inertia = type == DYNAMIC ? collider->CalculateInertia() : 0;
    inverseInertia = inertia == 0 ? 0 : 1 / inertia;
    // PrintScalar("mass", mass);
    // PrintScalar("i_mass", inverseMass);
    // PrintScalar("inertia", inertia);
    // PrintScalar("i_intertia", inverseInertia);
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

void Body::SetPosition(Vector2 pos)
{
    position = pos;
    collider->SetPosition(position);
}

void Body::SetRotation(SCALAR rot)
{
    rotation = rot;
    collider->SetRotation(rotation);
}

void Body::AddForce(Vector2 f)
{
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
    velocity += impulse * inverseMass * .01;
    angularVelocity += pos.Cross(impulse) * inverseInertia * .01;
}

void Body::CalculateVelocity()
{
    velocity += force * inverseMass * DELTA_TIME;
}

void Body::CalculateAngularVelocity()
{
    angularVelocity += torque * inverseInertia * DELTA_TIME;
}

void Body::CalculatePosition()
{
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

void Body::UpdateCentroid()
{
    position = collider->GetCentroid();
    // PrintVector("body centroid", position);
}

// test
void Body::SetVel(SCALAR w)
{
    angularVelocity = w;
}