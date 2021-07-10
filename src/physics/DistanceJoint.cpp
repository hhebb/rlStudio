# include "DistanceJoint.hpp"

DistanceJoint::DistanceJoint(Body* a, Vector2 offset_a, Body* b, Vector2 offset_b): Joint(a, offset_a, b, offset_b)
{
    Joint(a, offset_a, b, offset_b);
}

void DistanceJoint::InitJoint()
{
    // J is 1x4 jacobian matrix.
    

    // K is 2x2 JMJ matrix.
    double K_11 = 0;
    double K_12 = 0;
    double K_21 = 0;
    double K_22 = 0;
}

void DistanceJoint::Solve()
{
    Vector2 pa_dot = bodyA->GetVelocity() + offsetA * bodyA->GetAngularVelocity();
    Vector2 pb_dot = bodyB->GetVelocity() + offsetB * bodyB->GetAngularVelocity();
    Vector2 C_dot = pa_dot - pb_dot; // = J * V
    

    // impulse - J * K^-1 * C'
    // Vector2 impulse = ; // 

}