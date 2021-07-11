# include "RevoluteJoint.hpp"
# include "cmath"
# include "../Definition.hpp"

RevoluteJoint::RevoluteJoint(Body* a, Vector2 offset_a, Body* b, Vector2 offset_b): Joint(a, offset_a, b, offset_b)
{
    // Joint(a, offset_a, b, offset_b);
    bodyA = a;
    bodyB = b;
    offsetA = offset_a;
    offsetB = offset_b;

    m_a = bodyA->GetInverseMass();
    m_b = bodyB->GetInverseMass();
    i_a = bodyA->GetInverseInertia();
    i_b = bodyB->GetInverseInertia();
}

void RevoluteJoint::InitJoint()
{
    r_a = offsetA; //bodyA->GetPosition() + offsetA;
    r_b = offsetB; //bodyB->GetPosition() + offsetB;
    t_a = bodyA->GetRotation() * INVERSE_RADIAN;
    t_b = bodyB->GetRotation() * INVERSE_RADIAN;
    mr_a = {cos(t_a) * r_a.x - sin(t_a) * r_a.y, sin(t_a) * r_a.x + cos(t_a) * r_a.y};
    mr_b = {cos(t_b) * r_b.x - sin(t_b) * r_b.y, sin(t_b) * r_b.x + cos(t_b) * r_b.y};

    // K is 2x2 JMJ matrix.
    K.m11 = m_a + m_b + mr_a.y * mr_a.y * i_a + mr_b.y * mr_b.y * i_b;
    K.m12 = -mr_a.y * mr_a.x * i_a - mr_b.y * mr_b.x * i_b;
    K.m21 = K.m12;
    K.m22 = m_a + m_b + mr_a.x * mr_a.x * i_a + mr_b.x * mr_b.x * i_b;

    // v_a = Vector2{0, 0};
    // v_b = Vector2{0, 0};
    // w_a = 0;
    // w_b = 0;
}

void RevoluteJoint::Solve()
{
    Vector2 pa_dot = bodyA->GetVelocity() + offsetA.Cross(bodyA->GetAngularVelocity());
    Vector2 pb_dot = bodyB->GetVelocity() + offsetB.Cross(bodyB->GetAngularVelocity());
    Vector2 C_dot = pa_dot - pb_dot; // = J * V
    Vector2 impulse = K.Solve(C_dot);

    bodyA->AddJointImpulse(impulse * m_a, i_a * mr_a.Cross(impulse));
    bodyB->AddJointImpulse(impulse * m_b, i_b * mr_b.Cross(impulse));
    // v_a += impulse * m_a;
    // v_b += impulse * m_b;
    // w_a += i_a * mr_a.Cross(impulse);
    // w_b += i_b * mr_b.Cross(impulse);

    PrintVector("joint impulse", impulse);

}

void RevoluteJoint::ApplyJointImpulse()
{
    bodyA->AddJointImpulse(v_a, w_a);
    bodyB->AddJointImpulse(v_b, w_b);
}