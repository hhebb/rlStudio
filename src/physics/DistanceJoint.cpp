# include "DistanceJoint.hpp"

DistanceJoint::DistanceJoint(Body* a, Vector2 offset_a, Body* b, Vector2 offset_b): Joint(a, offset_a, b, offset_b)
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

    d = (offset_a - offset_a).GetLength();
}

void DistanceJoint::InitJoint()
{
    pos_a = bodyA->GetPosition();
    pos_b = bodyB->GetPosition();
    r_a = offsetA; //bodyA->GetPosition() + offsetA;
    r_b = offsetB; //bodyB->GetPosition() + offsetB;
    t_a = bodyA->GetRotation();
    t_b = bodyB->GetRotation();
    v_a = bodyA->GetVelocity();
    v_b = bodyB->GetVelocity();
    w_a = bodyA->GetAngularVelocity();
    w_b = bodyB->GetAngularVelocity();

    mr_a = {cos(t_a) * r_a.x - sin(t_a) * r_a.y, sin(t_a) * r_a.x + cos(t_a) * r_a.y};
    mr_b = {cos(t_b) * r_b.x - sin(t_b) * r_b.y, sin(t_b) * r_b.x + cos(t_b) * r_b.y};
    m_u = pos_b + mr_b - (pos_a + mr_a);
    cr_au = mr_a.Cross(m_u);
    cr_bu = mr_b.Cross(m_u);
    SCALAR inv_tmp_m = m_a + i_a * cr_au * cr_au + m_b + i_b * cr_bu * cr_bu;
    inv_m = inv_tmp_m != 0 ? 1 / inv_tmp_m : 0;
    double m_gamma = 0;
    double bias = 0;
    SCALAR m_soft = inv_m;
    m_impulse = 0;

}

void DistanceJoint::VelocitySolve()
{
    v_a = bodyA->GetVelocity();
    v_b = bodyB->GetVelocity();
    w_a = bodyA->GetAngularVelocity();
    w_b = bodyB->GetAngularVelocity();

    Vector2 vp_a = bodyA->GetVelocity() + mr_a.Cross(w_a);
    Vector2 vp_b = bodyB->GetVelocity() + mr_b.Cross(w_b);
    SCALAR C_dot = m_u.Dot(vp_b - vp_a); // = J * V
    SCALAR impulse = -inv_m * C_dot;
    m_impulse += impulse;

    Vector2 P = m_u * impulse;

    bodyA->AddVelocity(-P * m_a, -mr_a.Cross(P) * i_a);
    bodyB->AddVelocity(P * m_b, mr_b.Cross(P) * i_b);

    PrintVector("p impulse", P);
}

void DistanceJoint::PositionSolve()
{
    pos_a = bodyA->GetPosition();
    pos_b = bodyB->GetPosition();
    t_a = bodyA->GetRotation();
    t_b = bodyB->GetRotation();

    r_a = {cos(t_a) * r_a.x - sin(t_a) * r_a.y, sin(t_a) * r_a.x + cos(t_a) * r_a.y};
    r_b = {cos(t_b) * r_b.x - sin(t_b) * r_b.y, sin(t_b) * r_b.x + cos(t_b) * r_b.y};
    Vector2 u = pos_b + r_b - (pos_a + r_a);
    SCALAR length = u.GetLength();
    double C = length - d;

    SCALAR impulse = -inv_m * C;
    Vector2 P = u * impulse;
    bodyA->AddTranslation(-P * m_a, - r_a.Cross(P) * i_a);
    bodyB->AddTranslation(P * m_b, + r_b.Cross(P) * i_b);
    PrintVector("dist impulse", P);
    PrintVector("angle", r_b);
    PrintScalar("m", m_b);
}


void DistanceJoint::ApplyJointImpulse()
{}