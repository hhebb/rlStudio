# include "Collision.hpp"
# include <cmath>

Collision::Collision(Body* b1, Body* b2)
{
    this->b1 = b1;
    this->b2 = b2;
}

void Collision::FindCollisioninfo(Simplex simplex)
{
    // EPA algorithm for find collision normal, penetration depth.
    while (true)
    {
        Edge e = FindClosetEdge(simplex);
        Vector2 p = SupportFunction(b1->GetCollider()->GetVertices(), b2->GetCollider()->GetVertices(), e.normal);
        double d = p.Dot(e.normal);
        if (d - e.distance < TOLERANCE)
        {
            collisionNormal = e.normal;
            penetrationDepth = d;
            cout << "> collision info: " << collisionNormal.x << ", " << collisionNormal.y << ", " << penetrationDepth << endl;            
            break;
        }
        else
        {
            simplex.Insert(p, e.index);
        }
    }
}

void Collision::FindManifolds()
{
    // 
    flip = false;
    Edge e1 = b1->GetCollider()->FindBestEdge(collisionNormal);
    Edge e2 = b2->GetCollider()->FindBestEdge(-collisionNormal);

    // cout << "> find best edge" << endl;

    Edge ref, inc;

    if (abs(e1.GetVector().Dot(collisionNormal) <= abs(e2.GetVector().Dot(collisionNormal))))
    {
        ref = e1;
        inc = e2;
    }
    else
    {
        ref = e2;
        inc = e1;
        flip = true;
        cout << "flipped!" << endl;
    }

    Vector2 normalizedRef = ref.GetVector().Normalise();
    // cout << "> ref a: " << ref.a.x  << ", " << ref.a.y << endl;
    // cout << "> ref b: " << ref.b.x  << ", " << ref.b.y << endl;
    // cout << "> ref edge: " << ref.GetVector().x  << ", " << ref.GetVector().y << endl;
    // cout << "> normalized ref edge: " << normalizedRef.x  << ", " << normalizedRef.y << endl;
    double offset1 = normalizedRef.Dot(ref.a);

    // cout << "> offset1, " << offset1 << endl;

    ClippedPoints cp = Clip(inc.a, inc.b, normalizedRef, offset1);

    if (cp.cPoints.size() < 2)
    {
        // cout << "less than 2 point (1)" << endl;
        return;
    }
    
    double offset2 = normalizedRef.Dot(ref.b);

    // cout << "> offset2, " << offset2 << endl;
    
    ClippedPoints cp2 = Clip(cp.cPoints[0], cp.cPoints[1], -normalizedRef, -offset2);
    if (cp2.cPoints.size() < 2)
    {
        // cout << "> less than 2 point (2)" << endl;
        return;
    }

    // for (int i = 0; i < 2; i++)
    //     cout << "> points: " << cp2.cPoints[i].x << ", " << cp2.cPoints[i].y << endl;

    Vector2 refNormal = ref.GetVector().Cross(-1.0);
    if (flip)
    {
        refNormal = -refNormal;
    }
    double max = refNormal.Dot(ref.farthest);
    // cout << "> max vertex: " << max << endl;

    // cout << "point count_1: " << cp2.cPoints.size() << endl;
    if (refNormal.Dot(cp2.cPoints[0]) - max < 0.0)
    {
        // cout << "> erase 1, dot: " << refNormal.Dot(cp2.cPoints[0]) << endl;
        // cp2.cPoints.erase(cp2.cPoints.begin());

        if (refNormal.Dot(cp2.cPoints[1]) - max < 0.0)
        {
            // PrintVector("ref norm", refNormal);
            // PrintVector("erase 1, 2", cp2.cPoints[1]);
            // PrintScalar("max", max);
            // cout << "point count_2: " << cp2.cPoints.size() << endl;
            cp2.cPoints.erase(cp2.cPoints.begin());
            cp2.cPoints.erase(cp2.cPoints.begin());
        }
        else
        {
            cout << "erase 1: " << cp2.cPoints.size() << endl;
            cp2.cPoints.erase(cp2.cPoints.begin());
        }
    }
    else
    {
        if (refNormal.Dot(cp2.cPoints[1]) - max < 0.0)
        {
            // cout << "> erase 2, " << cp2.cPoints[1].x << endl;
            cout << "erase 2: " << cp2.cPoints.size() << endl;
            cp2.cPoints.erase(cp2.cPoints.begin() + 1);

        }
        
    }

    manifolds = cp2;

    for (int i = 0; i < manifolds.cPoints.size(); i ++)
    {
        // cout << "> manifold: " << manifolds.cPoints[i].x << ", " << manifolds.cPoints[i].y << endl;
    }

}

ClippedPoints Collision::Clip(Vector2 p1, Vector2 p2, Vector2 n, double o)
{
    ClippedPoints cp;
    double d1 = n.Dot(p1) - o;
    double d2 = n.Dot(p2) - o;

    if (d1 > 0.0)
    {
        cp.cPoints.push_back(p1);
    }
    if (d2 > 0.0)
    {
        cp.cPoints.push_back(p2);
    }

    if (d1 * d2 < 0.0)
    {
        Vector2 e = p2 - p1;
        double u = d1 / (d1 - d2);
        e *= u;
        e += p1;
        cp.cPoints.push_back(e);
    }

    return cp;
}

void Collision::Solve()
{
    // PrintVector("before velocity", b2->GetVelocity());
    // PrintVector("normal", b2->GetVelocity());

    SCALAR e = .5;
    SCALAR mu = .02;
    Vector2 relative = b1->GetVelocity() - b2->GetVelocity();
    SCALAR normalVelocity = collisionNormal.Dot(relative);
    SCALAR impulse = (1 + e) * normalVelocity / (collisionNormal.Dot(collisionNormal) * (b1->GetInverseMass() + b2->GetInverseMass()));
    
    // Vector2 relative = b1->GetVelocity() - b2->GetVelocity();
    // SCALAR normalVelocity = collisionNormal.Dot(relative);
    // SCALAR impulse = (1 + e) * normalVelocity / (collisionNormal.Dot(collisionNormal) * (b1->GetInverseMass() + b2->GetInverseMass()));
    Vector2 tangential = collisionNormal.Cross(-1);

    Vector2 norm = collisionNormal.Normalise();
    // PrintVector("norm", norm);

    if (flip)
    {
        // cout << "flipped!!!!!!" << endl;
    }
    
    for (int i = 0; i < manifolds.cPoints.size(); i ++)
    {
        PrintVector("manifold", manifolds.cPoints[i]);
    }
    
    // impulse
    if(manifolds.cPoints.size() > 0)
    {
        if (b1->GetType() != DYNAMIC)
        {
            // 둘 다 dynamic 이 아님.
            if (b2->GetType() != DYNAMIC)
            {
                ;
            }
            // b2 만 dynamic
            else
            {
                PrintScalar("impulse 2", impulse);
                // penetration 이 너무 작지 않으면 그 깊이만큼 튕겨내줌.
                if (penetrationDepth > 0.0001)
                {
                    Vector2 push = collisionNormal * penetrationDepth * 1;
                    b2->SetPosition(b2->GetPosition() + push);
                }

                Vector2 friction = tangential * impulse * mu;
                if (b2->GetVelocity().Dot(tangential) > 0)
                    friction *= -1;
                b2->AddImpulseAt(norm * impulse + friction, manifolds.cPoints[0]);
                b2->SetAngular(b2->GetAngularVelocity() + b2->GetAngularVelocity() * -.1);
                PrintVector("b2 velocity", b2->GetVelocity());
                // // 속도가 너무 작을 때 impulse 를 주지않고 정지시킴.
                // if (b2->GetVelocity().GetLength() < 0.3)
                // {
                //     Vector2 friction = norm.Cross(-1) * impulse * mu;
                //     PrintScalar("no impulse", b2->GetVelocity().GetLength());
                //     PrintScalar("no rot", b2->GetAngularVelocity());
                //     b2->SetVel(b2->GetVelocity() + friction);
                //     // b2->SetAngular(b2->GetAngularVelocity() * .95);
                    
                //     // if (abs(b2->GetAngularVelocity()) < 1)
                //     // {
                //     //     b2->SetAngular(0);
                //     // }

                // }
            }
        }
        else
        {
            // b1 만 dynamic
            if (b2->GetType() != DYNAMIC)
            {
                PrintScalar("impulse 1", impulse);
                // penetration 이 너무 작지 않으면 그 깊이만큼 튕겨내줌.
                if (penetrationDepth > 0.0001)
                {
                    Vector2 push = collisionNormal * penetrationDepth * 1;
                    b1->SetPosition(b1->GetPosition() - push);
                }

                Vector2 friction = tangential * impulse * mu;
                if (b1->GetVelocity().Dot(tangential) > 0)
                    friction *= -1;
                b1->AddImpulseAt(-norm * impulse + friction, manifolds.cPoints[0]);
                b1->SetAngular(b1->GetAngularVelocity() + b1->GetAngularVelocity() * -.1);
                PrintVector("b1 velocity", b1->GetVelocity());
                // 속도가 너무 작을 때 impulse 를 주지않고 정지시킴.
                // if (b1->GetVelocity().GetLength() < 0.3)
                // {
                //     // PrintScalar("no impulse", b2->GetVelocity().GetLength());
                //     // PrintScalar("no rot", b2->GetAngularVelocity());
                //     b1->SetVel(b2->GetVelocity() * .9);
                //     b1->SetAngular(b2->GetAngularVelocity() * .9);
                    
                //     SCALAR tmp = abs(b1->GetVelocity().Dot(collisionNormal.Cross(-1)));
                //     PrintScalar("range", tmp);

                //     if (abs(b1->GetAngularVelocity()) < 1)
                //     {
                //         b1->SetAngular(0);
                //     }

                // }
            }

            // 둘 다 dynamic
            else
            {
                // penetration 이 너무 작지 않으면 그 깊이만큼 튕겨내줌.
                if (penetrationDepth > 0.0001)
                {
                    Vector2 push = collisionNormal * penetrationDepth * 1.0;
                    b1->SetPosition(b1->GetPosition() - push);
                    b2->SetPosition(b2->GetPosition() + push);
                    // PrintVector("impulse", impulse_2);
                }

                b1->AddImpulseAt(-norm * impulse, manifolds.cPoints[0]);
                b2->AddImpulseAt(norm * impulse, manifolds.cPoints[0]);
            }
        }
    }
    else
    {
        // cout << "no manifold" << endl;
    }
    // PrintVector("after velocity", b2->GetVelocity());

}