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
            PrintVector("ref norm", refNormal);
            PrintVector("erase 1, 2", cp2.cPoints[1]);
            PrintScalar("max", max);
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
    Vector2 norm = collisionNormal.Normalise();
    Vector2 impulse_1 = b1->GetVelocity() - norm * (b1->GetVelocity().Dot(norm)) * 2; // 각 body 에 추가해줄 속도 방향.
    Vector2 impulse_2 = b2->GetVelocity() - norm * (b2->GetVelocity().Dot(norm)) * 2; // 각 body 에 추가해줄 속도 방향.
    impulse_1 *= b1->GetMass() * (1 + e);
    impulse_2 *= b2->GetMass() * (1 + e);

    // Vector2 impulse_1 = b1->GetVelocity() * b1->GetMass() * e;
    // Vector2 impulse_2 = -b2->GetVelocity() * b2->GetMass() * e;

    if (flip)
    {
        Vector2 tmp = impulse_1;
        impulse_1 = impulse_2;
        impulse_2 = tmp;
    }
    // Vector2 relative = b2->GetVelocity() - b1->GetVelocity(); // 상대가 고정되었다고 본 자신의 상대속도.
    // if (flip) {relative *= -1;}
    // b1->velocity = b1->velocity - collisionNormal;// * DELTA_TIME; // add velocity 할 수 있도록 메서드 추가하기
    // b2->velocity = b2->velocity + collisionNormal;// * DELTATIME; // add velocity 할 수 있도록 메서드 추가하기
    
    for (int i = 0; i < manifolds.cPoints.size(); i ++)
    {
        PrintVector("manifold", manifolds.cPoints[i]);
    }
    
    if(manifolds.cPoints.size() > 0)
    {
        if (b1->GetType() != DYNAMIC)
        {
            if (b2->GetType() != DYNAMIC)
            {
                ;
            }
            else
            {
                // penetration 이 너무 작지 않으면 그 깊이만큼 튕겨내줌.
                if (penetrationDepth > 0.005)
                {
                    Vector2 push = collisionNormal * penetrationDepth;
                    b2->SetPosition(b2->GetPosition() + push);
                    // b2->GetCollider()->Update(b2->GetPosition(), push, 0);
                    // PrintVector("impulse", impulse_2);

                }



                // 속도가 너무 작을 때 impulse 를 주지않고 정지시킴.
                if (b2->GetVelocity().GetLength() < 0.1)
                {
                    PrintScalar("no impulse", b2->GetVelocity().GetLength());
                    PrintScalar("no rot", b2->GetAngularVelocity());
                    b2->SetVel(Vector2{0, 0});

                    if (abs(b2->GetAngularVelocity()) <2)
                    {
                        b2->SetAngular(0);
                    }

                    if (manifolds.cPoints.size() == 2)
                    {
                        b2->SetAngular(0);
                    }
                }
                else
                {
                    PrintVector("impulse", impulse_2);
                    // manifold 갯수에 따라 impulse 주는 포인트 다시 계산.
                    if (manifolds.cPoints.size() == 2)
                    {
                        Vector2 point = (manifolds.cPoints[0] + manifolds.cPoints[1]) / 2;
                        b2->AddImpulseAt(impulse_2, point);
                    }
                    else
                    {
                        b2->AddImpulseAt(impulse_2, manifolds.cPoints[0]);
                    }

                }

            }
        }
        else
        {
            if (b2->GetType() != DYNAMIC)
            {
                b1->AddImpulseAt(impulse_1, manifolds.cPoints[0]);
            }
            else
            {
                b1->AddImpulseAt(impulse_1, manifolds.cPoints[0]);
                b2->AddImpulseAt(impulse_2, manifolds.cPoints[0]);
            }
        }


    }
    else
    {
        // cout << "no manifold" << endl;
    }
    PrintVector("after velocity", b2->GetVelocity());

}