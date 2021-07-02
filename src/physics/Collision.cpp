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
            cout << "> erase 1, 2, dot: " << refNormal.Dot(cp2.cPoints[1]) << cp2.cPoints[1].x << endl;
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

    SCALAR e = 1;
    Vector2 norm = collisionNormal.Normalise();
    Vector2 relative = b2->GetVelocity() - b1->GetVelocity(); // 상대가 고정되었다고 본 자신의 상대속도.
    if (flip) {relative *= -1;}
    Vector2 reflect = relative - norm * (relative.Dot(norm)) * 2 * 1; // 각 body 에 추가해줄 속도 방향.
    // b1->velocity = b1->velocity - collisionNormal;// * DELTA_TIME; // add velocity 할 수 있도록 메서드 추가하기
    // b2->velocity = b2->velocity + collisionNormal;// * DELTA_TIME; // add velocity 할 수 있도록 메서드 추가하기
    
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
                // (b2->GetVelocity() + reflect) .Normalise() * reflect.GetLength();
                b2->AddImpulseAt(reflect, manifolds.cPoints[0]);
            }
        }
        else
        {
            if (b2->GetType() != DYNAMIC)
            {
                b1->AddImpulseAt(-reflect, manifolds.cPoints[0]);
            }
            else
            {
                b1->AddImpulseAt(-reflect, manifolds.cPoints[0]);
                b2->AddImpulseAt(reflect, manifolds.cPoints[0]);
            }
        }


    }
    // PrintVector("after velocity", b2->GetVelocity());

}