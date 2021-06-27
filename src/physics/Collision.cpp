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
    bool flip = false;
    Edge e1 = b1->GetCollider()->FindBestEdge(collisionNormal);
    Edge e2 = b2->GetCollider()->FindBestEdge(-collisionNormal);

    // cout << "> find best edge" << endl;

    Edge ref;
    Edge inc;

    if (abs(e1.GetVector().Dot(collisionNormal) <= abs(e1.GetVector().Dot(collisionNormal))))
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
        return;
    }
    
    double offset2 = normalizedRef.Dot(ref.b);

    // cout << "> offset2, " << offset2 << endl;
    
    ClippedPoints cp2 = Clip(cp.cPoints[0], cp.cPoints[1], -normalizedRef, -offset2);
    if (cp2.cPoints.size() < 2)
    {
        cout << "> less than 2 cp" << endl;
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

    if (refNormal.Dot(cp2.cPoints[0]) - max < 0.0)
    {
        // cout << "> erase 1, dot: " << refNormal.Dot(cp2.cPoints[0]) << endl;
        // cp2.cPoints.erase(cp2.cPoints.begin());

        if (refNormal.Dot(cp2.cPoints[1]) - max >= 0.0)
        {
            // cout << "> erase 2, dot: " << refNormal.Dot(cp2.cPoints[1]) << cp2.cPoints[1].x << endl;
            cp2.cPoints.erase(cp2.cPoints.begin());
            cp2.cPoints.erase(cp2.cPoints.begin());

        }
        else
        {
            cp2.cPoints.erase(cp2.cPoints.begin());
        }
    }
    else
    {
        if (refNormal.Dot(cp2.cPoints[1]) - max >= 0.0)
        {
            // cout << "> erase 2, " << cp2.cPoints[1].x << endl;
            cp2.cPoints.erase(cp2.cPoints.begin() + 1);

        }
        
    }

    manifolds = cp2;

    for (int i = 0; i < manifolds.cPoints.size(); i ++)
    {
        cout << "> manifold: " << manifolds.cPoints[i].x << ", " << manifolds.cPoints[i].y << endl;
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
        e = e * u;
        e = e + p1;
        cp.cPoints.push_back(e);
    }

    return cp;
}

void Collision::Solve()
{
    cout << "> before velocity: " << b1->GetVelocity().x << ", " << b1->GetVelocity().y << endl;
    cout << "> normal: " << collisionNormal.x << ", " << collisionNormal.y << endl;
    // b1->velocity = b1->velocity - collisionNormal;// * DELTA_TIME; // add velocity 할 수 있도록 메서드 추가하기
    // b2->velocity = b2->velocity + collisionNormal;// * DELTA_TIME; // add velocity 할 수 있도록 메서드 추가하기
    b1->AddImpulseAt(-collisionNormal, manifolds.cPoints[0]);
    b2->AddImpulseAt(collisionNormal, manifolds.cPoints[0]);
    cout << "> after velocity: " << b2->GetVelocity().x << ", " << b2->GetVelocity().y << endl;

}