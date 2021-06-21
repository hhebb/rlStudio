# include "Collision.hpp"
# include <cmath>

void Collision::FindCollisioninfo(Simplex simplex)
{
    // EPA algorithm for find collision normal, penetration depth.
    while (true)
    {
        Edge e = FindClosetEdge(simplex);
        Vector2 p = SupportFunction(b1->GetCollider()->GetVertices(), b2->GetCollider()->GetVertices(), e.normal);
        float d = p.Dot(e.normal);
        if (d - e.distance < TOLERANCE)
        {
            collisionNormal = e.normal;
            penetrationDepth = d;
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
    float offset1 = normalizedRef.Dot(ref.a);

    ClippedPoints cp = Clip(inc.a, inc.b, normalizedRef, offset1);

    if (cp.cPoints.size() < 2)
    {
        return;
    }
    
    float offset2 = normalizedRef.Dot(ref.b);
    ClippedPoints cp2 = Clip(cp.cPoints[0], cp.cPoints[1], -normalizedRef, -offset2);
    if (cp2.cPoints.size() < 2)
    {
        return;
    }

    Vector2 refNormal = ref.GetVector().Cross(-1.0);
    if (flip)
    {
        refNormal = -refNormal;
    }
    float max = refNormal.Dot(ref.farthest);
    if (refNormal.Dot(cp2.cPoints[0]) - max < 0.0)
    {
        cp2.cPoints.erase(cp2.cPoints.begin());
    }
    if (refNormal.Dot(cp2.cPoints[1]) - max < 0.0)
    {
        cp2.cPoints.erase(cp2.cPoints.begin() + 1);
    }

    manifolds = cp2;
}

ClippedPoints Collision::Clip(Vector2 p1, Vector2 p2, Vector2 n, float o)
{
    ClippedPoints cp;
    float d1 = n.Dot(p1) - o;
    float d2 = n.Dot(p2) - o;

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
        float u = d1 / (d1 - d2);
        e = e * u;
        e = e + p1;
        cp.cPoints.push_back(e);
    }

    return cp;
}