# ifndef HELPER
# define HELPER

# include "Definition.hpp"

Vector2 SupportFunction(VERTEX verts1, VERTEX verts2, Vector2 d)
{
    Vector2 a = GetFarthestVertex(verts1, d);
    Vector2 b = GetFarthestVertex(verts2, -d);
    
    return a - b;
}

Vector2 GetFarthestVertex(VERTEX verts, Vector2 d)
{
    float max = -1;
    int maxIdx = -1;
    for (int i = 0; i < verts.size(); i ++)
    {
        float dist = verts[i].Dot(d);
        if (dist > max)
        {
            max = dist;
            maxIdx = i;
        }
    }

    return verts[maxIdx];
}

Vector2 TripleProduct(Vector2 a, Vector2 b, Vector2 c)
{
    return b * c.Dot(a) - a * c.Dot(b);
}

bool IsContainOrigin(Simplex simplex, Vector2& d)
{
    Vector2 a = simplex.GetLastElement();
    
    if (simplex.elements.size() == 3)
    {
        Vector2 b = simplex.GetB();
        Vector2 c = simplex.GetC();

        Vector2 ab = b - a;
        Vector2 ac = c - a;

        Vector2 abPerp = TripleProduct(ac, ab, ab);
        Vector2 acPerp = TripleProduct(ab, ac, ac;

        if (abPerp.Dot(-a) > 0)
        {
            simplex.Remove(2);
            d.Set(abPerp);
        }
        else
        {
            if (acPerp.dot(-a) > 0)
            {
                simplex.Remove(1);
                d.Set(acPerp);
            }
            else
            {
                // already contain origin. dont need to change direction
                return true;
            }
        }

    }
    else if (simplex.elements.size() == 2)
    {
        Vector2 b = simplex.GetB();
        Vector2 abPerp = TripleProduct(ab, -a, ab);
        d.Set(abPerp);
    }

    return false;
}

# endif