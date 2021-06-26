# include "Collider.hpp"
// # include "Body.hpp"
# include <iostream>
# include <limits>

using namespace std;

Collider::Collider(POLY_DATA vert, Vector2 pos, float rad)
{
    vertices = vert;
    radius = rad; // deprecated
    CalculateArea();
    CalculateCentroid();
    InitVertices(pos); // body 의 position 기준으로 centroid 를 옮김.

}

void Collider::Update(Vector2 center,Vector2 translate, SCALAR rotate)
{
    matrix.SetIdentity();
    matrix.Translate(translate);
    matrix.Rotate(rotate, center);
    vertices = matrix.Multiply(vertices);
    // CalculateCenter();
    // cout << vertices[1].x << ", " << vertices[1].y << endl;

    // for (int i = 0; vertices.size(); i ++)
    // {
    //     cout << vertices[i].x << ", " << vertices[i].y << ", ";
    // }
    // cout << endl;
}

SCALAR Collider::CalculateArea()
{
    for (int i = 0; i < vertices.size() - 1; i ++)
    {
        // area += vertices[i].x * vertices[i + 1].y - vertices[i + 1].x * vertices[i].y;
        area += vertices[i].Cross(vertices[i + 1]);
    }

    area /= 2;
    return area;
}

Vector2 Collider::CalculateCentroid()
{
    for (int i = 0; i < vertices.size() - 1; i ++)
    {
        // similar to cross product.
        // center.x += (vertices[i].x + vertices[i + 1].x) * (vertices[i].x * vertices[i + 1].y - vertices[i + 1].x * vertices[i].y);
        // center.y += (vertices[i].y + vertices[i + 1].y) * (vertices[i].x * vertices[i + 1].y - vertices[i + 1].x * vertices[i].y);
        
        // center.x += (vertices[i].x + vertices[i + 1].x) * (vertices[i].Cross(vertices[i + 1]));
        // center.y += (vertices[i].y + vertices[i + 1].y) * (vertices[i].Cross(vertices[i + 1]));

        centroid += (vertices[i] + vertices[i + 1]) * vertices[i].Cross(vertices[i + 1]);
    }

    // center = center / (6 * area);
    centroid /= (6 * area);

    return centroid;
}

void Collider::InitVertices(Vector2 origin)
{
    Vector2 diff = origin - centroid;
    for (int i = 0; i < vertices.size(); i ++)
    {
        vertices[i] -= diff;
    }
}

SCALAR Collider::CalculateMass(SCALAR density)
{
    SCALAR mass = density * area;
    return mass;
}

SCALAR Collider::CalculateInertia()
{
    SCALAR inertia;
    for (int i = 0; i < vertices.size() - 1; i ++)
    {
        SCALAR mass_tri = vertices[i].Cross(vertices[i + 1]) * .5f;
        SCALAR inertia_tri = mass_tri * (vertices[i].GetSqrLength() + vertices[i + 1].GetSqrLength() + vertices[i].Dot(vertices[i + 1])) / 6;
        inertia += inertia_tri;
    }

    return inertia;
}

POLY_DATA Collider::GetVertices()
{
    return vertices;
}

POLY_DATA Collider::GetLocalVertices()
{
    POLY_DATA local;
    for (int i = 0; i< vertices.size(); i ++)
    {
        Vector2 v = vertices[i] -= centroid;
        local.push_back(v);
    }

    return local;
}

Vector2 Collider::GetCentroid()
{
    return centroid;
}

Vector2 Collider::GetCenter()
{
    // deprecated!
    float x = 0;
    float y = 0;
    
    for (int i = 0; i < vertices.size(); i ++)
    {
        x += vertices[i].x;
        y += vertices[i].y;
    }

    x /= vertices.size();
    y /= vertices.size();

    return Vector2{x, y};
}

Edge Collider::FindBestEdge(Vector2 normal)
{
    // return best match to normal vector's perpendicular vector
    float max = -numeric_limits<float>::max();
    int index = 0;
    for (int i = 0; i < vertices.size(); i ++)
    {
        float proj = normal.Dot(vertices[i]);
        // cout << "> projection: " << proj << ", verts: " << vertices.size() << endl;
        if (proj > max)
        {
            max = proj;
            index = i;
        }
    }

    Vector2 v = vertices[index];
    int rightIndex = index + 1 >= vertices.size() ? 0 : index + 1;
    int leftIndex = index <= 0 ? vertices.size() - 1 : index - 1;

    // cout << "> index test: " << vertices.size() << ", " << index << ", " << rightIndex << ", " << leftIndex << endl;
    Vector2 v1 = vertices[rightIndex]; // index + 1
    Vector2 v0 = vertices[leftIndex]; // index - 1

    Vector2 left = v - v1;
    Vector2 right = v - v0;
    left = left.Normalise();
    right = right.Normalise();

    if (right.Dot(normal) <= left.Dot(normal))
    {
        return Edge{v0, v, v, Vector2{0, 0}, Vector2{0, 0}, 0, 0};
    }
    else
    {
        return Edge{v, v1, v, Vector2{0, 0}, Vector2{0, 0}, 0, 0};
    }
}