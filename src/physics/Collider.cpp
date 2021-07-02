# include "Collider.hpp"
// # include "Body.hpp"
# include <iostream>
# include <limits>
# include <cmath>

using namespace std;

Collider::Collider(POLY_DATA vert, Vector2 pos, SCALAR rot, double rad)
{
    vertices = vert;
    radius = rad; // deprecated
    CalculateArea();
    CalculateCentroid();
    InitVertices(pos, rot); // body 의 position 기준으로 centroid 를 옮김.

}

void Collider::Update(Vector2 center,Vector2 translate, SCALAR rotate)
{
    matrix.SetIdentity();
    matrix.Translate(translate);
    matrix.Rotate(rotate, center);
    vertices = matrix.Multiply(vertices);

    // cout << vertices[1].x << ", " << vertices[1].y << endl;

    for (int i = 0; i < vertices.size(); i ++)
    {
        // PrintVector("v", vertices[i]);
    }
    // cout << endl;

    // centroid update!
    // CalculateArea();
    
    
    // PrintScalar("ratio", area_ratio);
    CalculateCentroid();
    // PrintVector("centroid updated", centroid);
}

void Collider::CalculateArea()
{
    area_init = 0;
    for (int i = 0; i < vertices.size() - 1; i ++)
    {
        // area += vertices[i].x * vertices[i + 1].y - vertices[i + 1].x * vertices[i].y;
        area_init += vertices[i].Cross(vertices[i + 1]);
    }
    area_init += vertices[vertices.size() - 1].Cross(vertices[0]);

    area_init /= 2;
    // PrintScalar("initial area", area_init);
}

Vector2 Collider::CalculateCentroid()
{
    centroid = Vector2{0, 0};
    area = 0;
    double area_tmp = 0;
    for (int i = 0; i < vertices.size() - 1; i ++)
    {
        // similar to cross product.
        // PrintVector("v", vertices[i]);
        area_tmp = vertices[i].Cross(vertices[i + 1]);
        area += area_tmp;
        centroid += (vertices[i] + vertices[i + 1]) * area_tmp;
        // PrintVector("centroid tmp", centroid);
    }
    area_tmp = vertices[vertices.size() - 1].Cross(vertices[0]);
    area += area_tmp;
    centroid += (vertices[vertices.size() - 1] + vertices[0]) * area_tmp;
        
    //
    area *= .5;
    area_ratio = area / area_init;
    centroid /= (6 * area);
    // PrintScalar("area update", area);
    // PrintVector("centroid updated", centroid);

    return centroid;
}

void Collider::InitVertices(Vector2 origin, SCALAR rot)
{
    // PrintVector("centroid", centroid);

    Vector2 diff = origin - centroid;
    for (int i = 0; i < vertices.size(); i ++)
    {
        vertices[i] += diff;
        // PrintVector("v", vertices[i]);
    }

    // rotation setting.
    matrix.SetIdentity();
    matrix.Rotate(rot, origin);
    vertices = matrix.Multiply(vertices);
}

SCALAR Collider::CalculateMass(SCALAR density)
{
    SCALAR mass = density * area;
    return mass;
}

SCALAR Collider::CalculateInertia(Vector2 center)
{
    SCALAR inertia = 0;
    SCALAR mass_tri;
    SCALAR inertia_tri;
    Vector2 a, b;
    for (int i = 0; i < vertices.size() - 1; i ++)
    {
        a = vertices[i] - center;
        b = vertices[i + 1] - center;
        mass_tri = abs(a.Cross(b) * .5);
        inertia_tri = mass_tri * (a.GetSqrLength() + b.GetSqrLength() + a.Dot(b)) / 6;
        inertia += inertia_tri;
    }
    int idx = vertices.size() - 1;
    a = vertices[idx] - center;
    b = vertices[0] - center;
    mass_tri = abs(a.Cross(b) * .5);
    inertia_tri = mass_tri * (a.GetSqrLength() + b.GetSqrLength() + a.Dot(b)) / 6;
    inertia += inertia_tri;
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
    double x = 0;
    double y = 0;
    
    for (int i = 0; i < vertices.size(); i ++)
    {
        x += vertices[i].x;
        y += vertices[i].y;
    }

    x /= vertices.size();
    y /= vertices.size();

    return Vector2{x, y};
}

void Collider::SetPosition(Vector2 pos)
{
    Vector2 diff = pos - centroid;
    for (int i = 0; i < vertices.size(); i ++)
    {
        vertices[i] += diff;
    }
}

void Collider::SetRotation(SCALAR rot)
{
    // SCALAR diff = rot - ;
    SCALAR radAngle = rot * INVERSE_RADIAN;
    double c = cos(radAngle);
    double s = sin(radAngle);
    for (int i = 0; i < vertices.size(); i ++)
    {
        vertices[i].x = (vertices[i].x - centroid.x) * c - (vertices[i].y - centroid.y) * s + centroid.x;
        vertices[i].y = (vertices[i].x - centroid.x) * s + (vertices[i].y - centroid.y) * c + centroid.y;
    }
}

Edge Collider::FindBestEdge(Vector2 normal)
{
    // return best match to normal vector's perpendicular vector
    double max = -numeric_limits<double>::max();
    int index = 0;
    for (int i = 0; i < vertices.size(); i ++)
    {
        double proj = normal.Dot(vertices[i]);
        // cout << "> projection: " << proj << ", verts: " << vertices.size() << endl;
        if (proj > max)
        {
            max = proj;
            index = i;
        }
    }

    Vector2 v = vertices[index];
    int leftIndex = index + 1 >= vertices.size() ? 0 : index + 1;
    int rightIndex = index <= 0 ? vertices.size() - 1 : index - 1;

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