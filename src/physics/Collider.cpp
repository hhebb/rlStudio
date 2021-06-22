# include "Collider.hpp"
// # include "Body.hpp"
# include <iostream>
# include <limits>

using namespace std;

Collider::Collider(VERTEX vert, float rad)
{
    // this->body = body;
    vertices = vert;
    radius = rad;
}

VERTEX Collider::GetVertices()
{
    return vertices;
}

void Collider::Update(Vector2 center,Vector2 translate, SCALAR rotate)
{
    matrix.SetIdentity();
    matrix.Translate(translate);
    matrix.Rotate(rotate, center);
    vertices = matrix.Multiply(vertices);
    // cout << vertices[1].x << ", " << vertices[1].y << endl;

    // for (int i = 0; vertices.size(); i ++)
    // {
    //     cout << vertices[i].x << ", " << vertices[i].y << ", ";
    // }
    // cout << endl;
}

Vector2 Collider::GetCenter()
{
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
    int rightIndex = index + 1 >= vertices.size() ? index + 1 : 0;
    int leftIndex = index <= 0 ? vertices.size() - 1 : index - 1;

    cout << "> index test: " << vertices.size() << ", " << index << endl;
    Vector2 v1 = vertices[rightIndex]; // index + 1
    Vector2 v0 = vertices[leftIndex]; // index - 1. 마지막 인덱스일 때 0 으로 처리해줘야함.

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