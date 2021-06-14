# include "Collider.hpp"
// # include "Body.hpp"
# include <iostream>

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