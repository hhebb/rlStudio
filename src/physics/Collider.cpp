# include "Collider.hpp"
# include "Body.hpp"
# include <QMatrix2x2>
# include <QMatrix>
# include <iostream>

using namespace std;

Collider::Collider(Body* body, VERTEX vert, float rad)
{
    this->body = body;
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
    matrix.SetCenter(center);
    matrix.Translate(translate);
    matrix.Rotate(rotate);
    vertices = matrix.Multiply(vertices);
    // cout << vertices[1].x << ", " << vertices[1].y << endl;

    // for (int i = 0; vertices.size(); i ++)
    // {
    //     cout << vertices[i].x << ", " << vertices[i].y << ", ";
    // }
    // cout << endl;
}