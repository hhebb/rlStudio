# ifndef COLLIDER
# define COLLIDER

# include "../namespace.hpp"
# include "Body.hpp"

class Body; // ??? 없으면 참조 에러남.

class Collider
{
private:
    Body* body;
    VERTEX vertices;
    float radius;
    
public:
    Collider(Body*, VERTEX, float);
    void Update(Vector2, SCALAR);
};

//
Collider::Collider(Body* body, VERTEX vert, float rad)
{
    this->body = body;
    vertices = vert;
    radius = rad;
}

void Collider::Update(Vector2 translate, SCALAR rotate)
{
    
}

# endif