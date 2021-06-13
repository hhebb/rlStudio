# ifndef COLLIDER
# define COLLIDER

# include "../namespace.hpp"

class Body;

class Collider
{
private:
    Body* body;
    VERTEX vertices;
    float radius;
    Matrix3x3 matrix;
    
public:
    Collider(Body*, VERTEX, float);
    void Update(Vector2, Vector2, SCALAR);
    VERTEX GetVertices();
};

# endif