# ifndef COLLIDER
# define COLLIDER

# include "../Definition.hpp"
# include "../Helper.hpp"

// class Body;

class Collider
{
private:
    // Body* body;
    VERTEX vertices;
    float radius;
    Matrix3x3 matrix;
    
public:
    Collider(VERTEX, float);
    void Update(Vector2, Vector2, SCALAR);
    VERTEX GetVertices();
    Vector2 GetCenter();
    Edge FindBestEdge(Vector2);
};

# endif