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
    Matrix3x3 matrix;
    MassData massData;
    float radius;
    float area = 0;
    
public:
    Collider(VERTEX, float);
    void Update(Vector2, Vector2, SCALAR);
    MassData GetMassData();
    VERTEX GetVertices();
    Vector2 GetCenter();
    Edge FindBestEdge(Vector2);
    void CalculateArea();
    void CalculateCenter();
};

# endif