# ifndef COLLIDER
# define COLLIDER

# include "../Definition.hpp"
# include "../Helper.hpp"

// class Body;

class Collider
{
private:
    POLY_DATA vertices;
    HomogeneousMatrix3x3 matrix;
    float radius;

    // attr
    SCALAR area = 0;
    Vector2 centroid;
    
public:
    Collider(POLY_DATA, Vector2, float);
    void Update(Vector2, Vector2, SCALAR);

    // collider attribute. mass, inertia are called by body.
    SCALAR CalculateArea();
    Vector2 CalculateCentroid();
    void InitVertices(Vector2);
    SCALAR CalculateMass(SCALAR);
    SCALAR CalculateInertia();

    // getter
    POLY_DATA GetVertices();
    POLY_DATA GetLocalVertices();
    Vector2 GetCentroid(); // same as body's position.
    Vector2 GetCenter(); // deprecated??
    
    // collision info
    Edge FindBestEdge(Vector2);
};

# endif