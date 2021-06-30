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
    double radius;

    // attr
    SCALAR area = 0;
    SCALAR area_init = 0;
    double area_ratio = 0;
    Vector2 centroid{0, 0};
    
public:
    Collider(POLY_DATA vert, Vector2 pos, SCALAR rot, double rad);
    void Update(Vector2, Vector2, SCALAR);

    // collider attribute. mass, inertia are called by body.
    void CalculateArea();
    Vector2 CalculateCentroid();
    void InitVertices(Vector2 origin, SCALAR rot);
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