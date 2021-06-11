# ifndef COLLIDER
# define COLLIDER

# include "../namespace.hpp"
# include "Body.hpp"

class Collider
{
private:
    // Body* body;
    VERTEX vertices;
    float radius;
    
public:
    // Collider(Body*, VERTEX, float);
    void Update(Vector2, SCALAR);
};


# endif