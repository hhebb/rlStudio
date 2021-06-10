# ifndef WORLD
# define WORLD

# include "Body.hpp"
# include "Collider.hpp"

class World
{
private:
    // PhysicalObjects objs
    vector<Body> bodies;
    VERTEX vertices;
public:
    World();
    void Init();
    void Reset();
    void Step();
    VERTEX GetVertices();
    void Create(VERTEX, Vector2, SCALAR, SCALAR, float);
    void Debug();
    
};

# endif