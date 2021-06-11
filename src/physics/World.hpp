# ifndef WORLD
# define WORLD

# include "Body.hpp"
# include "Collider.hpp"
# include <QThread>
# include <QVariant>

class World: public QThread
{
    Q_OBJECT

signals:
    void physicsUpdate(QVariant);
private:
    // PhysicalObjects objs
    vector<Body> bodies;
    VERTEX vertices;
    void run();

public:
    bool ready;
    World();
    void Init();
    void Reset();
    void Step();
    VERTEX GetVertices();
    void Create(VERTEX, Vector2, SCALAR, SCALAR, float);
    void Debug();
    vector<Body> GetBodies();
    
};

# endif