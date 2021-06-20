# ifndef WORLD
# define WORLD

# include <QThread>
# include <QVariant>
# include "Body.hpp"
# include "Collider.hpp"
# include "../Helper.hpp"

class World: public QThread
{
    Q_OBJECT

signals:
    void physicsUpdate(QVariant);
private:
    // PhysicalObjects objs
    vector<Body> bodies;
    VERTEX_LIST vertices;
    void run();

public:
    bool ready;
    World();
    void Init();
    void Reset();
    void Step();
    VERTEX_LIST GetVertices();
    void Create(VERTEX, Vector2, SCALAR, SCALAR, float);
    void Debug();
    vector<Body> GetBodies();
    bool IsCollide(Body*, Body*);
    // void GetCollisionInfo(Simplex, Body*, Body*);
    
};

# endif