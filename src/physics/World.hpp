# ifndef WORLD
# define WORLD

# include <QThread>
# include <QVariant>
# include "Body.hpp"
# include "Collider.hpp"
# include "Collision.hpp"
# include "../Helper.hpp"

class World: public QThread
{
    Q_OBJECT

signals:
    void physicsUpdate(QVariant);
private:
    vector<Body> bodies;
    POLY_LIST vertices;
    vector<Collision*> collisionList; // collision manager 만들어서 관리하도록 수정해야함.
    void run();

public:
    bool ready;
    World();
    void Init();
    void Reset();
    void Step();
    POLY_LIST GetVertices();
    void Create(POLY_DATA, Vector2, SCALAR, SCALAR, float, int);
    void Debug();
    vector<Body> GetBodies();
    bool IsCollide(Body*, Body*);
    
};

# endif