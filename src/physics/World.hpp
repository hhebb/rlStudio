# ifndef WORLD
# define WORLD

# include <QThread>
# include <QVariant>
# include "Body.hpp"
# include "Collider.hpp"
# include "Collision.hpp"
# include "../Helper.hpp"
# include "Joint.hpp"
# include "RevoluteJoint.hpp"
# include "DistanceJoint.hpp"

class World: public QThread
{
    Q_OBJECT

signals:
    void physicsUpdate(QVariant);
private:
    vector<Body> bodies;
    POLY_LIST vertices;
    vector<Collision*> collisionList; // collision manager 만들어서 관리하도록 수정해야함.
    vector<Joint*> jointList; // manager 만들어서 관리하도록 수정해야함.
    void run();

public:
    bool ready;
    World();
    void Init();
    void Reset();
    void Step();
    POLY_LIST GetVertices();
    void Create(POLY_DATA ver, Vector2 pos, SCALAR rot, int id, BodyType t);
    void Debug();
    vector<Body> GetBodies();
    bool IsCollide(Body*, Body*);
    
};

# endif