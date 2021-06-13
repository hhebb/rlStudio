# ifndef BODY
# define BODY

# include "../namespace.hpp"
# include "Collider.hpp"

class Body
{
private:
    Vector2 position;
    SCALAR rotation;
    SCALAR angularVelocity;

    Vector2 force = {0, 0};
    SCALAR torque;
    SCALAR inverseMass;
    float radius;

    Collider* collider;

public:
    SCALAR mass;
    Vector2 velocity;
    Body(VERTEX, Vector2, SCALAR, SCALAR, float);
    void AddForce(Vector2);
    void ClearForce();
    void CalculateVelocity();
    void CalculatePosition();

    Collider* GetCollider();
    Vector2 GetPosition();
    Vector2 GetVelocity();
    SCALAR GetRotation();

};



# endif