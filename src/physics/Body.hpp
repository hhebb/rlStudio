# ifndef BODY
# define BODY

# include "../namespace.hpp"
# include "Collider.hpp"

class Body
{
private:
    Vector2 position;
    SCALAR rotation;
    Vector2 velocity = {.0f, .0f};
    SCALAR angularVelocity;

    Vector2 force = {.0f, .0f};
    SCALAR torque;
    SCALAR inverseMass;
    float radius;

    Collider* collider;

public:
    SCALAR mass;
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