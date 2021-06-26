# ifndef BODY
# define BODY

# include "../Definition.hpp"
# include "Collider.hpp"

class Body
{
private:
    int id; // 초기화 시 추가
    BodyType type = DYNAMIC; // 초기화 시 추가

    Vector2 position; // = centroid
    SCALAR rotation;
    Vector2 velocity = {0.0f, 0.0f};
    SCALAR angularVelocity;
    Vector2 force = {0, 0};
    SCALAR torque = 0;

    // mass data
    SCALAR mass;
    SCALAR inverseMass;
    SCALAR inertia;
    SCALAR inverseInertia;
    float radius;

    Collider* collider;

public:
    Body(POLY_DATA, Vector2, SCALAR, float, int, SCALAR);

    // getter
    Vector2 GetPosition();
    SCALAR GetRotation();
    Vector2 GetVelocity();
    SCALAR GetAngularVelocity();
    SCALAR GetMass();
    SCALAR GetInertia();
    Collider* GetCollider();

    void AddForce(Vector2);
    void AddTorque(SCALAR);
    void AddImpulseAt(Vector2, Vector2);
    void CalculateVelocity();
    void CalculateAngularVelocity();
    void CalculatePosition();
    void CalculateAngle();
    void ClearForce();
    void ClearTorque();
    
};


# endif