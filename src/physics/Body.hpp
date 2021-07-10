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
    double radius;

    Collider* collider;

public:
    Body(POLY_DATA vertices, Vector2 pos, SCALAR rot, int i, SCALAR density, BodyType t);

    // getter
    Vector2 GetPosition();
    SCALAR GetRotation();
    Vector2 GetVelocity();
    SCALAR GetAngularVelocity();
    SCALAR GetMass();
    SCALAR GetInertia();
    SCALAR GetInverseMass();
    SCALAR GetInverseInertia();
    BodyType GetType();
    Collider* GetCollider();

    // setter
    void SetPosition(Vector2);
    void SetRotation(SCALAR);

    void AddForce(Vector2);
    void AddTorque(SCALAR);
    void AddImpulseAt(Vector2, Vector2);
    void AddJointImpulse(Vector2 v, SCALAR a);
    void CalculateVelocity();
    void CalculateAngularVelocity();
    void CalculatePosition();
    void CalculateAngle();
    void ClearForce();
    void ClearTorque();
    void UpdateCentroid();
    
    void SetVel(Vector2);
    void SetAngular(SCALAR);
};


# endif