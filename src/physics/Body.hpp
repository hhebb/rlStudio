# ifndef BODY
# define BODY

# include "../Definition.hpp"
# include "Collider.hpp"

class Body
{
private:
    int index; // 초기화 시 추가
    BodyType type = DYNAMIC; // 초기화 시 추가
    Vector2 position;
    SCALAR rotation;
    SCALAR angularVelocity;

    Vector2 force = {0, 0};
    SCALAR torque = 0;
    SCALAR inverseMass;
    SCALAR moment;
    float radius;

    Collider* collider;

public:
    SCALAR mass;
    Vector2 velocity;
    Body(VERTEX, Vector2, SCALAR, SCALAR, float); // index 추가하기
    void AddForce(Vector2);
    void AddTorque(SCALAR);
    void ClearForce();
    void ClearTorque();
    void CalculateVelocity();
    void CalculateAngularVelocity();
    void CalculatePosition();
    void CalculateAngle();

    Collider* GetCollider();
    Vector2 GetVelocity();
    SCALAR GetAngularVelocity();
    Vector2 GetPosition();
    SCALAR GetRotation();

};



# endif