# include "World.hpp"
# include <iostream>
using namespace std;

World::World()
{
    // vertices;
    // Init();
    // Reset();
}

void World::Init()
{
    // body 들 생성, 물성치 설정. 모든 물리 환경 설정.
    VERTEX vert; // vertices
    Vector2 pos; // center
    SCALAR rot;
    SCALAR mass;
    float radius;

    // 바닥
    pos = {-0.4, -0.1};
    vert = {Vector2{pos.x -.4f, pos.y}, Vector2{pos.x + .4f, pos.y}, Vector2{pos.x + .4f, pos.y + .2f}, Vector2{pos.x -.4f, pos.y + .2f}}; // x, y order
    rot = 0;
    mass = 0;
    radius = 0;
    Create(vert, pos, rot, mass, radius);


    // 예시 body
    pos = {-0.2, 0.205};
    vert = {Vector2{pos.x, pos.y}, Vector2{pos.x + .1f, pos.y}, Vector2{pos.x + .1f, pos.y + .1f}}; // x, y order
    rot = 0;
    mass = 0;
    radius = 1;
    Create(vert, pos, rot, mass, radius);

}

void World::Reset()
{
    // 초기 물리 환경 설정으로 초기화.
}

void World::Step()
{
    // obj 들 update, vertices update
    for (int i = 0; i < bodies.size(); i ++)
    {   
        Vector2 translateDiff = bodies[i].GetPosition();
        SCALAR rotateDiff = bodies[i].GetRotation();

        // - force generation
        Vector2 gravity = {.0f, -9.8f * bodies[i].mass};
        SCALAR torque = 20;
        bodies[i].AddForce(gravity);
        bodies[i].AddTorque(torque);
        
        // - velocity calculation
        bodies[i].CalculateVelocity();
        bodies[i].CalculateAngularVelocity();
        // cout << bodies[i].velocity.x << bodies[i].velocity.y << endl;
        
        // - collision handling
        for (int j = i + 1; j < bodies.size(); j ++)
        {
            if (IsCollide(&bodies[i], &bodies[j]))
            {
                cout << "> collide!" << endl;
            }
        }

        // - position calculation
        bodies[i].CalculatePosition();
        bodies[i].CalculateAngle();
        // - update position, rotation, vertices

        //  calculate diff position, rotation
        translateDiff.x = bodies[i].GetPosition().x - translateDiff.x;
        translateDiff.y = bodies[i].GetPosition().y - translateDiff.y;
        rotateDiff = bodies[i].GetRotation() - rotateDiff;
        
        // - clear forces
        bodies[i].ClearForce();

        // update collider shape
        // rotateDiff = 3;
        bodies[i].GetCollider()->Update(bodies[i].GetPosition(), translateDiff, rotateDiff);
    }
}

VERTEX_LIST World::GetVertices()
{
    return this->vertices;
}

void World::Create(VERTEX ver, Vector2 pos, SCALAR rot, SCALAR m, float rad)
{
    Body body(ver, pos, rot, m, rad);
    this->bodies.push_back(body);
    // this->vertices.push_back(ver.data());
}

void World::Debug()
{
    for (int i = 0; i < bodies.size(); i ++)
    {
        cout << "Body " << i + 1 << " : ";
        Vector2 vel = bodies[i].GetPosition();
        cout << vel.x << ", " << vel.y << endl;
    }
}

vector<Body> World::GetBodies()
{
    return bodies;
}

void World::run()
{
    // 이 메서드에서 스레드 work 를 실행한다.
    // 모든 물리 연산은 여기서 수행되고 결과를 emit 한다.
    // GL window 가 vsync 가 될 때까지 대기를 하므로, 렌더링을 켜면 연산이 느려진다.
    int count = 0;
    while(count < 1)
    {
        // main physics.
        while(!ready) {}
        ready = false;
        count ++;
        Step();

        // emit for rendering.
        QVariant var;
        vertices.clear();

        for (int i = 0; i < bodies.size(); i ++)
        {
            // var.setValue(bodies[i].GetCollider()->GetVertices());
            vertices.push_back(bodies[i].GetCollider()->GetVertices());
        }
        var.setValue(vertices);
        emit physicsUpdate(var);
    }
}

bool World::IsCollide(Body* body1, Body* body2)
{
    Vector2 center1 = body1->GetCollider()->GetCenter();
    Vector2 center2 = body2->GetCollider()->GetCenter();
    Vector2 direction = center1 - center2;
    Simplex simplex;
    Vector2 a = SupportFunction(body1->GetCollider()->GetVertices(), body2->GetCollider()->GetVertices(), direction);
    simplex.add(a);
    direction = -direction;
    int vertexSum = body1->GetCollider()->GetVertices().size() + body2->GetCollider()->GetVertices().size();
    int count = 0;

    while(vertexSum > count)
    {
        // cout << "> colliding check" << endl;
        Vector2 b = SupportFunction(body1->GetCollider()->GetVertices(), body2->GetCollider()->GetVertices(), direction);
        simplex.add(b);

        if (simplex.GetLastElement().Dot(direction) < 0)
        {
            // origin 을 지나지 않는다는 것이 확실할 때 종료
            return false;
        }
        else
        {
            // origin 지날 때, 아직은 지나지 않을 때
            if (IsContainOrigin(simplex, direction))
            {
                // 이 때 simplex 갖고 있어야 함!
                Collision collision(body1, body2);
                collisionList.push_back(collision);
                collision.FindCollisioninfo(simplex);
                collision.FindManifolds();
                return true;
            }

        }

        count ++;
    }

    return false;
}
