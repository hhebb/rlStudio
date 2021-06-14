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
    vert = {Vector2{-.4, -.1}, Vector2{.4, -.1}, Vector2{.4, .1}, Vector2{-.4, .1}}; // x, y order
    pos = {0.0, -0.1};
    rot = 0;
    mass = 1;
    radius = 0;
    Create(vert, pos, rot, mass, radius);


    // 예시 body
    vert = {Vector2{0, 0.5}, Vector2{.1, .6}, Vector2{.1, .5}}; // x, y order
    pos = {0.0, 0.5};
    rot = 0;
    mass = 1;
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
        bodies[i].AddForce(gravity);
        
        // - velocity calculation
        bodies[i].CalculateVelocity();
        // cout << bodies[i].velocity.x << bodies[i].velocity.y << endl;
        
        // - collision handling
        // - position calculation
        bodies[i].CalculatePosition();
        // - update position, rotation, vertices
        // bodies[i].GetCollider().Update();

        //  calculate diff position, rotation
        translateDiff.x = bodies[i].GetPosition().x - translateDiff.x;
        translateDiff.y = bodies[i].GetPosition().y - translateDiff.y;
        rotateDiff = bodies[i].GetRotation() - rotateDiff;
        
        // - clear forces
        bodies[i].ClearForce();

        // update collider shape
        rotateDiff = 3;
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