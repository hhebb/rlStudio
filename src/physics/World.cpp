# include "World.hpp"
# include <iostream>
using namespace std;

World::World()
{
    // vertices;
    Init();
    Reset();
}

void World::Init()
{
    // body 들 생성, 물성치 설정. 모든 물리 환경 설정.
    // Create();
}

void World::Reset()
{
    // 초기 물리 환경 설정으로 초기화.
}

void World::Step()
{
    Debug();

    // obj 들 update, vertices update
    for (int i = 0; i < bodies.size(); i ++)
    {        
        // - force generation
        Vector2 gravity = {0, -9.8};
        bodies[i].AddForce(gravity);
        // - velocity calculation
        bodies[i].CalculateVelocity();
        // - collision handling
        // - position calculation
        // bodies[i].CalculatePosition();
        // - update position, rotation, vertices
        // bodies[i].GetCollider().Update();
        // - clear forces
        bodies[i].ClearForce();
    }
}

VERTEX World::GetVertices()
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
        // Vector2 pos = bodies[i].GetPosition();
        // cout << pos.x << ", " << pos.y << endl;
    }
}