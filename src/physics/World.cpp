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
    POLY_DATA vert; // vertices
    Vector2 pos; // center
    SCALAR rot;
    SCALAR mass;
    float radius;

    // 바닥
    pos = {0.0, -0.95};
    vert = {Vector2{0.0, 0.0}, Vector2{2, .0}, Vector2{2, .1}, Vector2{0.0, .1}}; // x, y order
    rot = 0;
    Create(vert, pos, rot, 0, STATIC);
    // bodies[0].SetVel(Vector2{0, 5.9});
    // bodies[0].SetAngular(500);

    // 예시 body
    pos = {-0.6, 0.6};
    // vert = {Vector2{0.0, 0.0}, Vector2{.4, .0}, Vector2{.4, .4}, Vector2{0.0, .4}}; // x, y order
    // vert = {Vector2{-0.2, 0.0}, Vector2{.4, .0}, Vector2{.2, .2}, Vector2{0.0, .2}}; // x, y order
    vert = {Vector2{0.0, 0.0}, Vector2{.2, 0.0}, Vector2{.13, .2}}; // x, y order
    rot = -30;
    Create(vert, pos, rot, 2, DYNAMIC);
    bodies[1].SetVel(Vector2{.1, 2.0});
    bodies[1].SetAngular(50);

    // 예시 body
    pos = {0.7, 0.6};
    // vert = {Vector2{0.0, 0.0}, Vector2{.4, .0}, Vector2{.4, .4}, Vector2{0.0, .4}}; // x, y order
    // vert = {Vector2{-0.2, 0.0}, Vector2{.4, .0}, Vector2{.2, .2}, Vector2{0.0, .2}}; // x, y order
    vert = {Vector2{0.0, 0.0}, Vector2{.2, 0.0}, Vector2{.13, .2}}; // x, y order
    rot = 0;
    Create(vert, pos, rot, 1, DYNAMIC);
    bodies[2].SetVel(Vector2{-.3, 1.9});
    bodies[2].SetAngular(-50);
    
    
}

void World::Reset()
{
    // 초기 물리 환경 설정으로 초기화.
}

void World::Step()
{
    // physics pipeline.
    // obj 들 update, vertices update
    for (int i = 0; i < bodies.size(); i ++)
    {   
        Vector2 translateDiff = bodies[i].GetPosition();
        SCALAR rotateDiff = bodies[i].GetRotation();

        // - force generation
        Vector2 gravity = {.0, -GRAVITY * bodies[i].GetMass()};
        SCALAR torque = 1;
        bodies[i].AddForce(gravity);
        // bodies[i].AddTorque(torque);
        
        // - velocity calculation
        bodies[i].CalculateVelocity();
        bodies[i].CalculateAngularVelocity();
        // bodies[i].SetVel(500);

        // cout << "> velocity: " << i << ", " << bodies[i].GetVelocity().x << ", " << bodies[i].GetVelocity().y << endl;
        // cout << "> position: " << i << ", " << bodies[i].GetPosition().x << ", " << bodies[i].GetPosition().y << endl;
        
        // - collision detection
        for (int j = i + 1; j < bodies.size(); j ++)
        {
            if (IsCollide(&bodies[i], &bodies[j]))
            {
                cout << "> collide!" << endl;
            }
        }

        // collision solve
        for (int j = 0; j < collisionList.size(); j ++)
        {
            collisionList[j]->Solve();
        }

        // - position calculation
        bodies[i].CalculatePosition();
        bodies[i].CalculateAngle();
        // - update position, rotation, vertices

        //  calculate diff position, rotation
        translateDiff = bodies[i].GetPosition() - translateDiff;
        rotateDiff = bodies[i].GetRotation() - rotateDiff;
        
        // - clear forces
        bodies[i].ClearForce();

        // update collider shape, body centroid
        bodies[i].GetCollider()->Update(bodies[i].GetPosition(), translateDiff, rotateDiff);
        bodies[i].UpdateCentroid();

        // clear collisions
        collisionList.clear();
    }
}

POLY_LIST World::GetVertices()
{
    return this->vertices;
}

void World::Create(POLY_DATA ver, Vector2 pos, SCALAR rot, int id, BodyType t)
{
    Body body(ver, pos, rot, id, 1, t);
    this->bodies.push_back(body);
    // this->vertices.push_back(ver.data());
}

void World::Debug()
{
    for (int i = 0; i < bodies.size(); i ++)
    {
        // cout << "Body " << i + 1 << " : ";
        Vector2 vel = bodies[i].GetPosition();
        // cout << vel.x << ", " << vel.y << endl;
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
    Vector2 center1 = body1->GetPosition();
    Vector2 center2 = body2->GetPosition();
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
        
        // cout << "count: " << count << endl;
        
        if (simplex.GetLastElement().Dot(direction) < 0)
        {
            // origin 을 지나지 않는다는 것이 확실할 때 종료
            // cout << "not collide" << endl;
            // PrintVector("last: ", simplex.GetLastElement());
            // PrintVector("dir: ", direction);
            return false;
        }
        else
        {
            // PrintVector("contain, last: ", simplex.GetLastElement());
            // PrintVector("contain, dir: ", direction);
            // origin 지날 때, 아직은 지나지 않을 때
            if (IsContainOrigin(simplex, direction))
            {
                // 이 때 simplex 갖고 있어야 함!
                Collision* collision = new Collision(body1, body2);
                collisionList.push_back(collision);
                collision->FindCollisioninfo(simplex);
                collision->FindManifolds();
                return true;
            }
            // PrintVector("changed dir", direction);

        }

        count ++;
    }

    return false;
}
