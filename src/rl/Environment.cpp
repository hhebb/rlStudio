# include "Environment.hpp"

Environment::Environment()
{
    // world 객체 생성, world json parsing.
    world = new World();
    world->Init(); // need json parse
}

void Environment::Reset()
{
    // initialize env's state.
    // state.discreteState = ;
}

void Environment::Step(Action action)
{
    // world 에 action 전달.
    // world->RegistAction(action);
    world->Step();
    // state update // state = newState;
    // reward update
}

void Environment::Render()
{
    ;
}

State Environment::GetState()
{
    return state;
}

REWARD Environment::GetReward()
{
    return reward;
}

bool Environment::GetDone()
{
    return done;
}