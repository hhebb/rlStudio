# include "Agent.hpp"

Action Agent::MakeAction(State s)
{
    Action a;
    a.continuousAction = algorithm.Predict();
    return a;
}

void Agent::TakeEnvResponse(State s, REWARD r, bool d)
{
    // learning algorithm 객체에 정보 전달.
    algorithm.Update();
}

void Agent::Train()
{
    ;
}
