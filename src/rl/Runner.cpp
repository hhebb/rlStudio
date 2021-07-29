# include "Runner.hpp"

Runner::Runner()
{
    ;
}

void Runner::MakeEnv()
{
    env = new Environment();
}

void Runner::MakeAgent()
{
    agent = new Agent();
}

void Runner::ExecuteTrain()
{
    env->Reset();
    for (int epoch = 0; epoch < totalEpochs; epoch ++)
    {
        // take a step until env get done
        while (true)
        {
            State currentState = env->GetState();
            Action currentAction = agent->MakeAction(currentState);
            env->Step(currentAction);
            
            State nextState = env->GetState();
            REWARD reward = env->GetReward();
            bool done = env->GetDone();
            agent->SetEnvResponse(nextState, reward, done);
            
            env->Render();
        }
    }
}

void Runner::ExecuteSimulate()
{
    ;
}