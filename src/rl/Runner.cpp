# include "Runner.hpp"

Runner::Runner(string jsonPath)
{
    ifstream s;
    s.open(jsonPath);
    Json::Reader reader;
    Json::Value root;
    if (!reader.parse(s, root))
    {
        cout << "> json parse error." << endl;
        return;
    }
    
}

void Runner::CreateEnv()
{
    env = new Environment();
}

void Runner::CreateAgent()
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
            agent->TakeEnvResponse(nextState, reward, done);
            
            env->Render();
        }
    }
}

void Runner::ExecuteSimulate()
{
    ;
}