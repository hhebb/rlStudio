# ifndef RUNNER
# define RUNNER

# include "Environment.hpp"
# include "Agent.hpp"

class Runner
{
private:
    // some memebers are from env, agent's meta file.
    int totalEpochs;
    Environment* env;
    Agent* agent;
public:
    Runner();
    void MakeEnv(); // make env using json config
    void MakeAgent(); // make agent using json config
    void ExecuteTrain();
    void ExecuteSimulate();
    void GetResult();
};

# endif