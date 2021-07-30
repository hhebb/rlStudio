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
    void CreateEnv(); // make env using json config
    void CreateAgent(); // make agent using json config
    void ExecuteTrain();
    void ExecuteSimulate();
    void GetResult();
};

# endif