# ifndef RUNNER
# define RUNNER

# include "Environment.hpp"
# include "Agent.hpp"

class Runner
{
private:
    // some memebers are from env, agent's meta file.
    int epochs;
    Environment env;
    Agent agent;
public:
    void ExecuteTrain();
    void ExecuteSimulate();
};

# endif