# ifndef RUNNER
# define RUNNER

# include <jsoncpp/json/json.h>
# include <string>
# include <fstream>
# include "Environment.hpp"
# include "Agent.hpp"

using namespace std;

class Runner
{
private:
    // some memebers are from env, agent's meta file.
    int totalEpochs;
    Environment* env;
    Agent* agent;
public:
    Runner(string jsonPath);
    void CreateEnv(); // make env using json config
    void CreateAgent(); // make agent using json config
    void ExecuteTrain();
    void ExecuteSimulate();
    void GetResult();
};

# endif