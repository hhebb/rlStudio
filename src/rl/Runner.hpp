# ifndef RUNNER
# define RUNNER

class Runner
{
private:
    // some memebers are from env, agent's meta file.
    int epochs;
public:
    void ExecuteTrain();
    void ExecuteSimulate();
};

# endif