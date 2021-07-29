# ifndef ENVIRONMENT
# define ENVIRONMENT

# include "../physics/World.hpp"
# include "RLUtil.hpp"

class Environment
{
private:
    State state;
    REWARD reward;
    bool done;
    World* world;

public:
    Environment();
    void Reset();
    void Step(Action action);
    void Render();
    State GetState();
    REWARD GetReward();
    bool GetDone();
};

# endif