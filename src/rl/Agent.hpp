# ifndef AGENT
# define AGENT

# include "RLUtil.hpp"

class Agent
{
private:
    // Action action;
    History history;
public:
    Action MakeAction(State s);
    void SetEnvResponse(State s, REWARD r, bool d);
    void Train();
};

# endif