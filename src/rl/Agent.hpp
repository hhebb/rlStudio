# ifndef AGENT
# define AGENT

# include "RLUtil.hpp"
# include "algorhthm/Algorithm.hpp"

class Agent
{
private:
    // Action action;
    History history;
    Algorithm algorithm;
public:
    Action MakeAction(State s);
    void TakeEnvResponse(State s, REWARD r, bool d); // take 로 이름 바꿀까?
    void Train(); // ??
};

# endif