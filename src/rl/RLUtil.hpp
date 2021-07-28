# ifndef RL_Util
# define RL_Util

# define DISCRETE_STATE vector<int>
# define CONTINUOUS_STATE vector<float>
# define IMAGE_STATE vector<vector<float>>
# define DISCRETE_ACTION vector<int>
# define CONTINUOUS_ACTION vector<float>

# include <vector>

using namespace std;

struct State
{
    // discrete - usually single value.
    DISCRETE_STATE discreteState;
    // continuous
    CONTINUOUS_STATE continuousState;
    // image
    IMAGE_STATE imageState;
};

struct Action
{
    // discrete - usually single value.
    DISCRETE_ACTION discreteAction;
    // continuous
    CONTINUOUS_ACTION continuousAction;
};


# endif