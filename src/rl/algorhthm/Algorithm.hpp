# ifndef ALGORITHM
# define ALGORITHM

# include <vector>

using namespace std;

class Algorithm
{
public:
    vector<float> Predict();
    void Update();
    void Watch(); // agent 가 받아오는 env response 를 지켜보며 알고리즘 조건부 실행.
};

# endif