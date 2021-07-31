# ifndef TEST_APP
# define TEST_APP

# include <iostream>
# include "../../src/rl/Runner.hpp"

using namespace std;

class TestApp
{
private:
    Runner* runner;
public:
    void StartApp();
};

# endif