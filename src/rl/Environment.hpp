# ifndef ENVIRONMENT
# define ENVIRONMENT

# include "../physics/World.hpp"

class Environment
{
private:
    World world;
public:
    void Step();
    void Reset();
    void Render();
};

# endif