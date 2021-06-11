# ifndef NAMESPACE
# define NAMESPACE

# include <vector>
# include <QMetaType>

using namespace std;

# define SCALAR float
# define DELTA_TIME 0.016f
# define GRAVITY 9.8f
# define VERTEX vector<float>

struct Vector2
{
    float x;
    float y;
};

Q_DECLARE_METATYPE(Vector2);

# endif