# ifndef HELPER
# define HELPER

# include "Definition.hpp"

Vector2 GetFarthestVertex(VERTEX, Vector2);
Vector2 SupportFunction(VERTEX, VERTEX, Vector2);
Vector2 TripleProduct(Vector2, Vector2, Vector2);
bool IsContainOrigin(Simplex, Vector2&);

struct Edge
{
    Vector2 a;
    Vector2 b;
    Vector2 e; // ?? 걍 getter 로 가져오기
    Vector2 normal;
    float distance;
    int index;
    Vector2 GetVector();
};

Edge FindClosetEdge(Simplex);

# endif