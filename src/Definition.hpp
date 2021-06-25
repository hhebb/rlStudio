# ifndef DEFINITION
# define DEFINITION

# include <vector>
# include <queue>
# include <math.h>
# include <QMetaType>
# include <iostream>

using namespace std;

# define SCALAR float
# define DELTA_TIME 0.016f
# define GRAVITY 9.8f
# define VERTEX vector<Vector2> //vector<float>
# define VERTEX_LIST vector<VERTEX>
# define PI 3.141592
# define INVERSE_RADIAN 0.0174533 //PI / 180.0
# define TOLERANCE 0.0001

using namespace std;

enum BodyType {DYNAMIC, KINEMATIC, STATIC};

struct Vector2
{
    float x;
    float y;

    void Set(float x, float y)
    {
        this->x = x;
        this->y = y;
    }

    void Set(Vector2 vec)
    {
        this->x = vec.x;
        this->y = vec.y;
    }

    SCALAR Dot(Vector2 vec)
    {
        return SCALAR{x * vec.x + y * vec.y};
    }

    Vector2 Normalise()
    {
        float factor = sqrt(x * x + y * y);
        return Vector2{x / factor, y / factor};
    }

    // operator
    Vector2 operator-()
    {
        return Vector2{-x, -y};
    }

    Vector2 operator+(Vector2 vec)
    {
        return Vector2{x + vec.x, y + vec.y};
    }

    Vector2 operator-(Vector2 vec)
    {
        return Vector2{x - vec.x, y - vec.y};
    }

    Vector2 operator*(SCALAR scale)
    {
        return Vector2{x * scale, y * scale};
    }

    Vector2 operator/(SCALAR div)
    {
        return Vector2{x / div, y / div};
    }

    Vector2 Cross(float z)
    {
        return Vector2{-1.0f * y * z, x * z};
    }
};


// homogeneous transform matrix for 2D.
struct Matrix3x3
{
    float m11, m12, m13, m21, m22, m23, m31, m32, m33;
    Vector2 center;

    void SetIdentity()
    {
        m11 = 1, m22 = 1, m33 = 1;
        m12 = 0, m13 = 0, m21 = 0, m23 = 0, m31 = 0, m32 = 0;
    }

    void Translate(Vector2 translate)
    {
        m11 = m11 + translate.x * m31;
        m12 = m12 + translate.x * m32;
        m13 = m13 + translate.x * m33;
        m21 = m21 + translate.y * m31;
        m22 = m22 + translate.y * m32;
        m23 = m23 + translate.y * m33;
    }

    void Rotate(SCALAR angle, Vector2 center)
    {
        SCALAR radAngle = angle * INVERSE_RADIAN;
        float c = cos(radAngle);
        float s = sin(radAngle);
        float e1 = center.x * (1 - c) + center.y * s;
        float e2 = center.y * (1 - c) - center.x * s;

        m11 = c * m11 - s * m21 + e1 * m31;
        m12 = c * m12 - s * m22 + e1 * m32;
        m13 = c * m13 - s * m23 + e1 * m33;
        m21 = s * m11 + c * m21 + e2 * m31;
        m22 = s * m12 + c * m22 + e2 * m32;
        m23 = s * m13 + c * m23 + e2 * m33;
        
    }


    VERTEX Multiply(VERTEX vertices)
    {
        VERTEX result;
        Vector2 tmp;
        for (int i = 0 ; i < vertices.size(); i ++)
        {
            tmp.x = m11 * vertices[i].x + m12 * vertices[i].y + m13;
            tmp.y = m21 * vertices[i].x + m22 * vertices[i].y + m23;
            result.push_back(tmp);
        }

        return result;
    }
};

struct Simplex
{
    // float a, b, c;
    vector<Vector2> elements;

    void add(Vector2 v)
    {
        elements.push_back(v);
    }

    Vector2 GetLastElement()
    {
        return elements[elements.size() - 1];
    }

    Vector2 GetB()
    {
        return elements[elements.size() - 2];
    }

    Vector2 GetC()
    {
        return elements[0];
    }

    void Remove(int index)
    {
        elements.erase(elements.begin() + index);
    }

    void Insert(Vector2 v, int index)
    {
        elements.insert(elements.begin() + index, v);
    }

};

struct ClippedPoints
{
    VERTEX cPoints;
};

struct MassData
{
    SCALAR density = 1;
    SCALAR mass; // = density * area
    SCALAR inverseMass;
    SCALAR area;
    SCALAR inertia;
    SCALAR inverseInertia;
    Vector2 centroid;
};

Q_DECLARE_METATYPE(Vector2);
Q_DECLARE_METATYPE(Matrix3x3);

# endif