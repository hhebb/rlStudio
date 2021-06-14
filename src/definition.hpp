# ifndef NAMESPACE
# define NAMESPACE

# include <vector>
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

using namespace std;

struct Vector2
{
    float x;
    float y;
};

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

Q_DECLARE_METATYPE(Vector2);
Q_DECLARE_METATYPE(Matrix3x3);

# endif