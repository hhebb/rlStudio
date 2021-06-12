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
# define PI 3.141592
# define INVERSE_RADIAN 0.01745 //PI / 180.0

using namespace std;

struct Vector2
{
    float x;
    float y;
};

struct Matrix3x3
{
    float m11;
    float m12;
    float m13;
    float m21;
    float m22;
    float m23;
    float m31;
    float m32;
    float m33;
    Vector2 center;

    void SetIdentity()
    {
        m11 = 1, m22 = 1, m33 = 1;
        m12 = 0, m13 = 0, m21 = 0, m23 = 0, m31 = 0, m32 = 0;
    }

    void SetCenter(Vector2 center)
    {
        this->center = center;

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

    void Rotate(SCALAR angle)
    {
        Translate(Vector2{-center.x, -center.y});

        SCALAR radAngle = angle * INVERSE_RADIAN;
        float c = cos(radAngle);
        float s = sin(radAngle);

        m11 = c * m11 - s * m21;
        m12 = c * m12 - s * m22;
        m13 = c * m13 - s * m23;
        m21 = s * m11 + c * m21;
        m22 = s * m12 + c * m22;
        m23 = s * m13 + c * m23;
        
        Translate(center);

        // local rotation.
        // m11 = m11 * c + m12 * s;
        // m12 = m12 * c - m11 * s;
        // m21 = m21 * c + m22 * s;
        // m22 = m22 * c - m21 * s;
        // m31 = m31 * c + m32 * s;
        // m32 = m32 * c - m31 * s;
    }


    VERTEX Multiply(VERTEX vertices)
    {
        VERTEX result;
        Vector2 tmp;
        for (int i = 0 ; i < vertices.size(); i ++)
        {
            tmp.x = m11 * vertices[i].x + m12 * vertices[i].y + m13;
            tmp.y = m21 * vertices[i].x + m22 * vertices[i].y + m23;

            // cout << tmp.x << ", " << tmp.y << endl;

            result.push_back(tmp);
        }

        return result;
    }
};


Q_DECLARE_METATYPE(Vector2);
Q_DECLARE_METATYPE(Matrix3x3);

# endif