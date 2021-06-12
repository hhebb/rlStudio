# include "TestWindow.hpp"
# include <iostream>

using namespace std;

TestWindow::TestWindow()
    // : QOpenGLWindow(QOpenGLWindow::NoPartialUpdate)
{
    draw = false;
}

TestWindow::~TestWindow()
{
}

void TestWindow::initializeGL()
{
    Init();
}

void TestWindow::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT);
    glColor3f(1, 0, 0);
    glPointSize(2);

    glBegin(GL_POLYGON);
    for (int i = 0; i < testCoord.size(); i ++)
    {
        glVertex2d(this->testCoord[i].x, this->testCoord[i].y);
        // cout << this->testCoord[i].x << ", " << this->testCoord[i].y << endl;

    }
    glEnd();
    glFlush();
    // cout << "painting" << endl; //<< testCoord.x << ", " << testCoord.y << endl;
}

void TestWindow::Init()
{
    glClearColor(.2f, .2f, 0, 1);
    testCoord = {Vector2{0, 0}, Vector2{.1, .1}, Vector2{.1, 0}}; // x, y order
}


void TestWindow::Render(QVariant pos)
{    
    this->testCoord = pos.value<VERTEX>();
    // // 버퍼 데이터들을 받아와서 셋팅해주기.
    update();
}