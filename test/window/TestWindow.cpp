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
    glPointSize(10);
    glBegin(GL_POINTS);
    glVertex2d(this->testCoord.x, this->testCoord.y);
    glEnd();
    glFlush();
    cout << "painting, get: " << testCoord.x << ", " << testCoord.y << endl;
}

void TestWindow::Init()
{
    glClearColor(.2f, .2f, 0, 1);
    testCoord = {0.0f, 0.0f};
}


void TestWindow::Render(QVariant pos)
{    
    this->testCoord = pos.value<Vector2>();
    // // 버퍼 데이터들을 받아와서 셋팅해주기.
    update();
}