# include "TestWindow.hpp"
# include <iostream>

using namespace std;

TestWindow::TestWindow()
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
    cout << "painting, get: " << num << endl;
}

void TestWindow::Init()
{

}

void TestWindow::Render(int num)
{    
    this->num =  num;
    // 버퍼 데이터들을 받아와서 셋팅해주기.
    // update();
    paintGL();
}