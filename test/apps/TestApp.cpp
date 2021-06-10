# include "TestApp.hpp"
# include <iostream>
# include <QKeyEvent>

using namespace std;

TestApp::TestApp(TestWindow* window, World* world)//: TestWIndow*(), World*()
{
    // 생성자에서 멤버 변수, 시그널 전부 셋팅해준다.
    
    this->window = window;
    this->world = world;
    // 물리엔진-렌러창 signal -slot 연결
    connect(this, SIGNAL (requestUpdate(int)), this->window, SLOT (Render(int)));
    window->installEventFilter(this);
}


// void TestApp::SetWorld(World* world)
// {
//     this->world = world;
// }

// void TestApp::SetWindow(TestWindow* window)
// {
//     this->window = window;
    
// }

TestWindow* TestApp::GetWindow()
{
    return window;
}


void TestApp::Run()
{
    // 루프를 돌면서 물리 연산과 디버깅 창 렌더링 동시에 진행.
    // 독립적으로 진행할 수 있도록 변경해야 함.



    // window create 함수 따로 만들기.
    // QSurfaceFormat format;
	// format.setRenderableType(QSurfaceFormat::OpenGL);
	// format.setProfile(QSurfaceFormat::CoreProfile);
	// format.setVersion(3, 3);
    // window->setFormat(format);
	// window->resize(640, 600);
	// window->show();
    //
    count = 0;
    window->draw = true;

    while(count < 10)
    {
        qDebug() << "step: " << count;
        count ++;
        world->Step();
        emit requestUpdate(count*-2 + 103);
    }
    window->draw = false;
}

bool TestApp::eventFilter(QObject *obj, QEvent *event)
{
    if (event->type() == QEvent::KeyPress) {
        QKeyEvent *keyEvent = static_cast<QKeyEvent*>(event);
        // qDebug() << "Ate key press" << keyEvent->key();
        this->Run();

        return true;
    } else {
        return false;
    }

}
