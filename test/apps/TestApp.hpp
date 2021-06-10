# ifndef TEST_APP
# define TEST_APP

# include <QObject>
# include "../../src/physics/World.hpp"
# include "../window/TestWindow.hpp"

class TestApp: public QObject
{
    Q_OBJECT

signals:
    void requestUpdate(int); // vertices 를 넘겨줘야함.
public slots:
    void Run();

private:
    World* world;
    TestWindow* window;
    int count;
public:
    TestApp(TestWindow*, World*);
    void SetWorld(World*);
    void SetWindow(TestWindow*);
    TestWindow* GetWindow();
    bool eventFilter(QObject*, QEvent*);
};

# endif