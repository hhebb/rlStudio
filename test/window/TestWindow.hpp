# ifndef TEST_WINDOW
# define TEST_WINDOW

# include <QOpenGLWindow>
# include <QOpenGLBuffer>
# include <QOpenGLVertexArrayObject>
# include <QOpenGLShaderProgram>

class TestWindow: public QOpenGLWindow
{
    Q_OBJECT
public slots:
    void Render(int); // param 은 뭐?
private:
    QOpenGLBuffer vbo;
    QOpenGLBuffer ibo;
    QOpenGLVertexArrayObject vao;
    QOpenGLShaderProgram program;
    int num;

public:
    bool draw;
    TestWindow();
    ~TestWindow();
    void initializeGL() Q_DECL_OVERRIDE;
	void paintGL() Q_DECL_OVERRIDE;
    void Init();
};


# endif