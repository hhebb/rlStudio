// #include "_gltest/RectangleWindow.hpp"

#include <QApplication>
#include <QSurfaceFormat>
# include <QPushButton>

int main(int argc, char **argv) {
	QGuiApplication app(argc, argv);

	// Set OpenGL Version information
	QSurfaceFormat format;
	format.setRenderableType(QSurfaceFormat::OpenGL);
	format.setProfile(QSurfaceFormat::CoreProfile);
	format.setVersion(3,3);

	// Note: The format must be set before show() is called.
	QPushButton* button = new QPushButton();
	button->show();

	return app.exec();
}