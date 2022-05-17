#include <QApplication>
#include <QLabel>
#include <QWidget>

int main(int argc, char** argv){
    QApplication app(argc, argv);
    QLabel window("<center>Test Qt</center>");
    window.setWindowTitle("window title");
    window.resize(600, 400);
    window.show();
    return app.exec();
}