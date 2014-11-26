#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "arcasciiparser.h"
#include "arcasciirenderer.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
    public:
        explicit MainWindow(QStringList args, QWidget* parent = 0);
        ~MainWindow();

    public slots:
        void arcAsciiDataReady(ArcAsciiData* data);
        void totalProgressChanged(qreal percent);
        void currentFileProgrssChanged(qreal percent, ArcAsciiData* data);
        void arcAsciiParsingFile(QString filePath);
        void arcAsciiParseError(QString filePath);
        void parserFinished();

        void renderProgress(QRect bounds, qreal progress);
        void tileComplete(QRect bounds);

        void tabChanged(int value);

        void saveFile();
        void openFile();

    protected:
        void closeEvent(QCloseEvent *event);

    private:
        Ui::MainWindow* ui;

        QThread* parserThread;
        QThread* renderThread;

        bool parsedStats;

        ArcAsciiParser* parser;
        ArcAsciiRenderer* renderer;
        QList<ArcAsciiData*> dataStats;

        QList<QString> _fileList;

        QList<QThread*> _backgroundThreads;
        QList<ArcAsciiParser*> _backgroundParsers;
};

#endif // MAINWINDOW_H
