#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFileDialog>

MainWindow::MainWindow(QStringList args, QWidget* parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    _fileList=args.mid(1);
    parsedStats = false;
    renderer = NULL;
    ui->setupUi(this);

    parserThread = new QThread();

    parser = new ArcAsciiParser();
    parser->moveToThread(parserThread);


    for(int i=1; i < args.size(); i++){
        parser->queueFile(args[i]);
    }

    connect(ui->actionOpen, SIGNAL(triggered()), this, SLOT(openFile()));

    connect(parser, SIGNAL(dataReady(ArcAsciiData*)), this, SLOT(arcAsciiDataReady(ArcAsciiData*)));
    connect(parser, SIGNAL(totalProgress(qreal)), this, SLOT(totalProgressChanged(qreal)));
    connect(parser, SIGNAL(currentFileProgress(qreal,ArcAsciiData*)), this, SLOT(currentFileProgrssChanged(qreal,ArcAsciiData*)));
    connect(parser, SIGNAL(parsingFile(QString)), this, SLOT(arcAsciiParsingFile(QString)));
    connect(parser, SIGNAL(parseError(QString)), this, SLOT(arcAsciiParseError(QString)));
    connect(parserThread, SIGNAL(destroyed()), parser, SLOT(stopParsing()));
    connect(parserThread, SIGNAL(finished()), parserThread, SLOT(deleteLater()));

    connect(parser, SIGNAL(finished()), this, SLOT(parserFinished()));

    parserThread->start(QThread::LowPriority);

    if(_fileList.size() > 0){
        QTimer::singleShot(10, parser, SLOT(startParsing()));
    }
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::closeEvent(QCloseEvent *event){

}


void MainWindow::arcAsciiDataReady(ArcAsciiData* data){
    QString str;
    QTextStream stream(&str);

    if(!data->elevation.empty()){
        stream << "<b>DataReady: </b>" << data->info.filePath() << "<br>";
    }
    else{
        stream << "<b>HeaderReady: </b>" << data->info.filePath() << "<br>";
    }

    ui->debugTextEdit->insertHtml(str);
    ui->debugTextEdit->ensureCursorVisible();

    dataStats.push_back( data );

    //delete data;
}

void MainWindow::totalProgressChanged(qreal percent){
    int value = percent * (qreal) ui->totalProgressBar->maximum();
    ui->totalProgressBar->setValue( value );
    ui->totalProgressBar->update();
}

void MainWindow::currentFileProgrssChanged(qreal percent, ArcAsciiData* data){
    int value = percent * (qreal) ui->currentFilePsrogressBar->maximum();
    ui->currentFilePsrogressBar->setValue( value );
    ui->currentFilePsrogressBar->update();
}

void MainWindow::arcAsciiParsingFile(QString filePath){
    QString str;
    QTextStream stream(&str);

    stream << "<b>Parsing: </b>" << filePath << "<br>";

    ui->debugTextEdit->insertHtml(str);
    ui->debugTextEdit->ensureCursorVisible();
}

void MainWindow::arcAsciiParseError(QString filePath){
    QString str;
    QTextStream stream(&str);

    stream << "<b><font color=red>Parser Error</font>: </b>" << filePath << "<br>";
    ui->debugTextEdit->ensureCursorVisible();

    ui->debugTextEdit->insertHtml(str);
}

void MainWindow::parserFinished(){
    if(!parsedStats){
        parsedStats = true;


        renderThread = new QThread();
        renderer = new ArcAsciiRenderer(dataStats);
        renderer->moveToThread(renderThread);

        connect(renderer, SIGNAL(renderProgress(QRect,qreal)), this, SLOT(renderProgress(QRect,qreal)));
        connect(renderer,SIGNAL(tileComplete(QRect)), this, SLOT(tileComplete(QRect)));
        connect(ui->tabWidget, SIGNAL(currentChanged(int)), this, SLOT(tabChanged(int)));
        connect(ui->actionSave, SIGNAL(triggered()), this, SLOT(saveFile()));

        _backgroundThreads.push_back(parserThread);
        _backgroundParsers.push_back(parser);

        connect(renderThread, SIGNAL(finished()), renderThread, SLOT(deleteLater()));
        connect(parser, SIGNAL(dataReady(ArcAsciiData*)), renderer, SLOT(processData(ArcAsciiData*)));

        disconnect(parser, SIGNAL(dataReady(ArcAsciiData*)), this, SLOT(arcAsciiDataReady(ArcAsciiData*)));

        //Adapt thread setup to system
        int threadCount = QThread::idealThreadCount() - 1;
        if(threadCount < 1){
            threadCount = 2;
        }
        else if(threadCount > 4){
            //More than 4 probably saturates HDD bandwidth on most systems
            threadCount = qMin(threadCount / 2, 4);
        }

        qDebug() << "Starting " << threadCount << " additional parser threads";

        for(int i=0; i<threadCount; i++){
            QThread* t = new QThread();
            ArcAsciiParser* p = new ArcAsciiParser();

            _backgroundThreads.push_back(t);
            _backgroundParsers.push_back(p);

            p->moveToThread(t);

            connect(p, SIGNAL(parsingFile(QString)), this, SLOT(arcAsciiParsingFile(QString)));
            connect(p, SIGNAL(parseError(QString)), this, SLOT(arcAsciiParseError(QString)));
            connect(t, SIGNAL(destroyed()), p, SLOT(stopParsing()));
            connect(t, SIGNAL(finished()), t, SLOT(deleteLater()));
            connect(p, SIGNAL(dataReady(ArcAsciiData*)), renderer, SLOT(processData(ArcAsciiData*)));
            t->start();
        }

        qDebug() << "Rendering " << _fileList.size() << " files";

        for(int i=0; i < _fileList.size(); i++){
            _backgroundParsers[i%_backgroundParsers.size()]->queueFile(_fileList[i]);
        }

        renderThread->start(QThread::HighPriority);
    }
}

void MainWindow::tabChanged(int index){
    qDebug() << "tabChanged";
    if(parsedStats && !parser->parsing()){
        for(int i=0; i<_backgroundParsers.size(); i++){
            _backgroundParsers[i]->setLoadElevation(true);
            qDebug() << "Starting parser " << i;

            QTimer::singleShot(500, _backgroundParsers[i], SLOT(startParsing()));
        }
    }
}

void MainWindow::renderProgress(QRect bounds, qreal progress){
    int value = progress * (qreal) ui->tileRenderProgress->maximum();
    ui->tileRenderProgress->setValue( value );
    ui->tileRenderProgress->update();
}


void MainWindow::tileComplete(QRect bounds){
    QPixmap temp = renderer->getPixmap();

    //QPainter painter(&temp);

    //painter.setPen(Qt::green);

    int w = ui->renderOutput->width();
    int h = ui->renderOutput->height();
    ui->renderOutput->setPixmap(temp.scaled(w,h,Qt::KeepAspectRatio));
    ui->renderOutput->update();
}

void MainWindow::saveFile(){
    QString outputPath = QFileDialog::getSaveFileName(this, tr("Save output image"), QDir::currentPath());

    QPixmap pixmap = renderer->getPixmap();

    pixmap.save(outputPath, "PNG", 100);

    qDebug() << "Pixmap Alpha Channel: " << pixmap.hasAlphaChannel();
}


void MainWindow::openFile(){
    _fileList = QFileDialog::getOpenFileNames(this, "Open ArcAscii files", QDir::currentPath(), "*.asc");

    foreach(QString fileName, _fileList){
        parser->queueFile(fileName);
    }

    if(_fileList.size() > 0){
        QTimer::singleShot(10, parser, SLOT(startParsing()));
    }
}
