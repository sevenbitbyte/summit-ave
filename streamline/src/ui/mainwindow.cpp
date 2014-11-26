#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "mapwidget.h"

MainWindow::MainWindow(QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::MainWindow)
{
	ui->setupUi(this);

	content = new ContentManager(this);

	mapView = new MapWidget;
	mapView->setContentManager(content);

	ui->mainGridLayout->addWidget(mapView);

	connect(ui->actionOpen_GPX, SIGNAL(triggered()), this, SLOT(openGPXSlot()));
    connect(ui->actionOpen_Directory, SIGNAL(triggered()), this, SLOT(openDirectorySlot()));
	connect(this, SIGNAL(openFiles(QList<QUrl>)), content, SLOT(indexFiles(QList<QUrl>)));

    _gotoCityGroup = new QActionGroup(this);
    ui->actionGotoRichmond->setActionGroup(_gotoCityGroup);
    ui->actionGotoRochester->setActionGroup(_gotoCityGroup);
    ui->actionGotoSan_Francisco->setActionGroup(_gotoCityGroup);
    ui->actionGotoSeattle->setActionGroup(_gotoCityGroup);
    ui->actionGotoSan_Jose->setActionGroup(_gotoCityGroup);
    ui->actionGotoChicago->setActionGroup(_gotoCityGroup);
    ui->actionGotoPortland->setActionGroup(_gotoCityGroup);
    ui->actionGotoWashington_DC->setActionGroup(_gotoCityGroup);

    ui->actionGotoSeattle->setChecked(true);

    connect(_gotoCityGroup, SIGNAL(triggered(QAction*)), this, SLOT(gotoCity(QAction*)));
}

MainWindow::~MainWindow()
{
	delete ui;
	//delete mapView;
}


void MainWindow::openGPXSlot(){
	QStringList paths = QFileDialog::getOpenFileNames(this,
													  tr("Open location files"), QDir::currentPath(),
													  tr("Location Files (*.gpx)"));


	QList<QUrl> urlList;
	for(int i=0; i<paths.size(); i++){
		QUrl url = QUrl::fromLocalFile(paths[i]);

		urlList.append(url);
	}

	content->indexFiles(urlList);
}

void MainWindow::openDirectorySlot(){
    QString dirPaths = QFileDialog::getExistingDirectory(this, tr("Open Directory"),
                                                             "",
                                                             QFileDialog::ShowDirsOnly
                                                             | QFileDialog::DontResolveSymlinks);

    QList<QUrl> urlList;
    QDir dir(dirPaths);

    if(dir.exists()){
        QStringList filters;
        filters << "*.gpx";

        QFileInfoList fileList = dir.entryInfoList(filters, QDir::Files);

        for(int i=0; i<fileList.size(); i++){
            QString filePath = fileList[i].absoluteFilePath();

            if(filePath.endsWith(".gpx", Qt::CaseInsensitive)){
                QUrl url = QUrl::fromLocalFile(filePath);

                if(!urlList.contains(url)){
                    urlList.append(url);
                }
            }
        }
    }

    content->indexFiles(urlList);
}

void MainWindow::gotoCity(QAction *action){
    if(action == ui->actionGotoChicago){
            mapView->setX(-447913);
            mapView->setY(-4636859);
            mapView->setZoom(1);
    }
    else if(action == ui->actionGotoWashington_DC){
        mapView->setX(-323373);
        mapView->setY(-4307109);
        mapView->setZoom(1);
    }
    else if(action == ui->actionGotoPortland){
        mapView->setX(-524840);
        mapView->setY(-5040768);
        mapView->setZoom(1);
    }
    else if(action == ui->actionGotoRichmond){
        mapView->setX(-285058);
        mapView->setY(-4157669);
        mapView->setZoom(1);
    }
    else if(action == ui->actionGotoRochester){
        mapView->setX(-287718);
        mapView->setY(-4782510);
        mapView->setZoom(1);
    }
    else if(action == ui->actionGotoSan_Francisco){
        mapView->setX(-551366);
        mapView->setY(-4181936);
        mapView->setZoom(1);
    }
    else if(action == ui->actionGotoSan_Jose){
        mapView->setX(-597446);
        mapView->setY(-4132418);
        mapView->setZoom(1);
    }
    else if(action == ui->actionGotoSeattle){
        mapView->setX(-550125.0);
        mapView->setY(-5273139.0);
        mapView->setZoom(1);
    }

}
