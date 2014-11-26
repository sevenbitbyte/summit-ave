#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>


#include "src/ui/mapwidget.h"
#include "../contentmanager.h"

namespace Ui {
class MainWindow;

}

class MainWindow : public QMainWindow
{
		Q_OBJECT
		
	public:
		explicit MainWindow(QWidget *parent = 0);
		~MainWindow();
		
	signals:
		void openFiles(QList<QUrl> files);

	public slots:
		void openGPXSlot();
        void openDirectorySlot();
        void gotoCity(QAction* action);

	private:
		ContentManager* content;
		MapWidget* mapView;
		Ui::MainWindow *ui;
        QActionGroup* _gotoCityGroup;
};

#endif // MAINWINDOW_H
