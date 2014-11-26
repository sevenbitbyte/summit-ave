#ifndef MAPWIDGET_H
#define MAPWIDGET_H

#include <QtOpenGL>
#include <QGLWidget>
#include <QVector2D>

#include "../contentmanager.h"

const float pi = 3.141592653f;
const float twoPi = 2.0f * pi;
const float piBy2 = 0.5f * pi;
const float degToRad = pi / 180.0f;
const float radToDeg = 180.0f / pi;

const qint64 secondsPerHour = 60 * 60;
const qint64 secondsPerDay = secondsPerHour * 24;

class MapWidget : public QGLWidget
{
		Q_OBJECT
	public:
		explicit MapWidget(QWidget *parent = 0);

		void setContentManager(ContentManager* content);
		
		/**
		 * @brief	screenToWorld converts from a screen position to a world
		 *			coordinate using the depth buffer.
		 * @param position	2d screen position
		 * @return	Nearest 3d world position
		 */
		QVector3D screenToWorld(QPointF position);

		/*double x() const;
		double y() const;
		double zoom() const;*/


	protected:
		virtual void initializeGL();
		virtual void resizeGL(int w, int h);
		virtual void paintGL();


		virtual void keyPressEvent(QKeyEvent* e);
		virtual void mousePressEvent(QMouseEvent* event);
		virtual void mouseReleaseEvent(QMouseEvent* event);
		virtual void mouseMoveEvent(QMouseEvent* event);
		virtual void wheelEvent(QWheelEvent* event);

        QDateTime activeEndTime() const;
        QDateTime activeStartTime() const;

	signals:
		void repaintRequest();
        void doPaint();

	protected slots:
		void paintRequest();
		void startAnimation(int durationMs=500);
		void stopAnimation();
		void stepAnimation();

	public slots:
        void setX(double x);
		void setY(double y);
		void setZ(double z);
        void setZoom(double zoom);
        void setActiveStartTime(QDateTime time);
        void setActiveEndTime(QDateTime time);
        void setActiveTimeRange(qint64 timeSec);
        void setActiveCutoffPercent(qreal percent);
        void setActiveStyleElevation(bool enabled);
        void setActiveStyleSaturation(bool enabled);
        void setActiveStyleCutoff(bool enabled);
		
        void updateLocationData();

	private:
		float _width;
		float _height;
		float _range;
		float _theta;
		float _phi;
		float _aspectRatio;
		QVector2D _lineWidthRange;
		float _lineWidthStep;
		float _lineWidth;

		float _xShift;
		float _yShift;
		float _zShift;
		float _zoom;

		float _heightScale;

		QVector3D _lastMousePos;

		bool _leftDown;
		bool _middleDown;

		bool _animationRunning;
		float _animationDuration;
		QTimer _animationTimer;
		QDateTime _animationStart;

        QThread* _dataProcessingThread;

		QTimer _repaintTimer;
		QTimer _inputTimer;
		QDateTime _lastDrawTime;
		quint64 _lastDrawDuration;
		ContentManager* _content;

        QList<TrackPoint*> _activePoints;
        QList<TrackPoint*> _inactivePoints;

        bool _activeStyleElevationEA;
        bool _activeStyleSaturationEA;
        bool _activeStyleCutoffEA;
        QDateTime _activeEndTime;
        qint64 _activeTimeRangeSec;
        qreal _activeTimeCutoffPercent;
        bool _dataUpdateNeeded;
        bool _inputProcessingNeeded;
};

#endif // MAPWIDGET_H
