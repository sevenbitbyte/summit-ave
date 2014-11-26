#include "mapwidget.h"

#include<map>
#include<vector>

#include <GL/glu.h>
#include <GL/glut.h>

using namespace std;

class Filter {
	public:
		//int usedLength;
		//int bufferIndex;
		unsigned int bufferLength;
		unsigned int mediansLength;
		//int kernelLength;
		vector<double> kernel;
		vector<double> data;
		vector<double> medians;
		multimap<double,double> medianMap;

		enum FilterType { Filter_Average, Filter_Median };

		uint8_t type;

		Filter(){
			bufferLength = 15;
			mediansLength = 15;
			data.clear();
			kernel.clear();
			medians.clear();

			type = Filter_Median;

			//kernel.push_back(1.0/5.0);
		}

		Filter(int dataLength){
			bufferLength = dataLength;
			data.clear();
			kernel.clear();
			medians.clear();
		}

		void addKernel(double value){
			kernel.push_back(value);
		}

		double getValue(double value){


			if(type == Filter_Median){
				//if(data.size() < 1){
					data.push_back(value);
				//}

				//double avg = 0.0;
				medianMap.clear();
				for(int i=0; i < data.size(); i++){
					medianMap.insert(make_pair(data[i], data[i]));
					//avg += data[i];
				}
				//avg = avg / (double) data.size();

				double median = getValue();

				medians.push_back(median);

				if(medians.size() > mediansLength){
					medians.erase(medians.begin());
				}

				value = 0.0;
				for(unsigned int i=0; i<medians.size(); i++){
					value += medians[i];
				}

				value = value / (double) medians.size();

				//if(data.size() > 10){
					//data.pop_back();
					//data.push_back(median);

				//}
				//value = (median * 0.9) + (avg*0.1);
			}
			else if(type == Filter_Average){
				data.push_back(value);
				value = getValue();
			}


			if(data.size() > bufferLength){
				data.erase(data.begin());
			}


			return value;
		}

		double getValue(){
			double value = 0.0;

			if(type == Filter_Average){
				vector<double>::iterator kernelIter = kernel.begin();
				vector<double>::iterator dataIter = data.end();

				for(int i=0; i < data.size(); i++){
					double datam = data[i];

					if(kernel.size() != 0){
						value += datam * (*kernelIter);

						kernelIter++;

						if(kernelIter == kernel.end()){
							kernelIter = kernel.begin();
						}
					}
					else{
						value += datam;
					}
					dataIter--;
				}

				if(type==Filter_Average && kernel.size() == 0){
					value = value / (double) bufferLength;
				}
			}
			else if(type == Filter_Median){
				//select median value
				int middle = medianMap.size()/2;

				map<double,double>::iterator medianIter = medianMap.begin();
				for(int i=0; medianIter != medianMap.end(); medianIter++,i++){
					if(i==middle){
						value = medianIter->second;
						break;
					}
				}
			}

			return value;
		}
};


MapWidget::MapWidget(QWidget *parent) :
	QGLWidget(parent)
{
	_theta = 0.0f;
	_phi = 0.0f;
	_aspectRatio = 1.0f;
	_lineWidthRange = QVector2D();
	_lineWidthStep = 0.0f;
	_lineWidth = 1.0f;
	_range = 50.0f;

    _xShift = -549470; //551965);
    _yShift = -5277208; //5274200);
    //_xShift = -550125;
    //_yShift = -5273139;
	_zShift = 30;

	_zoom = 1;



    _activeTimeCutoffPercent = 4;
    _activeTimeRangeSec = secondsPerDay * 7;

    _activeEndTime=QDateTime::currentDateTimeUtc();

    _activeStyleCutoffEA = true;
    _activeStyleElevationEA = true;
    _activeStyleSaturationEA = true;
    _dataUpdateNeeded = true;
    _inputProcessingNeeded = true;

	_leftDown = false;
	_middleDown = false;

	_heightScale = 1.0f;

	this->setFocusPolicy(Qt::StrongFocus);

	_repaintTimer.setSingleShot(false);
	_repaintTimer.setInterval(30);

	_inputTimer.setSingleShot(true);
    _inputTimer.setInterval(5);

    /*_dataProcessingThread = new QThread(this);
    _inputTimer.moveToThread(_dataProcessingThread);
    _repaintTimer.moveToThread(_dataProcessingThread);*/

	connect(&_repaintTimer, SIGNAL(timeout()), this, SLOT(paintRequest()));
	connect(this, SIGNAL(repaintRequest()), this, SLOT(paintRequest()));
	connect(&_inputTimer, SIGNAL(timeout()), this, SLOT(paintRequest()));
	connect(&_animationTimer, SIGNAL(timeout()), this, SLOT(stepAnimation()));
    connect(this, SIGNAL(doPaint()), this, SLOT(updateGL()));

    //connect(_dataProcessingThread, SIGNAL(finished()), _dataProcessingThread, SLOT(deleteLater()));
    //_dataProcessingThread->start();

    _repaintTimer.start(1000);
    _lastDrawTime = QDateTime::currentDateTime();
}

void MapWidget::setX(double x){
    _xShift = x;
}

void MapWidget::setY(double y){
    _yShift = y;
}

void MapWidget::setZ(double z){
    _zShift = z;
}

void MapWidget::setZoom(double zoom){
    _zoom = zoom;
}


void MapWidget::setActiveStartTime(QDateTime time) {
    _activeEndTime = time.addSecs(_activeTimeRangeSec);
    updateLocationData();
}

void MapWidget::setActiveEndTime(QDateTime time) {
    _activeEndTime = time;
    updateLocationData();
}

void MapWidget::setActiveTimeRange(qint64 timeSec) {
    _activeTimeRangeSec = timeSec;
    updateLocationData();
}

void MapWidget::setActiveCutoffPercent(qreal percent) {
    Q_ASSERT(percent > 0.0f);
    _activeTimeCutoffPercent = percent;
}

void MapWidget::setActiveStyleElevation(bool enabled) {
    _activeStyleElevationEA = enabled;
}

void MapWidget::setActiveStyleSaturation(bool enabled) {
    _activeStyleSaturationEA = enabled;
}

void MapWidget::setActiveStyleCutoff(bool enabled) {
    _activeStyleCutoffEA = enabled;
}

void MapWidget::setContentManager(ContentManager* content){
    _content = content;
}

void MapWidget::updateLocationData(){
    QDateTime start = activeStartTime();
    QDateTime end = activeEndTime();
    QList<TrackSegment*> segmentList;

    if(!_activeStyleCutoffEA){
        segmentList = _content->locationData().values();
    }
    else{
        //Always show 1 month of history
        qint64 cutoffPaddingSec = qMax((qint64) (_activeTimeCutoffPercent * _activeTimeRangeSec), (qint64) secondsPerDay * 31);

        segmentList = _content->getLocationByTime(start.addSecs(-cutoffPaddingSec), end.addSecs(cutoffPaddingSec));
    }

    _activePoints.clear();
    _inactivePoints.clear();

    for(int i=0; i<segmentList.size(); i++){

        Filter filter;
        TrackSegment* segment = segmentList[i];

        Q_ASSERT(segment != NULL);

        for(int j=0; j<segment->points.size(); j++){
            TrackPoint* point = segment->points[j];

            float filterdZ = (point->elevation() + _zShift) * _heightScale;;

            if(point->filtered()){
                //filterdZ = _filteredZMap[point];
            }
            else{
                filterdZ = filter.getValue(filterdZ);

                point->setElevation(filterdZ);
                point->setFiltered(true);
            }

            if(point->time() >= start && point->time() <= end){
                _activePoints.push_back(point);
            }
            else{
                _inactivePoints.push_back(point);
            }
        }

    }
}

void MapWidget::paintRequest(){
	QDateTime currentTime = QDateTime::currentDateTime();

    bool drawingDirty=false;
	quint64 elapsedMs = _lastDrawTime.msecsTo(currentTime);


    if(_dataUpdateNeeded){
        updateLocationData();
        _dataUpdateNeeded = false;
        drawingDirty=true;

        elapsedMs = currentTime.msecsTo(QDateTime::currentDateTime());

        int count = _activePoints.size() + _inactivePoints.size();

        qDebug() << "ActiveTimeRange={" << activeStartTime().toString() << "-" << activeEndTime().toString() << "} selected " <<
                    count << " points (" << _activePoints.size() << "active, " << _inactivePoints.size() << "inactive) in " << elapsedMs << "ms";
        _inputTimer.setInterval(100);
    }

    if(_inputProcessingNeeded){
        _inputProcessingNeeded = false;
        drawingDirty=true;
        _inputTimer.setInterval(5);
    }

    if(!drawingDirty){
        return;
    }

		_lastDrawTime = currentTime;
        updateGL();

        //emit doPaint();


        elapsedMs = currentTime.msecsTo(QDateTime::currentDateTime());

        qDebug() << "Render Time: " << elapsedMs;

        if(elapsedMs >= 500){
            if(_lastDrawDuration != 1000){
                qDebug() << "Render Time: " << elapsedMs << "ms\t Draw Timer: 1000ms";
            }
            _lastDrawDuration = 1000;
            //_repaintTimer.setInterval(1000);
            //_inputTimer.setInterval(60);
		}
        else if(elapsedMs >= 200){
            if(_lastDrawDuration != 500){
                qDebug() << "Render Time: " << elapsedMs << "ms\t Draw Timer: 500ms";
            }
            _lastDrawDuration = 500;
            //_repaintTimer.setInterval(500);
            //_inputTimer.setInterval(100);
		}
        else if(elapsedMs >= 100){
            if(_lastDrawDuration != 200){
                qDebug() << "Render Time: " << elapsedMs << "ms\t Draw Timer: 200ms";
            }
            _lastDrawDuration = 200;
            //_repaintTimer.setInterval(200);
            //_inputTimer.setInterval(50);
		}
		else{
            if(_lastDrawDuration != 50){
                qDebug() << "Render Time: " << elapsedMs << "ms\t Draw Timer: 50ms";
            }
            _lastDrawDuration = 50;
            //_inputTimer.setInterval(25);
            //_repaintTimer.setInterval(50);
        }
    //}
}

void MapWidget::startAnimation(int durationMs){
	_animationDuration = durationMs;
	_animationStart = QDateTime::currentDateTime();

	_animationRunning = true;

	_animationTimer.start(30);
}

void MapWidget::stopAnimation(){
	_animationTimer.stop();
	_heightScale = 1.0;


    _inputProcessingNeeded = true;
	emit repaintRequest();

	_animationRunning=false;
}

void MapWidget::stepAnimation(){
	QDateTime currentTime = QDateTime::currentDateTime();

	double delta = _animationStart.msecsTo(currentTime) / _animationDuration;

	_heightScale = qMax(0.01, qMin(delta, 1.0));

	qDebug() << "stepAnimation() - delta=" << delta;

	if(delta >= 1.0){
		stopAnimation();
		return;
	}

    _inputProcessingNeeded = true;
	emit repaintRequest();
}

QVector3D MapWidget::screenToWorld(QPointF position){
	GLint viewport[4];
	GLdouble modelview[16];
	GLdouble projection[16];
	GLfloat winX, winY, winZ;
	GLdouble posX, posY, posZ;

	glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
	glGetDoublev( GL_PROJECTION_MATRIX, projection );
	glGetIntegerv( GL_VIEWPORT, viewport );

	winX = position.x();
	winY = (float)viewport[3] - position.y();
	glReadPixels( position.x(), int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ );

	gluUnProject( winX, winY, winZ, modelview, projection, viewport, &posX, &posY, &posZ);

	return QVector3D(posX, posY, posZ);
}

void MapWidget::initializeGL(){

	//Set clear color to black
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

	//Set drawing color to white
	glColor3f(0.0f, 0.0f, 1.0f);

	// Query some info about supported point sizes
	glGetFloatv( GL_LINE_WIDTH_RANGE, reinterpret_cast<float*>( &_lineWidthRange ) );
	glGetFloatv( GL_LINE_WIDTH_GRANULARITY, &_lineWidthStep );

	qDebug() << "Point size range:" << _lineWidthRange;
	qDebug() << "Point size step:" << _lineWidthStep;

	_lineWidth = _lineWidthRange.x();

	glEnable(GL_DEPTH);

	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable( GL_BLEND );

    //glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH | GLUT_STENCIL);   // display mode
}


void MapWidget::resizeGL(int w, int h){
	// Prevent zero width or height
	if ( h <= 0 ){
		h = 1;
	}

	if ( w <= 0 ){
		w = 1;
	}

	// Set the viewport to window dimensions
	glViewport( 0, 0, w, h );

	// reset the coordinate system
	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();

	// Establish the clipping volume by setting up an orthographic projection
	_aspectRatio = double( w ) / double( h );
	_width = w;
	_height = h;
	if ( w <=h )
		glOrtho( -_range, _range, -_range / _aspectRatio, _range / _aspectRatio, _range*10, -_range*100 );
	else
		glOrtho( -_range * _aspectRatio, _range * _aspectRatio, -_range, _range, _range*10, -_range*100 );

	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();
}


void MapWidget::paintGL()
{
	if(_content != NULL){
		glClear( GL_COLOR_BUFFER_BIT );

		// Save matrix state and do the custom rotation
		glPushMatrix();
		glRotatef( _theta, 1.0f, 0.0f, 0.0f );
		glRotatef( _phi,   0.0f, 0.0f, 1.0f );

		// Set drawing colour to white
		glColor3f( 1.0f, 1.0f, 1.0f );
		glVertex3f( 0, 0, 50 );
		glVertex3f( 0, 0, -50 );

		glColor3f( 1.0f, 1.0f, 1.0f );

        glBegin( GL_LINE_STRIP );
        for(int i=0; i<_activePoints.size(); i++){
            TrackPoint* point = _activePoints[i];

            float x = (point->utmX() + _xShift); //549470); //551965);
            float y = (point->utmY() + _yShift); //5277208); //5274200);
            float z = (point->elevation());


            float speed = qMin(point->speed(), (double)10.0);

            float hper = 1 - (qMax((double)0.0, qMin((double)(speed / 10.0f), 1.0)) * 0.8 );

            QColor color = QColor::fromHsvF(hper, hper, 1.0);

            glColor4f( color.redF(), color.greenF(), color.blueF(), color.alphaF());
            glVertex3f( x, y, -z * 2);
            glColor4f( 0.1, 0.1, 0.1, 0.8);
            glVertex3f( x, y, 0);


            glVertex3f( x, y, -z * 2);
        }
        glEnd();


        glBegin( GL_LINE_STRIP );
        for(int i=0; i<_inactivePoints.size(); i++){
            TrackPoint* point = _inactivePoints[i];
            float x = (point->utmX() + _xShift);
            float y = (point->utmY() + _yShift);
            float z = (point->elevation());

            float speed = qMin(point->speed(), (double)10.0);

            float hper = 1 - (qMax((double)0.0, qMin((double)(speed / 10.0f), 1.0)) * 0.8 );

            QColor color = QColor::fromHsvF(hper, hper, 0.5);

            glColor4f( color.redF(), color.greenF(), color.blueF(), 0.2);
            glVertex3f( x, y, -_zShift);

		}
        glEnd();




		// Restore the matrix state
		glPopMatrix();
	}
}

void MapWidget::keyPressEvent( QKeyEvent* event )
{
	switch ( event->key() )
	{
		case Qt::Key_Escape:
			QCoreApplication::instance()->quit();
			break;

		case Qt::Key_Space:
			startAnimation();
			break;

		case Qt::Key_S:
			_yShift -= 250;
			//updateGL();
			emit repaintRequest();
			break;
		case Qt::Key_W:
			_yShift += 250;
			//updateGL();
			emit repaintRequest();
			break;
		case Qt::Key_A:
			_xShift -= 250;
			//updateGL();
			emit repaintRequest();
			break;
		case Qt::Key_D:
			_xShift += 250;
			//updateGL();
			emit repaintRequest();
			break;
		case Qt::Key_R:
			_phi = 0.0f;
			_theta = 0.0f;
			_range = 50.0f;
			_zoom = 1;
			_xShift = -549470; //551965);
			_yShift = -5277208; //5274200);
			//updateGL();
			emit repaintRequest();
			break;

		case Qt::Key_Left:
			_phi += 3.0f;
			//updateGL();
			emit repaintRequest();
			break;

		case Qt::Key_Right:
			_phi -= 3.0f;
			//updateGL();
			emit repaintRequest();
			break;

		case Qt::Key_Up:
			_theta += 5.0f;
			//updateGL();
			emit repaintRequest();
			break;

		case Qt::Key_Down:
			_theta -= 5.0f;
			//updateGL();
			emit repaintRequest();
			break;

		case Qt::Key_Plus:
			//_lineWidth = qMin( _lineWidth + _lineWidthStep, float( _lineWidthRange.y() ) );
			_range -= 75.0f;
			resizeGL(_width, _height);
			qDebug() << "_range =" << _range;
			//updateGL();
			emit repaintRequest();
			break;

		case Qt::Key_Minus:
			//_lineWidth = qMax( _lineWidth - _lineWidthStep, float( _lineWidthRange.x() ) );
			_range += 75.0f;
			qDebug() << "_range =" << _range;
			resizeGL(_width, _height);
			//qDebug() << "_lineWidth =" << _lineWidth;
			//updateGL();
			emit repaintRequest();
			break;

		default:
			QGLWidget::keyPressEvent( event );
	}

    _inputProcessingNeeded = true;
}

void MapWidget::mousePressEvent(QMouseEvent* event){
	QVector3D worldPos = screenToWorld(event->pos());
	switch(event->button()){
		case Qt::LeftButton:
			_leftDown = true;
			_lastMousePos = worldPos.toVector2D();
			break;
		case Qt::RightButton:
			_middleDown = true;
			_lastMousePos = worldPos.toVector2D();
			break;
	}

    _inputProcessingNeeded = true;
}


void MapWidget::mouseReleaseEvent(QMouseEvent* event){
	switch(event->button()){
		case Qt::LeftButton:
			_leftDown = false;
			break;
		case Qt::RightButton:
			_middleDown = false;
			break;
	}

    _inputProcessingNeeded = true;
}


void MapWidget::mouseMoveEvent(QMouseEvent* event){
	if((event->buttons() & Qt::LeftButton) != 0 && _leftDown){
		QVector3D worldPos = screenToWorld(event->pos());

		QVector3D delta = _lastMousePos - worldPos;

		qDebug() << "left delta = (" << delta.x() << "," <<delta.y() << ") \t zoom=" << _zoom << "\t phi=" << _phi;

		_lastMousePos = worldPos;

		QVector2D pos = delta.toVector2D();
		float s = sin(_phi);
	    float c = cos(_phi);

	    // translate point back to origin:
	    //pos.setX(pos.x() + _xShift);
		//pos.setY(pos.y() + _yShift);

	    // rotate point
		float xnew = 0.0f;
		float ynew = 0.0f;

		/*if(_phi < 0.0f){
			xnew = pos.x() * c + pos.y() * s;
			ynew = -pos.x() * s + pos.y() * c;
		}
		else{*/
			xnew = pos.x() * c + pos.y() * s;
			ynew = -pos.x() * s + pos.y() * c;
		//}

	    // translate point back:
		//pos.setX(xnew );//-	 _xShift);
	    //pos.setY(ynew );//make


		/*if(qAbs(_phi) > 90){
			_xShift += delta.x(); // * (_zoom / 10); // * qCos(_phi);
			_yShift += delta.y(); // * (_zoom / 10.0f);  // * (qCos(_theta) + qSin(_phi));
		}
		else{
			_xShift -= delta.x(); // * (_zoom / 10); // * qCos(_phi);
			_yShift -= delta.y(); // * (_zoom / 10.0f);  // * (qCos(_theta) + qSin(_phi));
		}*/

		_xShift += xnew;
		_yShift += ynew;

		//updateGL();
        _inputTimer.start();
        _inputProcessingNeeded = true;
	}

	if((event->buttons() & Qt::RightButton) != 0 && _middleDown){
		QVector3D worldPos = screenToWorld(event->pos());

		QVector3D delta = _lastMousePos - worldPos;

		qDebug() << "left delta = (" << delta.x() << "," <<delta.y() << ") \t zoom=" << _zoom;

		_lastMousePos = worldPos;

		float lastTheta = _theta;

		_theta = qMin( qMax( ((-delta.y() / _height)*35.0) + _theta, 0.0), 90.0);
		_phi = _phi - ((delta.x() / _width)*35.0); //qMin( qMax(delta.x() + _phi, -180.0), 180.0);

		if(lastTheta < 20 && _theta >= 20){
			startAnimation(1000);
		}
		else if(_theta < 30){
			_heightScale = 0.01;
		}

		//updateGL();
		//emit repaintRequest();
		_inputTimer.start();
        _inputProcessingNeeded = true;
	}

}
void MapWidget::wheelEvent(QWheelEvent* event){
    if( (event->modifiers() & Qt::ControlModifier) != 0x0){
        //Zoom on time axis

        double sign = copysign(1, (double) event->delta());

        if((_activeTimeRangeSec/secondsPerDay) < 2 || ((_activeTimeRangeSec/secondsPerDay) == 2 && sign < 0)){
            _activeTimeRangeSec += (qint64) ((event->delta() / 120.0f ) * secondsPerHour );
            _activeTimeRangeSec = qMax(_activeTimeRangeSec, secondsPerHour);

            qDebug() << "_activeTimeRangeSec=" << (_activeTimeRangeSec/secondsPerHour) << "hours delta=" << event->delta();
        }
        else{
            _activeTimeRangeSec += (((event->delta() / 120.0f ) * secondsPerDay ) / secondsPerDay) * secondsPerDay;
            _activeTimeRangeSec = qMax(_activeTimeRangeSec, secondsPerDay);

            qDebug() << "_activeTimeRangeSec=" << (_activeTimeRangeSec/secondsPerDay) << "days delta=" << event->delta() << " sign=" << sign;
        }

        _dataUpdateNeeded=true;
    }
    else if( (event->modifiers() & Qt::AltModifier) != 0x0){
        //Scroll on time axis

        qreal newStartMs = activeStartTime().toMSecsSinceEpoch();

        //Scroll time by 25% of time range
        newStartMs += (_activeTimeRangeSec * 250 ) * (event->delta() / 120.0);

        QDateTime startTime = QDateTime::fromMSecsSinceEpoch(newStartMs);

        setActiveStartTime(startTime);

        qDebug() << "activeStartTime=" << activeStartTime().toString() << " activeEndTime=" << activeEndTime().toString() << " delta=" << event->delta();

        _dataUpdateNeeded=true;
    }
    else if(_zoom > 100){
		_zoom = qMax(2.0, _zoom - (event->delta() / 20.0));
        qDebug() << "_zoom =" << _zoom;
        resizeGL(_width, _height);
        _range = _zoom * 50;
	}
	else{
		_zoom = qMax(3.0, _zoom - (event->delta() / 100.0));
        qDebug() << "_zoom =" << _zoom;
        resizeGL(_width, _height);
        _range = _zoom * 50;
	}



	//updateGL();
	//emit repaintRequest();
	_inputTimer.start();
    _inputProcessingNeeded = true;
}


QDateTime MapWidget::activeEndTime() const {
    return _activeEndTime;
}


QDateTime MapWidget::activeStartTime() const{
    return _activeEndTime.addSecs(-_activeTimeRangeSec);
}
