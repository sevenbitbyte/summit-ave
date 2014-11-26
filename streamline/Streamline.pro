#-------------------------------------------------
#
# Project created by QtCreator 2013-03-15T10:52:12
#
#-------------------------------------------------

QT       += core gui opengl xml sql network

greaterThan(QT_MAJOR_VERSION, 5): QT += widgets

TARGET = Streamline
TEMPLATE = app

OBJECTS_DIR = build/obj
MOC_DIR = build/moc
UI_DIR = build/ui

LIBS += -lqjson -lGLU

CONFIG += debug_and_release

SOURCES += src/main.cpp\
        src/ui/mainwindow.cpp \
		src/ui/mapwidget.cpp src/gps/trackpoint.cpp \
		src/gps/UTM/conversion.cpp \
    src/contentmanager.cpp \
    src/datastore/database.cpp \
    src/datastore/datum.cpp \
    src/datastore/table.cpp \
    src/datastore/types/locationdatum.cpp \
    src/datastore/types/locationsegment.cpp \
    src/datastore/types/filerecord.cpp

HEADERS  += src/ui/mainwindow.h \
		src/ui/mapwidget.h src/gps/trackpoint.h \
		src/gps/UTM/constants.h  src/gps/UTM/conversion.h \
    src/contentmanager.h \
    src/datastore/database.h \
    src/datastore/datum.h \
    src/datastore/table.h \
    src/datastore/types/locationdatum.h \
    src/debug.h \
    src/datastore/types/locationsegment.h \
    src/datastore/types/filerecord.h



FORMS    += ui/mainwindow.ui

INCLUDEPATH += ./ src/
