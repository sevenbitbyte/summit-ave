#-------------------------------------------------
#
# Project created by QtCreator 2012-12-14T18:43:06
#
#-------------------------------------------------

QT       += core

QT       -= gui
QT		 += xml
QT       += sql

LIBS += -lqjson

TARGET = gpsfilter
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += main.cpp \
	UTM/conversion.cpp \
    trackpoint.cpp \
    locationdb.cpp

HEADERS += UTM/constants.h \
	UTM/conversion.h \
    trackpoint.h \
    locationdb.h
