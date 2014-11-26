#-------------------------------------------------
#
# Project created by QtCreator 2013-06-22T09:00:53
#
#-------------------------------------------------

QT       += core gui widgets

TARGET = ArcAsciiToImage
CONFIG   += console
CONFIG   -= app_bundle
CONFIG += debug

OBJECTS_DIR = build/obj
MOC_DIR = build/moc
UI_DIR = build/ui

TEMPLATE = app

SOURCES += main.cpp \
    mainwindow.cpp \
    arcasciiparser.cpp \
    arcasciirenderer.cpp

HEADERS += \
    mainwindow.h \
    arcasciiparser.h \
    arcasciirenderer.h

FORMS += \
    mainwindow.ui
