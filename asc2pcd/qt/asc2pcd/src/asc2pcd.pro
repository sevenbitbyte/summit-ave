QT += core
QT -= gui

TARGET = asc2pcd
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    asctopcd.cpp \
    arcasciiparser.cpp

HEADERS += \
    asctopcd.h \
    arcasciiparser.h
