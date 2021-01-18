#-------------------------------------------------
#
# Project created by QtCreator 2015-11-06T11:49:35
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = TrafficLight_v4
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += main.cpp \
    visualizer.cpp \
    trafficligthdetection.cpp \
    gisretrieval.cpp \
    detection.cpp \
    dataset.cpp \
    dataloader.cpp

HEADERS += \
    visualizer.h \
    trafficligthdetection.h \
    SensorsData.h \
    gisretrieval.h \
    detection.h \
    dataset.h \
    DataLoader.h \
    Database.h
