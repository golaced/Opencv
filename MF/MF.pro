TEMPLATE = app
CONFIG += console c++11
CONFIG += -lstdc++
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    src/px4/src/flow_opencv.cpp \
    src/px4/src/trackFeatures.cpp \
    src/aruco.cpp \
    src/AttitudePosition.cpp \
    src/MarkerWorldCoornidate.cpp \
    src/inifile.cpp \
    src/AM/AM.cpp \
    src/AM/rt_nonfinite.cpp \
    src/AM/rtGetInf.cpp \
    src/AM/rtGetNaN.cpp \
    src/AM/AM_initialize.cpp \
    src/AM/AM_rtwutil.cpp \
    src/AM/AM_terminate.cpp \
    src/AM/cosd.cpp \
    src/AM/sind.cpp \
    src/AM/AM_data.cpp \
    src/AM/bsearch.cpp \
    src/AM/eml_rand.cpp \
    src/AM/eml_rand_mcg16807_stateful.cpp \
    src/AM/eml_rand_mt19937ar_stateful.cpp \
    src/AM/eml_rand_shr3cong_stateful.cpp \
    src/AM/isfinite.cpp \
    src/AM/kmeans.cpp \
    src/AM/rand.cpp \
    src/AM/randi.cpp


include(deployment.pri)
qtcAddDeployment()

INCLUDEPATH += /usr/include/opencv\
               /home/pi/QT/MF/src/px4/include\
               /home/pi/QT/MF/src/AM\
               /home/pi/QT/MF/src/eigen3/eigen

LIBS    += -lopencv_highgui \
            -lopencv_photo \
            -lopencv_calib3d \
            -lopencv_imgproc \
            -lopencv_stitching \
            -lopencv_contrib \
            -lopencv_legacy \
            -lopencv_superres \
            -lopencv_core \
            -lopencv_ml \
            -lopencv_video \
            -lopencv_features2d \
            -lopencv_videostab \
            -lopencv_flann \
            -lopencv_objdetect \
            -lopencv_gpu \
            -lopencv_ocl \
            -laruco \
            -lpthread\
            -lserial\
            -lwiringPi\


HEADERS += \
    src/px4/include/flow_opencv.hpp \
    src/px4/include/optical_flow.hpp \
    src/px4/include/trackFeatures.h \
    src/AttitudePosition.h \
    src/MarkerWorldCoornidate.h \
    src/my_serial.h \
    src/inifile.h \
    src/AM/AM_initialize.h \
    src/AM/AM_rtwutil.h \
    src/AM/AM_terminate.h \
    src/AM/cosd.h \
    src/AM/sind.h \
    src/AM/AM.h \
    src/AM/AM_types.h \
    src/AM/rt_nonfinite.h \
    src/AM/rtGetInf.h \
    src/AM/rtGetNaN.h \
    src/AM/rtwtypes.h \
    src/AM/AM_initialize.h \
    src/AM/AM_rtwutil.h \
    src/AM/AM_terminate.h \
    src/AM/cosd.h \
    src/AM/sind.h \
    src/AM/AM_data.h \
    src/AM/bsearch.h \
    src/AM/eml_rand.h \
    src/AM/eml_rand_mcg16807_stateful.h \
    src/AM/eml_rand_mt19937ar_stateful.h \
    src/AM/eml_rand_shr3cong_stateful.h \
    src/AM/isfinite.h \
    src/AM/kmeans.h \
    src/AM/rand.h \
    src/AM/randi.h

OTHER_FILES += \
    markerconfig.ini \
    rasp.yml \
    rasp320.yml \
    rasp_reg.yml
