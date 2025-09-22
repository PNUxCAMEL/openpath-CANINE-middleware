cmake_minimum_required(VERSION 3.21)
project(Camel-GUI)

# Qt 모듈 설정
find_package(Qt5 COMPONENTS Core Network OpenGL Gui Widgets PrintSupport REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenGL REQUIRED)

# Qt 자동 처리 설정
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# 빌드 옵션
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
add_compile_definitions(GUI_RSC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources/")
if(BUILD_CONSOLE_STANDALONE)
    add_compile_definitions(BUILD_STANDALONE)
endif()


# UI 파일 처리 (수동 설정)
set(FORMS
        gui/mainwindow.ui
        gui/MotorDialog.ui
        # this code is custom widget example code.
)

# 리소스 파일 처리
set(RESOURCES
        resources/icons.qrc
        resources/stl.qrc
)
qt5_add_resources(RCC_FILES ${RESOURCES})

# 소스 파일 설정
set(SOURCES
        CommunicationClient.cpp
        main.cpp
        gui/mainwindow.cpp
        gui/MotorDialog.cpp
        gui/Qtplot/qcustomplot.cpp
        # this code is custom widget example code.
)

# 헤더 파일 설정
set(HEADERS
        gui/mainwindow.h
        gui/MotorDialog.h
        # this code is custom widget example code.
        IndexNotation.h
        CommunicationClient.h
        SharedMemory.h
)

# 실행 파일 생성
add_executable(${PROJECT_NAME}
        ${SOURCES}
        ${HEADERS}
        ${UIC_FILES}
        ${RCC_FILES}
)

target_include_directories(${PROJECT_NAME} PRIVATE
        ${CMAKE_BINARY_DIR}/GUI            # 자동 생성된 헤더 파일 경로
        ${CMAKE_CURRENT_SOURCE_DIR}/gui/Qtplot/
        ${CMAKE_CURRENT_SOURCE_DIR}/gui/RBModel/
        ${CMAKE_CURRENT_SOURCE_DIR}
)

# 타겟 설정
target_link_libraries(${PROJECT_NAME} PRIVATE
        Qt5::Core
        Qt5::Network
        Qt5::OpenGL
        Qt5::Gui
        Qt5::Widgets
        Qt5::PrintSupport
        Eigen3::Eigen
        CAMEL_SDK
        OpenGL::GL
)
