##############################################################################
# The following are set after configuration is done: 
# - OpenCV_FOUND
# - OpenCV_LIBS
# - OpenCV_INCLUDE_DIR
# - OpenCV_VERSION (OpenCV_VERSION_MAJOR, OpenCV_VERSION_MINOR, OpenCV_VERSION_PATCH)

set (OpenCV_FOUND TRUE)
set (OpenCV_INCLUDE_DIR "/usr/local/opencv-2.4.9/include")
file (GLOB OpenCV_LIBS "/usr/local/opencv-2.4.9/lib/*.a")
set (OpenCV_VERSION (2, 4, 9))