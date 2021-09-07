LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := perf_event_test
LOCAL_CFLAGS = -Wall -Wextra -O0
LOCAL_SHARED_LIBRARIES := libcutils libbase
LOCAL_SRC_FILES := perf_event_test.cpp
include $(BUILD_EXECUTABLE)

include $(CLEAR_VARS)
LOCAL_MODULE := perf_event_monitor
LOCAL_CFLAGS = -Wall -Wextra -O0
LOCAL_SHARED_LIBRARIES := libcutils libbase
LOCAL_SRC_FILES := perf_event_monitor.cpp
include $(BUILD_EXECUTABLE)

include $(CLEAR_VARS)
LOCAL_MODULE := perf_ioctl
LOCAL_CFLAGS = -Wall -Wextra -O0
LOCAL_SHARED_LIBRARIES := libcutils libbase
LOCAL_SRC_FILES := perf_ioctl.cpp
include $(BUILD_EXECUTABLE)

include $(CLEAR_VARS)
LOCAL_MODULE := stress_boost
LOCAL_CFLAGS = -Wall -Wextra -O0
LOCAL_SHARED_LIBRARIES := libcutils libbase
LOCAL_SRC_FILES := stress_boost.cpp
include $(BUILD_EXECUTABLE)
