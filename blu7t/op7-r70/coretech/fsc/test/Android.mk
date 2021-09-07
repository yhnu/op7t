LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := fsc-unit-tests
LOCAL_MODULE_TAGS := tests
LOCAL_CFLAGS = -Wall -Wextra -Werror
LOCAL_SHARED_LIBRARIES := libcutils libbase
LOCAL_SRC_FILES := fsc_test.cpp
include $(BUILD_NATIVE_TEST)
