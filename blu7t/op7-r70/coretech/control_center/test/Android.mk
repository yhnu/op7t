LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := cd-unit-tests
LOCAL_MODULE_TAGS := tests
LOCAL_CFLAGS = -Wall -Wextra -Werror
LOCAL_SHARED_LIBRARIES := libcutils libbase
LOCAL_SRC_FILES := cc_test.cpp
include $(BUILD_NATIVE_TEST)

include $(CLEAR_VARS)
LOCAL_MODULE := cd-test-2
LOCAL_MODULE_TAGS := tests
LOCAL_CFLAGS = -Wall -Wextra -Werror
LOCAL_SHARED_LIBRARIES := libcutils libbase
LOCAL_SRC_FILES := cc_test_2.cpp
include $(BUILD_EXECUTABLE)

include $(CLEAR_VARS)
LOCAL_MODULE := cd-test-3
LOCAL_MODULE_TAGS := tests
LOCAL_CFLAGS = -Wall -Wextra -Werror
LOCAL_SHARED_LIBRARIES := libcutils libbase
LOCAL_SRC_FILES := cc_test_3.cpp
include $(BUILD_EXECUTABLE)
