LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := sign-file
LOCAL_MODULE_TAGS := optional
LOCAL_PREBUILT_EXECUTABLES := scripts/sign-file
include $(BUILD_HOST_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE := x509keyid
LOCAL_MODULE_TAGS := optional
LOCAL_PREBUILT_EXECUTABLES := scripts/x509keyid
include $(BUILD_HOST_PREBUILT)

