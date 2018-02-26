MKFILE_DIR := $(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))

BUILD_ROOT := $(MKFILE_DIR)/../../../quest/
