cmake_minimum_required(VERSION 3.22)

#
# Multi-context project for STM32WL5 (CM4 + CM0PLUS)
#

# Set the project name
set(CMAKE_PROJECT_NAME Lora)
project(${CMAKE_PROJECT_NAME})

# Include ExternalProject to build each core context
include(ExternalProject)
include("mx-generated.cmake")
