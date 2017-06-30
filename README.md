ROS2 for Java
=============

Build status
------------

**JRE** [![Build Status](https://travis-ci.org/esteve/ros2_java.svg?branch=master)](https://travis-ci.org/esteve/ros2_java)

**Android** [![Build Status](https://travis-ci.org/esteve/ros2_android.svg?branch=master)](https://travis-ci.org/esteve/ros2_android)

Introduction
------------

This is a set of projects (bindings, code generator, examples and more) that enables developers to write ROS2
applications for the JVM and Android.

Besides this repository itself, there's also:
- https://github.com/esteve/ament_java, which adds support for Gradle to Ament
- https://github.com/esteve/ament_gradle_plugin, a Gradle plugin that makes it easier to use ROS2 in Java and Android project. The Gradle plugin can be installed from Gradle Central https://plugins.gradle.org/plugin/org.ros2.tools.gradle
- https://github.com/esteve/ros2_java_examples, examples for the Java Runtime Environment
- https://github.com/esteve/ros2_android_examples, examples for Android

Is this Java only?
------------------

No, any language that targets the JVM can be used to write ROS2 applications.

Including Android?
------------------

Yep! Make sure to use Fast-RTPS as your DDS vendor and at least [this revision](https://github.com/eProsima/Fast-RTPS/commit/5301ef203d45528a083821c3ba582164d782360b).

Features
--------

The current set of features include:
- Generation of all builtin and complex ROS types, including arrays, strings, nested types, constants, etc.
- Support for publishers and subscriptions
- Tunable Quality of Service (e.g. lossy networks, reliable delivery, etc.)
- Clients and services
- Timers
- Composition (i.e. more than one node per process)
- Time handling (system and steady, ROS time not yet supported https://github.com/ros2/ros2/issues/350)
- Support for Android

Sounds great, how can I try this out?
-------------------------------------

First of all, download the ament repositories in a separate workspace:

```
mkdir -p ~/ament_ws/src
cd ~/ament_ws
wget https://raw.githubusercontent.com/esteve/ament_java/master/ament_java.repos
vcs import ~/ament_ws/src < ament_java.repos
src/ament/ament_tools/scripts/ament.py build --symlink-install --isolated
```

We need to split the build process between Ament and the rest of `ros2_java` workspace so that the additional build type for Gradle projects is picked up by Ament.

The following sections deal with building the `ros2_java` codebase for the desktop Java runtime and for Android.

Desktop
-------

```
mkdir -p ~/ros2_java_ws/src
cd ~/ros2_java_ws
wget https://raw.githubusercontent.com/esteve/ros2_java/master/ros2_java_desktop.repos
vcs import ~/ros2_java_ws/src < ros2_java_desktop.repos
cd ~/ros2_java_ws/src/ros2/rosidl_typesupport
patch -p1 < ../../ros2_java/ros2_java/rosidl_typesupport_ros2_java.patch
cd ~/ros2_java_ws
. ~/ament_ws/install_isolated/local_setup.sh
ament build --symlink-install --isolated
```

Now you can just run a bunch of examples, head over to https://github.com/esteve/ros2_java_examples for more information.

Android
-------

The Android setup is slightly more complex, you'll need the SDK and NDK installed, and an Android device where you can run the examples.

Make sure to download at least the SDK for Android Lollipop (or greater), the examples require the API level 21 at least and NDK 14.

You may download the Android NDK from [the official](https://developer.android.com/ndk/downloads/index.html) website, let's assume you unpack it to `~/android_ndk`

```
mkdir -p ~/ros2_android_ws/src
cd ~/ros2_android_ws
wget https://raw.githubusercontent.com/esteve/ros2_java/master/ros2_java_android.repos
vcs import ~/ros2_android_ws/src < ros2_java_android.repos
cd ~/ros2_android_ws/src/ros2/rosidl
touch python_cmake_module/AMENT_IGNORE
touch rosidl_generator_py/AMENT_IGNORE
cd ~/ros2_android_ws/src/ros2/rosidl_typesupport
patch -p1 < ../../ros2_java/ros2_java/rosidl_typesupport_ros2_android.patch
cd ~/ros2_android_ws
. ~/ament_ws/install_isolated/local_setup.sh
ament build --isolated --cmake-args \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 \
  -DEPROSIMA_BUILD=ON \
  -DCMAKE_FIND_ROOT_PATH="$HOME/ament_ws/install_isolated;$HOME/ros2_android_ws/install_isolated" \
  -DANDROID_FUNCTION_LEVEL_LINKING=OFF \
  -DANDROID_TOOLCHAIN_NAME=arm-linux-androideabi-clang -DANDROID_STL=gnustl_shared \
  -DANDROID_ABI=armeabi-v7a \
  -DANDROID_NDK=$HOME/android_ndk/android-ndk-r14 \
  -DANDROID_NATIVE_API_LEVEL=android-21 \
  -DCMAKE_TOOLCHAIN_FILE=$HOME/android_ndk/android-ndk-r14/build/cmake/android.toolchain.cmake \
  -- \
  --ament-gradle-args \
  -Pament.android_stl=gnustl_shared -Pament.android_abi=armeabi-v7a -Pament.android_ndk=$HOME/android_ndk/android-ndk-r14 --
```

You can find more information about the Android examples at https://github.com/esteve/ros2_android_examples
