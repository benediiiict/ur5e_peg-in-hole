#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/ben/work/ur5e_RL/src/robotiq_gripper/robotiq_2f_gripper_control"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/ben/work/ur5e_RL/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/ben/work/ur5e_RL/install/lib/python3/dist-packages:/home/ben/work/ur5e_RL/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/ben/work/ur5e_RL/build" \
    "/usr/bin/python3" \
    "/home/ben/work/ur5e_RL/src/robotiq_gripper/robotiq_2f_gripper_control/setup.py" \
     \
    build --build-base "/home/ben/work/ur5e_RL/build/robotiq_gripper/robotiq_2f_gripper_control" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/ben/work/ur5e_RL/install" --install-scripts="/home/ben/work/ur5e_RL/install/bin"
