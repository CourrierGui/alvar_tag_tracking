#!/usr/bin/python

from alvar_tag_tracking.srv import LaunchFile
from threading import Lock

import roslaunch
import rospy

launch_files = {}
to_stop = []
running = {}
mutex = Lock()


class Launcher:
    def __init__(self, uuid):
        rospy.Service("~launch", LaunchFile, self.on_launch)
        rospy.Service("~stop", LaunchFile, self.on_stop)
        self.uuid = uuid
        roslaunch.configure_logging(self.uuid)

    def on_launch(self, req):
        args = [
            req.package,
            req.filename,
            "cam_id:="+req.id,
            "main_cam:="+req.main_cam,
            "calibration_file:="+req.calibration_file
        ]
        cam_name = req.type + '_cam_' + req.id

        for i in range(len(req.args)):
            args.append(req.args[i]+":="+req.values[i])

        launch_file = roslaunch.rlutil.resolve_launch_arguments(args)[0]
        launch_args = args[2:]

        launch = roslaunch.parent.ROSLaunchParent(self.uuid, [(launch_file, launch_args)])
        mutex.acquire()
        launch_files[cam_name] = launch
        mutex.release()

        return True

    def on_stop(self, req):
        mutex.acquire()
        cam_name = req.type + '_cam_' + req.id

        to_stop.append(running[cam_name])
        del running[cam_name]

        mutex.release()
        return True


if __name__ == '__main__':
    rospy.init_node("launcher")
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    launcher = Launcher(uuid)
    rate = rospy.Rate(0.3)

    while not rospy.is_shutdown():
        mutex.acquire()
        for key, launch in launch_files.items():
            launch.start()
            del launch_files[key]
            running[key] = launch

        for launch in to_stop:
            launch.shutdown()
            to_stop.remove(launch)

        mutex.release()
        rate.sleep()
