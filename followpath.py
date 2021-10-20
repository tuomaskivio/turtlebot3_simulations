#!/usr/bin/env python
import gotogoal
import sys
import rospy

def parse_path(file_location):
    f = open(file_location, "r")
    path = f.read()
    path = path.split("---\n")
    path.pop()
    for (i, pose) in enumerate(path):
        pose = pose.split('\n')
        pose = {'x': float(pose[0].split(': ')[1]),
                'y': float(pose[1].split(': ')[1]),
                'theta': float(pose[2].split(': ')[1])}
        path[i] = pose
    return path
        
if __name__ == '__main__':
    path = parse_path(sys.argv[1])
    try:
        x = gotogoal.TurtleBot()
        for pose in path:
            print(pose)
            x.move2goal(pose[x], pose[y], pose[theta])
    except rospy.ROSInterruptException:
        pass
