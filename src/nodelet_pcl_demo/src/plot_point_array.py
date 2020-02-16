#!/usr/bin/env python
import numpy as np
import rospy
from nodelet_pcl_demo.msg import PointArray
import sys


def main():
    rospy.init_node("point_array_plotter")

    # let's subscribe to a single message, plot the value, and then quit
    received = False
    for i in range(10):
        rospy.loginfo("Waiting for message")
        try:
            points = rospy.wait_for_message("cloud_points", PointArray, 1)
            received = True
            break
        except rospy.ROSException:
            pass
    if not received:
        rospy.loginfo("Never received message on 'cloud_points' topic.... exiting")
        sys.exit(1)
    rospy.loginfo("Received message!")

    # now let's turn the point arrays into numpy arrays:
    xarr = np.zeros(len(points.points))
    yarr = np.zeros(len(points.points))
    zarr = np.zeros(len(points.points))
    for i,pt in enumerate(points.points):
        xarr[i] = pt.x
        yarr[i] = pt.y
        zarr[i] = pt.z

    pltmode = rospy.get_param("~use_mayavi", False)

    if pltmode:
        from mayavi import mlab
        s = zarr
        mlab.points3d(xarr, yarr, zarr, s, colormap="RdYlBu", scale_factor=0.005,
                      scale_mode='none', mode='2dtriangle')
        mlab.show()
    else:    
        import matplotlib.pyplot as mpl
        from mpl_toolkits.mplot3d import Axes3D
        fig = mpl.figure()
        ax = Axes3D(fig)
        ax.scatter(xarr, yarr, zarr, c='r', marker='.', alpha=0.5)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        # hack to ensure equal aspect: https://stackoverflow.com/questions/13685386/matplotlib-equal-unit-length-with-equal-aspect-ratio-z-axis-is-not-equal-to
        max_range = np.array([xarr.max()-xarr.min(), yarr.max()-yarr.min(), zarr.max()-zarr.min()]).max()
        Xb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][0].flatten() + 0.5*(xarr.max()+xarr.min())
        Yb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][1].flatten() + 0.5*(yarr.max()+yarr.min())
        Zb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][2].flatten() + 0.5*(zarr.max()+zarr.min())
        for xb, yb, zb in zip(Xb, Yb, Zb):
            ax.plot([xb], [yb], [zb], 'w')
        mpl.show()

    return


if __name__ == '__main__':
    main()
