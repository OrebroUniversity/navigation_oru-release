#!/usr/bin/env python

# (c) 2018 EU H2020 Project ILIAD, iliad-project.eu
# The following code in this file is licensed under the MIT license:
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import tf, rospy, rospkg, os, os.path, sys, time, re, yaml
import tf.transformations

rospy.init_node("save_calibration")
rospack = rospkg.RosPack()

robot_prefix = "(robot.*)"
start_link_suffix = "_calib_start_link"
end_link_suffix = "_link"


tf_listener = tf.TransformListener()
rospy.loginfo("Waiting for transforms for 5 seconds... if you do not have a backup of your old calibration, CTRL+C now!")
time.sleep(5.0)

# Retrieve list of all TF frames
frames = tf_listener.getFrameStrings()
sensors = []
robot_name = None

# Look for sensors with calibration joints (if there is a _calib_start link)
for frame in frames:
    match = re.match("%s/(.*)%s" % (robot_prefix, start_link_suffix), frame)
    if match:
        curr_robot_name = match.group(1)
        if not robot_name:
            robot_name = curr_robot_name
        elif curr_robot_name != robot_name:
            rospy.logwarn("Multiple robots are active, this is not supported during calibration!")
            sys.exit(1)

        sensor_name = match.group(2)
        sensors += [ sensor_name ]

if not sensors:
    rospy.logwarn("Did not find any sensors with calibration joints! Are robot_state_publisher and joint_state_publisher running?")
    sys.exit(1)
else:
    rospy.loginfo("Found %d sensors with calibration joints for %s: %s" % (len(sensors), robot_name, str(sensors)))

# For each sensor, look up its transforms
rospy.loginfo("Looking up sensor transforms...")
print ""

joint_values_key = "zeros"
resulting_tf_key = "resulting_transforms"
result_dict = { joint_values_key : {}, resulting_tf_key : {} }

for sensor in sensors:
    # Walk through its transformation chain
    start_link = "%s/%s%s" % (robot_name, sensor, start_link_suffix)
    end_link = "%s/%s%s" % (robot_name, sensor, end_link_suffix)
    chain = tf_listener.chain(end_link, rospy.Time(0), start_link, rospy.Time(0), start_link)
    #print "Chain: " + str(chain)

    for i in range(1, len(chain)):
        from_link = chain[i-1]
        to_link = chain[i]
        
        match = re.match("%s/(.*_calib_.*)_link" % robot_name, to_link)
        if match:  # Won't happen for final dummy transform to sensor link
            joint_name = match.group(1)
            trans, rot = tf_listener.lookupTransform(from_link, to_link, rospy.Time(0))
            xyzrpy = list(trans) + list(tf.transformations.euler_from_quaternion(rot))

            # Assume max value in xyzrpy is the one that is important, because all other ones should be zero
            max_abs_val = max(xyzrpy, key=abs)
            if abs(max_abs_val) > 0.001:  # Only write non-zero values
                result_dict[joint_values_key][joint_name] = max_abs_val
                print " * %s: %s" % (joint_name, "%.3f" % max_abs_val)

    # Lookup final transform (all-in-one)
    trans, rot = tf_listener.lookupTransform(start_link, end_link, rospy.Time(0))
    xyzrpy = list(trans) + list(tf.transformations.euler_from_quaternion(rot))
    result_dict[resulting_tf_key][sensor] = xyzrpy
    print '=> %s: %s' % (sensor, " ".join(["%.3f" % val for val in xyzrpy]))
    print '-' * 80

calib_filename = os.path.join(rospack.get_path("cititruck_description"), "calibration", robot_name + "_calib.yaml")
rospy.loginfo("Saving calibration to: " + calib_filename)
with open(calib_filename, 'w') as outfile:
    outfile.write("# This file was auto-generated by save_calibration.py! Do not edit the '%s' field by hand!\n" % resulting_tf_key)
    outfile.write("# The content of the '%s' field, i.e. the joint values, can be adjusted at runtime using the joint_state_publisher GUI!\n" % joint_values_key)
    yaml.dump(result_dict, outfile, default_flow_style=False)