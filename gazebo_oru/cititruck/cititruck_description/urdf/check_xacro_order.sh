rosrun xacro xacro cititruck.xacro > /tmp/old.xml
rosrun xacro xacro --inorder cititruck.xacro > /tmp/new.xml
diff /tmp/old.xml /tmp/new.xml
