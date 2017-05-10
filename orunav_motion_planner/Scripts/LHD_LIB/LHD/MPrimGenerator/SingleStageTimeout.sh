#!/bin/bash

# Call this script with the parameters required by SingleStage

EXPECTED_ARGS=14

# check the number of arguments
if [ $# -lt $EXPECTED_ARGS ]
then
  echo "Usage: `basename $0` {arg}"
  exit 0
fi

# Absolute path to this script, e.g. /home/user/bin/foo.sh
SCRIPT=`readlink -f $0`
# Absolute path this script is in, thus /home/user/bin
SCRIPTPATH=`dirname $SCRIPT`
echo $SCRIPTPATH

# execute command and get pid
$SCRIPTPATH/SingleStage $* &
last_pid=$!

# 20 seconds timeout -- terminate script is process ends before timeout
for time in {1..10}
do
	sleep 1
	if ! kill -s 0 $last_pid 2> /dev/null 
	then
		exit
	fi
done

# the process is not over yet
if kill -s 0 $last_pid 2> /dev/null
then
	kill -TERM $last_pid   # this will kill only processes you own - if you're not privileged
	# delete the file, which might be there (incomplete)
	if [ -e ${14} ] ; then
		rm ${14}
	fi
fi
