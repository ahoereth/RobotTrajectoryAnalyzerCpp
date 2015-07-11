#!/bin/bash

# Topic name.
if [ -n "$1" ] ; then
  topic=$1
else
  echo "topic argument missing"
  exit 2
fi

# Topic data size.
if [ -n "$2" ] ; then
  size=$2
else
  echo "array length argument missing"
  exit 2
fi

# Build plot command.
plotcommand="rqt_plot "
for i in `seq 0 $size` ; do
  plotcommand="$plotcommand /$topic[$i]"
done

# Plot.
eval $plotcommand
