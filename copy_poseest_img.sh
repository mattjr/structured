#!/bin/bash
FILES=`cut -f3-4 $1`
for i in $FILES
do
echo cp $2/$i $3 
#cp $2/$i $3 
done
