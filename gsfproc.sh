#!/bin/bash
rm mbproccmds
for i in `ls $1/mb/*.gsf`; do 
    echo "mbgsf_to_xyz -start $3 -stop $4 $i" >> mbproccmds
done
$2/runtp.py mbproccmds $5
