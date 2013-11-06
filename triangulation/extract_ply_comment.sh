#!/bin/sh
case $1 in -x) set -x;shift;;esac

for f in $*;do
    s=`head -3 $f | fgrep comment | sed 's/^comment triangclean //'`
    if [ -n "$s" ]; then
	echo "$f : $s"
    fi
done
