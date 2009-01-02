#!/bin/bash
ALT_THRESH=10.0
BACKGROUND="-K /home/mkj/datasets/grd_5m/test3.grd"
#"-K 
HIALT_RES=1.0/1.0/m
function float_cond()
{
    local cond=0
    if [[ $# -gt 0 ]]; then
        cond=$(echo "$*" | bc -q 2>/dev/null)
        if [[ -z "$cond" ]]; then cond=0; fi
        if [[ "$cond" != 0  &&  "$cond" != 1 ]]; then cond=0; fi
    fi
    local stat=$((cond == 0))
    return $stat
}
grdfolder=`find . -type d |grep GRD_  | sort|head`
hialt=
for i in *.alt; do
   alt=`cat $i`
   if float_cond "$alt > $ALT_THRESH";then
       hialt="$hialt ${i%.alt}.gsf"
   fi
done
rm -f hialtlst
for i in $hialt; do
echo $i >> hialtlst
done
mbdatalist -F-1 -I hialtlst > hialtgsflist
mbm_grid -F -1 -I hialtgsflist  -E $HIALT_RES $BACKGROUND -O blendedhi
./blendedhi_mbgrid.cmd
./blendedhi.grd.cmd
#echo $hialt

