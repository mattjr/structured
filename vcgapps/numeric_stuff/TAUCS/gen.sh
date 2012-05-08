#!/bin/sh

rm -f *.c

for i in `ls generic/*.inc`
do
   f=`basename $i .inc`
   echo '#define TAUCS_CORE_GENERAL' > $f".c"
   echo '#include "'$i'"' >> $f".c"
   echo '#define TAUCS_CORE_DOUBLE' > $f"_D.c"
   echo '#include "'$i'"' >> $f"_D.c"
done
