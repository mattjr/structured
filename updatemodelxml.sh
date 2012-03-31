#!/bin/bash
COUNTER=0
mv models.xml models.xml.old
mkdir dtmp
cd dtmp
for i in `ls ../*.tar`
do
    tmp=`basename $i .tar`
    echo $tmp
    tar xf $i $tmp/$tmp.xml
    if [ -e $tmp/$tmp.xml ]
    then

	if (($COUNTER == 0 ))
	then
            cp $tmp/$tmp.xml models.xml
	else
	    sed -n "/item/,/\item/p" $tmp/$tmp.xml > item.xml
	    LASTLINE=$((`sed -n '/\item/=' models.xml | tail -n 1` +1 ))
	    LASTLINEPLUSONE=$(($LASTLINE+1))
        echo "sed -i -e '$LASTLINEPLUSONE{x;G};$LASTLINE{h;ritem.xml' -e 'd}' models.xml" > tmpcmd
	bash tmpcmd
	rm -f tmpcmd
	rm -f item.xml
	fi
	let COUNTER=COUNTER+1 
    fi
done
mv models.xml ..
cd ..
rm -rf dtmp

