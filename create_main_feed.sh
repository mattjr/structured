#!/bin/bash
 
#           DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
#                   Version 2, December 2004
#
#Copyright (C) 2004 Sam Hocevar
# 14 rue de Plaisance, 75014 Paris, France
#Everyone is permitted to copy and distribute verbatim or modified
#copies of this license document, and changing it is allowed as long
#as the name is changed.
#
#           DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
#  TERMS AND CONDITIONS FOR COPYING, DISTRIBUTION AND MODIFICATION
#
# 0. You just DO WHAT THE FUCK YOU WANT TO.
#
 
SYSDIR=$PWD
HTTPLINK="http://www-personal.acfr.usyd.edu.au/mattjr/benthos/"
FEEDTITLE="AUV Campaigns"
FEEDLINK="http://www-personal.acfr.usyd.edu.au/mattjr/benthos/"
FEEDDESC="All achived campaigns"

RSSFILE="campaigns.xml"
#DESC="`date`"
 
 
 
function testing_variables {
        if [ ! -d ${RSSDIR} ]; then
                echo -e 'ERROR: $RSSDIR does not exists!\nPlease create a directory and set the right path for $RSSDIR variable!'
                exit 1
        fi
 
        if [ ! -d ${SYSDIR} ]; then
                echo -e 'ERROR: $SYSDIR does not exists!\nPlease create a directory and set the right path for $SYSDIR variable!'
        fi
}
 
function rss_header {
### RSS HEADER
echo "<!--?xml version=\"1.0\"?-->
<rss version="\"2.0\"">
  <channel>
        <title>${FEEDTITLE}</title>
        <link>${FEEDLINK}</link>
        <description>${FEEDDESC}</description>" > $1
}
 
function rss_body {
#RSS BODY
echo "  <item>
                <title>${3}</title>
                <link>${HTTPLINK}/${2}</link>
<!--                <description>${DESC}</description> -->
        </item>" >> ${1}
}
 
function rss_footer {
### RSS FOOTER
echo "</channel></rss>" >> ${1}
}
 
 
### Main code ###
        rss_header ${RSSFILE}
for FILE in `find ./*/ -type f -name '*.xml'`; do
MI=`echo ${FILE} | cut -c 3- `
DIRN=`dirname ${MI}`
echo $DIRN
#        PARENTDIR="`dirname $FILES | awk -F "/" '{print $NF}'`"
        rss_body   ${RSSFILE} ${MI} ${DIRN}

done
        rss_footer ${RSSFILE}
