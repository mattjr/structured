#!/bin/bash
meshTitle=`zenity --entry --title="Title" --text="Enter a title for the mesh:"`
meshDescription=`zenity --entry --title="Description" --text="Enter a description of the mesh:"`
folder=`zenity --entry --title="Folder" --text="Enter a folder name:"`

LAT=`cut -f1 -d" " ../tmp/mesh-diced/firstpt.txt`
LON=`cut -f2 -d" " ../tmp/mesh-diced/firstpt.txt`
basename=$1
reportbase="https://df.arcs.org.au/ARCS/projects/IMOS/public/AUV/WA201004/all_reports/r20100428_065237_geebank_17_29m_out_report.pdf"
URLBASE="http://www-personal.acfr.usyd.edu.au/mattjr/benthos"
#Construct the frame
header='<?xml version="1.0" encoding="UTF-8"?>\n<rss version="2.0" xmlns:geo="http://www.w3.org/2003/01/geo/wgs84_pos#">\n<channel>\n<title>%s</title>\n<link>%s</link>\n<description>%s</description>\n\t<item>\n'
acfrtitle="Benthic Models"
acfrdesc="This are the ACFR's benthic model"
titlefmt='\t\t<title>%s</title>\n'
descfmt='\t\t<description>%s</description>\n'
latfmt='\t\t<link>%f</link>\n'
latfmt='\t\t<geo:lat>%s</geo:lat>\n'
lonfmt='\t\t<geo:long>%s</geo:long>\n'
linkfmt='\t\t<link>%s/%s/%s.tar</link>\n'
reportfmt='\t\t<report>%s</report>\n'
filenamefmt='\t\t<filename>%s.tar</filename>\n'
sizefmt='\t\t<size>%d</size>\n'
folderfmt='\t\t<folder>%s</folder>\n'
footer='\t</item>\n</channel>\n</rss>\n'

#Printing
{
printf "$header" "$acfrtitle" "$URLBASE/" "$acfrdesc"
printf "$titlefmt" "$meshTitle"
printf "$descfmt" "$meshDescription"
printf "$latfmt" "$LAT"
printf "$lonfmt" "$LON"
printf "$filenamefmt" "$basename"
printf "$linkfmt" "$URLBASE" "$2" "$basename"
printf "$folderfmt" "$folder"
printf "$sizefmt" `du -b $basename.tar | cut -f 1`
printf "$reportfmt" "$reportbase"
printf "$footer"
}
#> $basefile.xml
