#!/bin/tcsh -f

set name=`date +robox_linux-%Y%m%d.tar.bz2`

\rm ${name}

tar --exclude-from=exclude.txt -cvjf ${name} *
