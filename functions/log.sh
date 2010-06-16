#!/bin/bash
############################################################################
#    Copyright (C) 2007 by Ralf 'Decan' Kaestner                           #
#    ralf.kaestner@gmail.com                                               #
#                                                                          #
#    This program is free software; you can redistribute it and#or modify  #
#    it under the terms of the GNU General Public License as published by  #
#    the Free Software Foundation; either version 2 of the License, or     #
#    (at your option) any later version.                                   #
#                                                                          #
#    This program is distributed in the hope that it will be useful,       #
#    but WITHOUT ANY WARRANTY; without even the implied warranty of        #
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
#    GNU General Public License for more details.                          #
#                                                                          #
#    You should have received a copy of the GNU General Public License     #
#    along with this program; if not, write to the                         #
#    Free Software Foundation, Inc.,                                       #
#    59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             #
############################################################################

# Log settings

! defined LOGILE && fs_abspath "`basename $0 .sh`.log" LOGFILE
VERBOSE="false"

function log_command
{
  echo "COMMAND: $1" > $LOGFILE
  echo "INVOKED BY: $SCRIPT" >> $LOGFILE
  echo "INVOKED IN: `pwd`" >> $LOGFILE
  echo "TIMESTAMP: `date`" >> $LOGFILE
  echo -n "ENVIRONMENT: " >> $LOGFILE
  echo `printenv` >> $LOGFILE
  echo -n "****************************************" >> $LOGFILE
  echo "****************************************" >> $LOGFILE
}

function log_clean
{
  if [ -e "$LOGFILE" ]; then
    true VERBOSE && message "cleaning up log file"
    rm $LOGFILE
  fi
}
