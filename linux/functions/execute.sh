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

# Execution functions

function execute
{
  true VERBOSE && message_start "executing commands"

  while [ -n "$1" ]; do
    true VERBOSE && message "executing \"$1\""

    log_command "$1"
    eval "$1" >> $LOGFILE 2>&1

    [ "$?" != 0 ] && message_exit "failed to execute command \"$1\""
    shift
  done

  true VERBOSE && message_end "success, all commands executed"
}

function execute_if
{
  EXECCOND=$1
  shift

  true EXECCOND && execute "$*"
}

function execute_root
{
  ROOT=$1
  shift

  true VERBOSE && message_start "executing commands in $ROOT"

  while [ -n "$1" ]; do
    true VERBOSE && message "executing \"$1\""

    log_command "$1"
    chroot $ROOT sh -l -c "$1" >> $LOGFILE 2>&1

    [ "$?" != 0 ] && message_exit "failed to execute command \"$1\""
    shift
  done

  true VERBOSE && message_end "success, all commands executed"
}
