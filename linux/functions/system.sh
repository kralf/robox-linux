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

# System functions

NULL="/dev/null"

function system_mkgroups
{
  ROOT=$1
  shift
  message_start "making groups in $ROOT"

  while [ -n "$1" ]; do
    message_start "adding group $1"

    execute_root $ROOT "groupadd --gid $2 $1"

    message_end
    shift
    shift
  done

  message_end "success, groups made"
}

function system_mkusers
{
  ROOT=$1
  shift
  message_start "making users in $ROOT"

  while [ -n "$1" ]; do
    message_start "adding user $1"

    PASSWD=`mkpasswd --hash=md5 $6`
    execute_root $ROOT "useradd -u $2 -g $3 -m -d $4 -s $5 -p $PASSWD $1"
    [ -d $4 ] && execute_root "chown -R $2:$3 $4"

    message_end
    shift
    shift
    shift
    shift
    shift
    shift
  done

  message_end "success, users made"
}
