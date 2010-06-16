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

# Networking functions

function network_gethostip
{
  unset HOSTIP

  if [ -n "$HOSTNAME" ]; then
    THISHOSTNAME=`hostname -f | grep "^$1$"`
    THISHOSTALIAS=`hostname -a | grep -o "$1"`

    if [ -z "$THISHOSTNAME" ] && [ -z "$THISHOSTALIAS" ]; then
      HOSTIP=`host $1 | grep -o "[0-9]*\.[0-9]*\.[0-9]*\.[0-9]*"`
    else
      HOSTIP=`hostname -i`
    fi
  else
    HOSTIP=`hostname -i`
  fi

  [ -z "$HOSTIP" ] && message_warn "failed to determine address of host $1"

  define $2 $HOSTIP
}

function network_upfiles
{
  USER=$1
  HOST=$2
  ROOT=$3
  shift
  shift
  shift
  message_start "uploading files to $HOST"

  while [ -n "$1" ]; do
    message_start "uploading file $1"

    execute "scp $1 $USER@$HOST:$ROOT"

    message_end
    shift
  done

  message_end "success, files uploaded"
}
