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

# Xen virtualization functions

function xen_bootimg
{
  DOMAINNAME=$1
  DOMAINCFG=$2
  KERNELIMG=$3
  fs_abspath $4 ROOTFSIMG

  message_start "booting kernel image $KERNELIMG"
  if [ -e "$KERNELIMG" ]; then
    message_start "creating domain $DOMAINNAME"

    message "creating domain $1"
    xm create -c $DOMAINCFG name=$DOMAINNAME kernel=$KERNELIMG \
      disk=file:$ROOTFSIMG,hda1,w

    [ $? != 0 ] && message "failed to create domain $DOMAINNAME"
  else
    exit_message "missing kernel image"
  fi
}
