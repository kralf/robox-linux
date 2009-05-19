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

# Unmount filesystem image
# See usage for a description of the arguments

. functions/global.sh

UMNTFS="root"

script_init "unmount filesystem image" "FS" "$UMNTFS" \
  "the filesystem to be unmounted root|boot|..."

script_setopt "--image" "FILE" "UMNTIMG" "" \
  "filesystem image to be unmounted"
script_setopt "--mount-point" "DIR" "UMNTPOINT" "" \
  "mount point of the filesystem"

script_checkopts $*
script_checkroot

[ -z "$UMNTIMG" ] && UMNTIMG=".bootrd.images/*/${FS}fs.img"
[ -z "$UMNTPOINT" ] && UMNTPOINT=".${FS}fs.mount"

fs_umountimg $UMNTIMG $UMNTPOINT

log_clean