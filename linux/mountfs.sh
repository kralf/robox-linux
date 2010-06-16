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

# Mount filesystem image
# See usage for a description of the arguments

. ubash

script_init "Mount filesystem image" "FS" MNTFS "root" \
  "the filesystem to be mounted root|boot|..."

script_setopt "--image" "FILE" MNTIMG "" "filesystem image to be mounted"
script_setopt "--mount-point" "DIR" MNTPOINT "" "mount point of the filesystem"

script_setopt "--target" "i686|powerpc|..." "MNTTARGET" "`uname -m`" \
  "filesystem image target architecture"

script_checkopts $*
script_checkroot

[ -z "$MNTIMG" ] && MNTIMG=".bootrd.images/$MNTTARGET/${MNTFS}fs.img"
[ -z "$MNTPOINT" ] && MNTPOINT=".${MNTFS}fs.mount"

fs_mountimg $MNTIMG $MNTPOINT

log_clean
