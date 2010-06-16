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

# Install functions

function install_dirs
{
  CPOPTS="-dR --preserve=mode,timestamps --remove-destination --parents"
  
  ROOT=$1
  shift
  message_start "installing directory content to $ROOT"

  while [ -n "$1" ]; do
    INSTDIRS="`find $1 -maxdepth 0 2> $NULL`"

    for INSTDIR in $INSTDIRS; do
      message_start "installing directory $INSTDIR"

      execute "cp $CPOPTS $INSTDIR $ROOT"

      message_end
    done

    shift
  done

  message_end "success, directory content installed"
}

function install_objects
{
  ROOT=$1
  shift
  message_start "installing objects to $ROOT"

  while [ -n "$1" ]; do
    OBJS=$1
    ! [ -e "$OBJS" ] && OBJS=`which $1`
    ! [ -e "$OBJS" ] && OBJS=`ldconfig -p | grep $1 | grep -o "/.*$"`

    for OBJ in $OBJS; do
      if [ -e "$OBJ" ]; then
        message_start "installing object $OBJ"
    
        CPOBJFILES="$OBJ=-L"
  
        LIBS=`ldd $OBJ 2> $NULL | 
          sed s/'[[:space:]]*'// | 
          sed s/'.*=>[[:space:]]*'// |
          sed s/'[[:space:]]*(.*)'// | 
          sed s/'not a dynamic executable'//`
        for LIB in $LIBS; do
          [ -e "$LIB" ] && CPOBJFILES="$CPOBJFILES $LIB=-L"
        done

        fs_cpfiles $ROOT $CPOBJFILES

        message_end
      else
        warn_message "object $OBJ does not exist"
      fi
    done

    shift
  done

  message_end "success, objects installed"
}

function install_packages
{
  ROOT=$1
  shift
  message_start "installing packages to $ROOT"

  while [ -n "$1" ]; do
    keyval "$1" PKGNAME PKGARGS

    PKGINSTALLED=`dpkg --root=$ROOT -l $PKGNAME 2> $NULL | grep "^ii"`

    if [ -z "$PKGINSTALLED" ]; then
      message "installing package $PKGNAME"

      APTGET="/usr/bin/apt-get"
      APTENV=""
      APTOPTS="--yes --allow-unauthenticated"
      DPKGOPTS=""

      if ! [ -x "$ROOT$APTGET" ]; then
        abs_path ./cache
        DPKGCACHE=$ABSPATH

        if ! [ -d "$DPKGCACHE" ]; then
          execute "mkdir -p $DPKGCACHE/archives/partial"
        fi

        DPKGOPTS="$DPKGOPTS -o Dir::Cache=$DPKGCACHE"
        DPKGOPTS="$DPKGOPTS -o DPkg::Options::=--root=$ROOT"
        DPKGOPTS="$DPKGOPTS -o Dir::State::Status=$ROOT/var/lib/dpkg/status"
      fi

      DPKGOPTS="$DPKGOPTS -o DPkg::Options::=--force-confdef"
      if [[ "$PKGARGS" =~ noconf ]]; then
        DPKGOPTS="$DPKGOPTS -o DPkg::Options::=--unpack"
      fi

      APTCMD="$APTENV $APTGET install $APTOPTS $DPKGOPTS $PKGNAME"
      APTINPUT=`echo $PKGARGS | grep -o "\[.*\]" | sed s/"\["// | sed s/"\]"//`
      if [ -n "$APTINPUT" ]; then
        export "DEBIAN_FRONTEND=teletype"
        APTCMD="echo $APTINPUT | sed s/':'/'\n'/g | $APTCMD"
      else
        export "DEBIAN_FRONTEND=noninteractive"
      fi

      if [[ "$PKGARGS" =~ verbose ]]; then
        [ -x "$ROOT$APTGET" ] && eval "chroot $ROOT sh -l -c \"$APTCMD\""
        [ -x "$ROOT$APTGET" ] || eval "$APTCMD"
      else
        [ -x "$ROOT$APTGET" ] && execute_root $ROOT "$APTCMD"
        [ -x "$ROOT$APTGET" ] || execute "$APTCMD"
      fi
    
      [ $? != 0 ] && message_exit "failed to install package $PKGNAME"
    else
      message "package $PKGNAME is already installed"
    fi

    shift
  done

  message_end "success, packages installed"
}

function install_template
{
  message_start "installing template $1"

  TMPLVARS=`grep -o "\\\$[A-Z]\+" $1 | sed s/"^\\\\\$"// | sort -u`
  TMPLCMD="cat $1"
  TMPLWARN=$3

  for TMPLVAR in $TMPLVARS; do
    if defined $TMPLVAR; then
      TMPLVAL=`eval echo \\\$$TMPLVAR`
      TMPLCMD="$TMPLCMD | sed -e s:\"\\\$$TMPLVAR$\":\"$TMPLVAL\":g"
      TMPLCMD="$TMPLCMD | sed -e \"s:\\\$$TMPLVAR\([^A-Z]\):$TMPLVAL\1:g\""
    else
      ! false TMPLWARN && message_warn "variable $TMPLVAR is undefined"
    fi
  done
  TMPLCMD="$TMPLCMD > $2"

  execute "$TMPLCMD"

  message_end
}

function install_svn
{
  CPOPTS="-dR --preserve=mode,timestamps --remove-destination --parents"
  FINDOPTS="-depth -name .svn -exec rm -rf '{}' \;"
  
  ROOT=$1
  shift
  message_start "installing svn content to $ROOT"

  while [ -n "$1" ]; do
    INSTSVNDIRS="`find $1 -maxdepth 0 2> $NULL`"

    for INSTSVNDIR in $INSTSVNDIRS; do
      message_start "installing svn directory $INSTSVNDIR"

      execute "cp $CPOPTS $INSTSVNDIR $ROOT"
      execute "find $ROOT/$INSTSVNDIR $FINDOPTS"

      message_end
    done

    shift
  done

  message_end "success, svn content installed"
}

function install_archives
{
  ROOT=$1
  shift
  message_start "installing archives to $ROOT"

  while [ -n "$1" ]; do
    message_start "installing archive $1"

    archive_getfilter $1 FILTER
    execute "tar -x${FILTER}f $1 -C $ROOT"

    message_end
    shift
  done

  message_end "success, archives installed"
}
