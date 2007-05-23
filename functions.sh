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

#!/bin/bash

# Functions

SCRIPT="unknown script"
DOC="undocumented command"
DEFVAR=""
DEFDEFAULT=""
DEFDOC=""

ARGS=("--help"
      "--verbose|-v"
      "--logfile")
VALS=(""
      ""
      "FILE")
VARS=("HELP"
      "VERBOSE"
      "LOGFILE")
DEFAULTS=("false"
          "false"
          "`basename $0 .sh`.log")
DOCS=("display usage and exit"
      "generate verbose command output"
      "temporary log file")

if [ "$STAGE" == "" ]; then
  STAGE=0
  export STAGE
fi

function init
{
  abs_path $0
  SCRIPT=$ABSPATH
  DOC=$1
  DEFVAR=$2
  DEFDEFAULT=$3
  DEFDOC=$4
}

function calc
{
  EXPR="scale=0; ($1)/1"
  RETVAL=`echo $EXPR | bc`
}

function abs_path
{
  RETDIR=`pwd`
  cd `dirname $1`
  ABSPATH=`pwd`/`basename $1`
  cd $RETDIR
}

function echo_arg
{
  CHARS=`echo -n "$1" | wc -m`
  calc "30-$CHARS"
  BLANKS=$RETVAL
  
  echo -n "  $1"
  
  for (( B=0; B < $BLANKS; B++ )); do
    echo -n " "
  done
  
  echo "$2"
}

function echo_usage
{
  echo -n "usage: `basename $0` [OPT1 OPT2 ...]"
  if [ "$DEFVAR" != "" ]; then
    echo " [${DEFVAR}1 ${DEFVAR}2 ...]"
  else
    echo ""
  fi
  
  echo "$DOC"

  if [ "$DEFVAR" != "" ]; then
   echo_arg "${DEFVAR}1 ${DEFVAR}2 ..." "$DEFDOC"
  fi

  echo_arg "OPT1 OPT2 ..." "list of options as given below [default]"
  for (( A=0; A < ${#ARGS[@]}; A++ )); do
    ARG=${ARGS[A]}
    ARG=${ARG//"|"/", "}
    echo_arg "$ARG ${VALS[A]}" "${DOCS[A]} [${DEFAULTS[A]}]"
  done
  
  echo "Report bugs to <ralf.kaestner@gmail.com>, attach error logs"
}

function set_arg
{
  ARGS[${#ARGS[*]}]=$1
  VALS[${#VALS[*]}]=$2
  VARS[${#VARS[*]}]=$3
  DEFAULTS[${#DEFAULTS[*]}]=$4
  DOCS[${#DOCS[*]}]=$5
}

function check_args
{
  for (( A=0; A < ${#ARGS[@]}; A++ )); do
    eval "${VARS[A]}=\"${DEFAULTS[A]}\""
  done
  
  RETVAL=0

  while [ "$1" != "" ]; do
    if [[ "$1" =~ "^-" ]]; then
      MATCH=false
    
      for (( A=0; A < ${#ARGS[@]}; A++ )); do
        if [[ "$1" =~ "${ARGS[A]}" ]]; then
          if [[ "${DEFAULTS[A]}" =~ "true|false|yes|no" ]]; then
            eval "${VARS[A]}=\"true\""
            MATCH=true
          else
            eval "${VARS[A]}=\"$2\""
            MATCH=true
            shift
          fi
        fi
      done
      
      if [ "$MATCH" != "true" ]; then
        echo "unknown argument: $1"
        RETVAL=1
        HELP=true
      fi
    else
      VAR[${#VAR[*]}]=$1
    fi
    
    shift
  done
  
  abs_path $LOGFILE
  LOGFILE=$ABSPATH
  rm -f $LOGFILE

  if [ "$DEFVAR" != "" ]; then
    if [ ${#VAR[@]} != 0 ]; then
      eval "$DEFVAR=\"${VAR[@]}\""
    else
      eval "$DEFVAR=\"$DEFDEFAULT\""
    fi
  fi
  
  if [ "$HELP" == "true" ]; then 
    echo_usage
    exit $RETVAL
  fi
}

function check_uid
{
  stage_up

  if [ "$UID" != "0" ]; then
    exit_message "`basename $0` must be run as root"
  fi
  
  stage_down
}

function check_xcomp
{
  TARGET=$1
  shift

  stage_up
  message "checking the cross compiler"
  
  while [ "$1" != "" ]; do
    execute "$TARGET-linux-$1 --version"
    shift
  done
  
  stage_down
}

function check_image
{
  stage_up
  message "checking filesystem image"
  
  if ! [ -r $1 ]; then
    exit_message "image $1 could not be read"
  fi
  
  stage_down
}

function set_xcomp
{
  stage_up
  message "setting up environment"
  
  CC="$1-linux-gcc"
  CXX="$1-linux-g++"
  AR="$1-linux-ar"
  AS="$1-linux-as"
  RANLIB="$1-linux-ranlib"
  LD="$1-linux-ld"
  STRIP="$1T-linux-strip"
  export CC CXX AR AS RANLIB LD STRIP
  
  CFLAGS="-I$2/usr/include -I$3/include -I$3/usr/include"
  LDFLAGS="-L$2/lib -L$2/usr/lib -L$3/lib -L$3/usr/lib"
  if [ "$4" != "true" ]; then 
    LDFLAGS="$LDFLAGS -s"
  fi
  export CFLAGS LDFLAGS

  stage_down
}

function warn_message
{
  message "warning: $1"
}

function exit_message
{
  message "error: $1"
  
  STAGE=0
  message "bailing out, see $LOGFILE for details"
  
  exit 1
}

function message
{
  for (( S=0; S < $STAGE; S++ )); do
    echo -n "    "
  done
  echo -n "|-> "
  echo $1
}

function stage_up
{
  STAGE=`echo $STAGE + 1`
}

function stage_down
{
  STAGE=`echo $STAGE - 1`
}

function execute
{
  stage_up

  while [ "$1" != "" ]; do
    if [ "$VERBOSE" == "true" ]; then
      message "executing \"$1\""
    fi
  
    echo "COMMAND: $1" > $LOGFILE
    echo "INVOKED BY: $SCRIPT" >> $LOGFILE
    echo "INVOKED IN: `pwd`" >> $LOGFILE
    echo "TIMESTAMP: `date`" >> $LOGFILE
    echo -n "ENVIRONMENT: " >> $LOGFILE
    echo `printenv` >> $LOGFILE
    echo -n "****************************************" >> $LOGFILE
    echo "****************************************" >> $LOGFILE
    $1 >> $LOGFILE 2>&1
    
    if [ "$?" != 0 ]; then
      exit_message "failed to execute command \"$1\""
    fi
    shift
  done
  
  stage_down
}

function execute_if
{
  COND = $1
  shift
  
  if [ "$COND" == "true" ]; then
    execute $2
  fi
}

function execute_root
{
  execute chroot $1 $2
}

function mk_dirs
{
  ROOT=$1
  shift
  
  stage_up
  message "making directory structure in $ROOT"
  
  while [ "$1" != "" ]; do
    stage_up
    message "making directory /$1"
    
    execute "mkdir -p $ROOT/$1"
    shift
    
    stage_down
  done
  
  stage_down
}

function mk_files
{
  ROOT=$1
  shift
  
  stage_up
  message "making files in $ROOT"
  
  while [ "$1" != "" ]; do
    stage_up
    message "making file /$1"
    
    execute "touch $ROOT/$1"
    >$ROOT/$1
    execute "chmod -v $2 $ROOT/$1"
    shift
    shift
    
    stage_down
  done
  
  stage_down
}

function rm_dirs
{
  ROOT=$1
  shift
  
  stage_up
  message "removing directories in $ROOT"
  
  while [ "$1" != "" ]; do
    stage_up
    message "removing directory /$1"
    
    execute "rm -rf $ROOT/$1"
    shift
    
    stage_down
  done
  
  stage_down
}

function rm_files
{
  ROOT=$1
  shift
  
  stage_up 
  message "removing files in $ROOT"
  
  RETDIR=`pwd`
  execute "cd $ROOT"
  
  while [ "$1" != "" ]; do
    stage_up
    message "removing file(s) /$1"
    
    execute "find -wholename ./$1 -exec rm -rf {} ;"
    shift
    
    stage_down
  done
  
  execute "cd $RETDIR"
  
  stage_down
}

function rm_brokenlinks
{
  stage_up 
  message "removing broken links in $1"
  
  LINKS=`find $1 -type l`
  for LINK in $LINKS; do
    if ! [ -e $LINK ]; then
      execute "rm -rf $LINK"
    fi  
  done
  
  stage_down
}

function fill_files
{
  ROOT=$1
  shift
  
  stage_up
  message "filling files in $ROOT"
  
  while [ "$1" != "" ]; do
    stage_up
    message "filling file /$1"
    
    echo -e $2 > $ROOT/$1
    shift
    shift
    
    stage_down
  done
  
  stage_down
}

function mk_devices
{
  ROOT=$1
  shift

  stage_up
  message "making devices in $ROOT/dev"
  
  while [ "$1" != "" ]; do
    stage_up
    message "making device(s) $1"
    
    execute "cp -a --remove-destination /dev/$1 $ROOT/dev"
    shift
    
    stage_down
  done
  
  stage_down
}

function get_filesize
{
  RETVAL=(`du -hks $1`)
  SIZE=${RETVAL[0]}
}

function get_dirsize
{
  RETVAL=(`du -hks $1`)
  SIZE=${RETVAL[0]}
}

function extract_package
{
  EXTRACTPKG=$2

  case ${EXTRACTPKG##*.} in
    tar) execute "tar -xf $EXTRACTPKG -C $1"
         EXTRACTPKG=`basename $EXTRACTPKG .tar`
         ;;
    bz2) execute "tar -xjf $EXTRACTPKG -C $1"
         EXTRACTPKG=`basename $EXTRACTPKG .tar.bz2`
         ;;
     gz) execute "tar -xzf $EXTRACTPKG -C $1"
         EXTRACTPKG=`basename $EXTRACTPKG .tar.gz`
         ;;
      *) exit_message "unsupported format of package $EXTRACTPKG"
         ;;
  esac
}

function process_packages
{
  SUFFIX=$1
  shift
  abs_path $1
  ROOT=$ABSPATH
  shift
  USRDIR=$1
  USRROOT=$ROOT$USRDIR
  shift
  BUILDROOT=$1
  shift
  abs_path $1
  PGKDIR=$ABSPATH
  shift
  abs_path $1
  CFGDIR=$ABSPATH
  shift
  HOST=$1
  shift
  TARGET=$1
  shift
  INSTALL=$1
  shift

  stage_up
  message "processing package list"
  stage_up
  
  while [ "$1" != "" ]; do
    PKGS=`ls $PKGDIR/$1-[0-9]*.{gz,bz2} 2> /dev/null`
    
    if [ "$PKGS" == "" ]; then
      warn_message "package $1 not found"
    fi
    
    for PKG in $PKGS; do
      PKG=`readlink -m $PKG`
      if [ -r $PKG ]; then
        ADDONS=""
        BUILDDIR="."
        DEFCONFIGURE="./configure --prefix=$USRROOT --exec-prefix=$ROOT"
        DEFCONFIGURE="$DEFCONFIGURE --build=$HOST-linux --host=$TARGET-linux"
        CONFIGURE=("$DEFCONFIGURE")
        MAKEBUILD=("make all")
        MAKEINSTALL=("make install")
        COMMENT="this may take a while"
        TARNAME=`basename $PKG`
        FULLNAME=${TARNAME%.tar*}
        NAME=${FULLNAME%%-[0-9]*}
        VERSION=${FULLNAME##*-}

        message "processing package $1"
        PKGBUILDROOT=$BUILDROOT/$1
        stage_up

        if [ -r $CFGDIR/$1.$SUFFIX ]; then
          message "sourcing package configuration"
          . $CFGDIR/$1.$SUFFIX
        fi

        if [ -x $PKGBUILDROOT ]; then
          message "contents of `basename $PKG` found in $PKGBUILDROOT"
        else
          message "extracting contents of `basename $PKG` to $PKGBUILDROOT"
          extract_package $BUILDROOT $PKG
          
          PATCHES=`ls $PKGDIR/$1-[0-9]*.patch 2> /dev/null`
          if [ "$PATCHES" != "" ]; then
            message "patching package sources"
            stage_up
            
            for PATCH in $PATCHES; do
              message "applying $PATCH"
              patch -d $BUILDROOT -p0 < $PATCH >& /dev/null
            done
            
            stage_down
          fi

          if [ ! -x $PKGBUILDROOT ]; then 
            execute "mv $BUILDROOT/$EXTRACTPKG $PKGBUILDROOT"
          fi
          
          if [ "$ADDONS" != "" ]; then
            message "extracting addons to $PKGBUILDROOT"
            stage_up
            
            for ADDON in $ADDONS; do
              ADDONPKGS=`ls $PKGDIR/$ADDON-[0-9]*.{gz,bz2} 2> /dev/null`
              for ADDONPKG in $ADDONPKGS; do
                ADDONPKG=`readlink -m $ADDONPKG`
              
                MSG="extracting contents of `basename $ADDONPKG`"
                MSG="$MSG to $PKGBUILDROOT"
                message $MSG
                extract_package $PKGBUILDROOT $ADDONPKG
              done
            done
            
            stage_down
          fi
        fi
        
        message "descending into build directory"                
        RETDIR=`pwd`
        execute "cd $PKGBUILDROOT"
        if [ ! -x $BUILDDIR ]; then
          execute "mkdir -p $BUILDDIR"
        fi
        execute "cd $BUILDDIR"
          
        if [ "$INSTALL" != "true" ] && [ "$CONFIGURE" != "" ]; then
          message "configuring package sources"                
          execute "${CONFIGURE[@]}"
        fi
        if [ "$INSTALL" != "true" ] && [ "$MAKEBUILD" != "" ]; then
          message "compiling package sources ($COMMENT)"
          execute "${MAKEBUILD[@]}"
        fi
            
        if [ "$MAKEINSTALL" != "" ]; then
          message "installing built package content"
          execute "${MAKEINSTALL[@]}"
        fi
            
        message "ascending from build directory"                
        execute "cd $RETDIR"
    
        stage_down    
      fi
    done
    shift
  done
  
  stage_down
  stage_down
}

function mk_image
{
  stage_up
  message "making filesystem image $1"
  stage_up
  
  message "evaluating image size"
  get_dirsize $2
  calc "$SIZE*1.05+$5"
  SIZE=$RETVAL
  
  message "writing zeroed image"
  execute "dd if=/dev/zero of=$1 bs=1k count=$SIZE"
  
  message "building filesystem on image device"
  execute "/sbin/mkfs -t $4 -F $1"
  
  message "mounting filesystem image to $3"
  execute "mkdir -p $3"
  execute "mount -o loop $1 $3"
  
  message "copying filesystem content"
  execute "cp -a $2/* $3"
  
  message "unmounting filesystem image"
  execute "umount $3"
  execute "rm -rf $3"
  
  stage_down
  stage_down
}

function clean
{
  stage_up
  message "cleaning up working directories"
  
  while [ "$1" != "" ]; do
    rm -rf $1
    shift
  done
  
  stage_down
}

function upload_image
{
  message "uploading image $1 to $2"
  
  execute "scp $1 $3@$2:$4"
}
