cmake_minimum_required(VERSION 2.6)

include(ReMake)

remake_project(
  "RoboX Utils"
  VERSION 0.1
  RELEASE alpha
  SUMMARY "Utility binaries for the RoboX platform."
  AUTHOR "Ralf Kaestner"
  CONTACT "ralf.kaestner@gmail.com"
  HOME "http://aslforge.ethz.ch/projects/carmen-robox"
  LICENSE "GNU Lesser General Public License (LGPL)"
  INSTALL /usr
  PREFIX robox-
)

remake_pack_deb(tulibs)
