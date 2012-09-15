#!/bin/bash
# Automated install for jetty altshell for ReplicatorG > V33 on OS X
# Created by Aaron Ciuffo - aaron.ciuffo@gmail.com http://txoof.com
#
# This script should copy in the appropriate altshell files for your 
# version of ReplicatorG.  This script should work with versions of RepG > V33.
#

#Path to ReplicatorG
REPG=/Applications/ReplicatorG.app
#Base path to machine profiles
SFENGINE="$REPG/Contents/Resources/skein_engines/skeinforge-"
#Base path to craft directory
SFPLUGIN="skeinforge_application/skeinforge_plugins/craft_plugins/"
#Base path to profile plugins
SFPROFILE="skeinforge_application/skeinforge_plugins/profile_plugins/"

if [ -e $REPG ]; then
   echo "Found ReplicatorG"
   echo "Copying appropriate altshell profiles into ReplicatorG"
   for FILE in ./extrusion*.py
   do
      VERSION=`echo $FILE|cut -d"-" -f2|cut -d"." -f1`
      if [ -d $SFENGINE$VERSION ]; then
         echo "Copying altshell.py for version $VERSION" 
         cp ./altshell.py $SFENGINE$VERSION/$SFPLUGIN
         #make a backup of the extrusion.py
         echo "Copying extrusion.py for version: $VERSION"
         cp $SFENGINE$VERSION/$SFPROFILE/extrusion.py $SFENGINE$VERSION/$SFPROFILE/extrusion.py.old
         echo "Backup of extrusion.py stored as extrusion.py.old"
         cp $FILE $SFENGINE$VERSION/$SFPROFILE/extrusion.py
      fi
   done
else
   echo "Could not find $REPG; exiting"
   exit 0
fi
