#! /bin/bash

function unc_and_clean { 
 uncrustify --replace -c ./uncrustify.cfg $1/*.h $1/*.cpp
 rm $1/*.unc-backup*
}

unc_and_clean physics/include
unc_and_clean physics/src

exit 

unc_and_clean main
unc_and_clean main/src
unc_and_clean main/include

unc_and_clean main/viewer

