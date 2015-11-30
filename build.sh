#!/bin/sh

if [ ! -f mkdeb ] ; then
	wget http://people.debian.org/~wijnen/mkdeb
	chmod a+x mkdeb
fi
submodules="python-fhs python-network python-websocketd"
for submodule in $submodules ; do
	if [ ! -d $submodule ] ; then
		git submodule add https://github.com/wijnen/$submodule
	fi
	cd $submodule
	../mkdeb
	targets="$targets /tmp/$submodule*deb"
done
./mkdeb
targets="$targets /tmp/franklin*deb"
sudo dpkg --install $targets
