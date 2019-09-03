#!/bin/bash

DIR="$PWD"

dtc_git_build () {
	project="dtc"
	server="git://git.kernel.org/pub/scm/utils/dtc/dtc.git"

	if [ ! -d ${HOME}/git/ ] ; then
		mkdir -p ${HOME}/git/ || true
	fi

	if [ ! -f ${HOME}/git/${project}/.git/config ] ; then
		git clone ${server} ${HOME}/git/${project}/
	fi

	cd ${HOME}/git/${project}/
	make clean
	git checkout master -f
	git pull || true

	if [ ! "x${git_tag}" = "x" ] ; then
		test_for_branch=$(git branch --list ${git_tag}-build)
		if [ "x${test_for_branch}" != "x" ] ; then
			git branch ${git_tag}-build -D
		fi

		git checkout ${git_tag} -b ${git_tag}-build
	fi

	make clean
	make CC=gcc CROSS_COMPILE= all &> /dev/null
	cd ${DIR}/
}

git_tag="v1.4.4"
dtc_git_build

echo "*********************************************"
echo "dtc: `/var/lib/jenkins/git/dtc/dtc --version`"
DTC=/var/lib/jenkins/git/dtc/dtc make clean
DTC=/var/lib/jenkins/git/dtc/dtc make all || exit 1
echo "*********************************************"

git_tag="v1.4.6"
dtc_git_build

echo "*********************************************"
echo "dtc: `/var/lib/jenkins/git/dtc/dtc --version`"
DTC=/var/lib/jenkins/git/dtc/dtc DTCVERSION=2.0.0 make clean
DTC=/var/lib/jenkins/git/dtc/dtc DTCVERSION=2.0.0 make all || exit 1
echo "*********************************************"

git_tag="v1.4.7"
dtc_git_build

echo "*********************************************"
echo "dtc: `/var/lib/jenkins/git/dtc/dtc --version`"
DTC=/var/lib/jenkins/git/dtc/dtc DTCVERSION=2.0.0 make clean
DTC=/var/lib/jenkins/git/dtc/dtc DTCVERSION=2.0.0 make all || exit 1
echo "*********************************************"

git_tag="v1.5.0"
dtc_git_build

echo "*********************************************"
echo "dtc: `/var/lib/jenkins/git/dtc/dtc --version`"
DTC=/var/lib/jenkins/git/dtc/dtc DTCVERSION=2.0.0 make clean
DTC=/var/lib/jenkins/git/dtc/dtc DTCVERSION=2.0.0 make all || exit 1
echo "*********************************************"

unset git_tag
dtc_git_build

echo "*********************************************"
echo "dtc: `/var/lib/jenkins/git/dtc/dtc --version`"
DTC=/var/lib/jenkins/git/dtc/dtc DTCVERSION=2.0.0 make clean
DTC=/var/lib/jenkins/git/dtc/dtc DTCVERSION=2.0.0 make all || exit 1
echo "*********************************************"

