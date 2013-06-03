## Process this file with automake to produce Makefile.in

SUBDIRS = src tools examples doc test

pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = gts.pc

ChangeLog:
	darcs changes > ChangeLog

debian-snapshot:
	sh debian/changelog.sh
	dpkg-buildpackage -rfakeroot -b
	rm -f debian/repo/*
	mv -f ../*.changes ../*.deb debian/repo
	cd debian/repo && dpkg-scanpackages . /dev/null | gzip -9c > Packages.gz
