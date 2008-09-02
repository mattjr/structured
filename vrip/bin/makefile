#! gmake

#
#  Brian Curless
#  
#  Computer Graphics Laboratory
#  Stanford University
#  
#  ---------------------------------------------------------------------
#  
#  Copyright (1997) The Board of Trustees of the Leland Stanford Junior
#  University. Except for commercial resale, lease, license or other
#  commercial transactions, permission is hereby given to use, copy,
#  modify this software for academic purposes only.  No part of this
#  software or any derivatives thereof may be used in the production of
#  computer models for resale or for use in a commercial
#  product. STANFORD MAKES NO REPRESENTATIONS OR WARRANTIES OF ANY KIND
#  CONCERNING THIS SOFTWARE.  No support is implied or provided.
#  

# Make the symbolic links necessary for this directory,
# since cvs doesn't save link files...

PROGS = \
	crunch.tcl \
	ply2asc \
	ply2bin \
	ply2verts \
	plyaddinfo \
	plyarea \
	plybbox \
	plybound \
	plycomps \
	plycrunch \
	plyflip \
	plyhead \
	plynormals \
	plyremove \
	plyshared \
	plystrip \
	plysubtract \
	plyunshared \
	plyusages \
	plyxform \
	vripxftrans\
	plydice \
	plydicegroup \
	plymirror \
	plyvolume \
	plysubdiv \
	xyz2ply \
	vrip \
	vriphead \
	vripmarch \
	vripnew \
	vripslicer \
	vripsurf \
	vripupdate \
	loadbalance \
	vripdicebbox\
	permute \
	plycull \
	plycullmaxx \
	plycullmaxx_inv \
	plymerge \
	pvrip1 \
	pvrip2 \
	pvripsplit \
	vripsplit \
	vripsubvollist \
	plyclean \
	plygridtomesh \
	plyclipbboxes\
	plysubtractlist\
	$(NULL)

default: all

all: $(PROGS)

clean: 
	-rm -f *.o *~

clobber:
	-rm -f *.o *~ $(PROGS)

depend:
	echo > makedep
	makedepend -fmakedep -- $(CFLAGS) -- $(CSRCS)
	-rm makedep.bak

install: clobber depend all

# Get OS uname
UNAME := $(shell uname)

ifeq ($(UNAME), Cygwin)

    crunch.tcl:
	    cp ../src/plytools/crunch.tcl crunch.tcl

    ply2asc:
	    cp ../src/plytools/ply2asc.exe .

    ply2bin:
	    cp ../src/plytools/ply2bin.exe .

    ply2crunchset:
	    cp ../src/plytools/ply2crunchset .

    ply2verts:
	    cp ../src/plytools/ply2verts.exe .

    plyaddinfo:
	    cp ../src/plytools/plyaddinfo.exe .

    plyarea:
	    cp ../src/plytools/plyarea.exe .

    plybbox:
	    cp ../src/plytools/plybbox.exe .

    plybound:
	    cp ../src/plytools/plybound.exe .

    plycomps:
	    cp ../src/plytools/plycomps.exe .

    plycrunch:
	    cp ../src/plytools/plycrunch.exe .

    plyflip:
	    cp ../src/plytools/plyflip.exe .

    plyhead:
	    cp ../src/plytools/plyhead.exe .

    plynormals:
	    cp ../src/plytools/plynormals.exe .

    plyremove:
	    cp ../src/plytools/plyremove.exe .

    plysets2conf:
	    cp ../src/plytools/plysets2conf .

    plysetlevel:
	    cp ../src/plytools/plysetlevel .

    plyshared:
	    cp ../src/plytools/plyshared.exe .

    plysubtract:
	    cp ../src/plytools/plysubtract.exe .

    plystrip:
	    cp ../src/plytools/plystrip.exe .

    plyunshared:
	    cp ../src/plytools/plyunshared.exe .

    plyusages:
	    cp ../src/plytools/plyusages.csh plyusages

    plyxform:
	    cp ../src/plytools/plyxform.exe .

    plydice:
	    cp ../src/plytools/plydice.exe .

    plymirror:
	    cp ../src/plytools/plymirror.exe .

    plyvolume:
	    cp ../src/plytools/plyvolume.exe .

    plysubdiv:
	    cp ../src/plytools/plysubdiv.exe .

    xyz2ply:
	    cp ../src/plytools/xyz2ply.exe .

    mm2plys:
	    cp ../src/plytools/mm2plys .

    vrip:
	    cp ../src/vrip/vrip.csh vrip

    vriphead:
	    cp ../src/vrip/vriphead.exe .

    vripmarch:
	    cp ../src/march/vripmarch.exe .

    vripnew:
	    cp ../src/vrip/vripnew.csh vripnew

    vripslicer:
	    cp ../src/vrip/vripslicer.csh vripslicer

    vripsurf:
	    cp ../src/vrip/vripsurf.csh vripsurf

    vripupdate:
	    cp ../src/vrip/vripupdate.csh vripupdate

    loadbalance:
	    cp ../src/pvrip/loadbalance .

    vripdicebbox:
	cp ../src/pvrip/vripdicebbox .

    permute:
	    cp ../src/pvrip/permute .

    plycull:
	    cp ../src/pvrip/plycull .

    plycullmaxx:
	    cp ../src/pvrip/plycullmaxx.exe .

    plymerge:
	    cp ../src/pvrip/plymerge.exe .

    pvrip1:
	    cp ../src/pvrip/pvrip1 .

    pvrip2:
	    cp ../src/pvrip/pvrip2 .

    pvripsplit:
	    cp ../src/pvrip/pvripsplit .

    vripsplit:
	    cp ../src/pvrip/vripsplit .

    vripsubvollist:
	    cp ../src/pvrip/vripsubvollist .

    plyclean:
	    cp ../src/plyclean/plyclean.exe .

    plygridtomesh:
	    cp ../src/vrip/plygridtomesh .

else

    crunch.tcl:
	    ln -sf ../src/plytools/crunch.tcl crunch.tcl

    ply2asc:
	    ln -sf ../src/plytools/ply2asc ply2asc

    ply2bin:
	    ln -sf ../src/plytools/ply2bin ply2bin

    ply2verts:
	    ln -sf ../src/plytools/ply2verts ply2verts

    plyaddinfo:
	    ln -sf ../src/plytools/plyaddinfo plyaddinfo

    plyarea:
	    ln -sf ../src/plytools/plyarea plyarea

    plybbox:
	    ln -sf ../src/plytools/plybbox plybbox

    plybound:
	    ln -sf ../src/plytools/plybound plybound

    plycomps:
	    ln -sf ../src/plytools/plycomps plycomps

    plycrunch:
	    ln -sf ../src/plytools/plycrunch plycrunch

    plyflip:
	    ln -sf ../src/plytools/plyflip plyflip

    plyhead:
	    ln -sf ../src/plytools/plyhead plyhead

    plynormals:
	    ln -sf ../src/plytools/plynormals plynormals

    plyremove:
	    ln -sf ../src/plytools/plyremove plyremove

    plysets2conf:
	    ln -sf ../src/plytools/plysets2conf plysets2conf

    plyshared:
	    ln -sf ../src/plytools/plyshared plyshared

    plysubtract:
	    ln -sf ../src/plytools/plysubtract plysubtract

    plysubtractlist:
	    ln -sf ../src/plytools/plysubtractlist plysubtractlist

    plystrip:
	    ln -sf ../src/plytools/plystrip plystrip

    plyunshared:
	    ln -sf ../src/plytools/plyunshared plyunshared

    plyusages:
	    ln -sf ../src/plytools/plyusages.csh plyusages

    plyxform:
	    ln -sf ../src/plytools/plyxform plyxform

    vripxftrans:
	    ln -sf ../src/pvrip/vripxftrans vripxftrans

    plydice:
	    ln -sf ../src/plytools/plydice plydice

   plydicegroup:
	    ln -sf ../src/plytools/plydicegroup plydicegroup

    plymirror:
	    ln -sf ../src/plytools/plymirror plymirror

    plyvolume:
	    ln -sf ../src/plytools/plyvolume plyvolume

    plysubdiv:
	    ln -sf ../src/plytools/plysubdiv plysubdiv

    xyz2ply:
	    ln -sf ../src/plytools/xyz2ply xyz2ply

    mm2plys:
	    ln -sf ../src/plytools/mm2plys mm2plys

    vrip:
	    ln -sf ../src/vrip/vrip.csh vrip

    vriphead:
	    ln -sf ../src/vrip/vriphead vriphead

    vripmarch:
	    ln -sf ../src/march/vripmarch vripmarch

    vripnew:
	    ln -sf ../src/vrip/vripnew.csh vripnew

    vripslicer:
	    ln -sf ../src/vrip/vripslicer.csh vripslicer

    vripsurf:
	    ln -sf ../src/vrip/vripsurf.csh vripsurf

    vripupdate:
	    ln -sf ../src/vrip/vripupdate.csh vripupdate

    loadbalance:
	    ln -sf ../src/pvrip/loadbalance loadbalance

    vripdicebbox:
	    ln -sf ../src/pvrip/vripdicebbox vripdicebbox

    permute:
	    ln -sf ../src/pvrip/permute permute

    plycull:
	    ln -sf ../src/pvrip/plycull plycull

    plycullmaxx:
	    ln -sf ../src/pvrip/plycullmaxx plycullmaxx

    plycullmaxx_inv:
	    ln -sf ../src/pvrip/plycullmaxx_inv plycullmaxx_inv

    plymerge:
	    ln -sf ../src/pvrip/plymerge plymerge

    pvrip1:
	    ln -sf ../src/pvrip/pvrip1 pvrip1

    pvrip2:
	    ln -sf ../src/pvrip/pvrip2 pvrip2

    pvripsplit:
	    ln -sf ../src/pvrip/pvripsplit pvripsplit

    vripsplit:
	    ln -sf ../src/pvrip/vripsplit vripsplit

    vripsubvollist:
	    ln -sf ../src/pvrip/vripsubvollist vripsubvollist

    plyclean:
	    ln -sf ../src/plyclean/plyclean plyclean

    plygridtomesh:
	    ln -sf ../src/vrip/plygridtomesh plygridtomesh

    plyclipbboxes:
	    ln -sf ../src/plytools/plyclipbboxes plyclipbboxes

endif


