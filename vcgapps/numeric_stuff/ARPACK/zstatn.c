/* zstatn.f -- translated by f2c (version 19991025).
   You must link the resulting object file with the libraries:
	-lf2c -lm   (in that order)
*/

#include "f2c.h"

/* Common Block Declarations */

struct {
    integer nopx, nbx, nrorth, nitref, nrstrt;
    real tsaupd, tsaup2, tsaitr, tseigt, tsgets, tsapps, tsconv, tnaupd, 
	    tnaup2, tnaitr, tneigh, tngets, tnapps, tnconv, tcaupd, tcaup2, 
	    tcaitr, tceigh, tcgets, tcapps, tcconv, tmvopx, tmvbx, tgetv0, 
	    titref, trvec;
} timing_;

#define timing_1 timing_


/* \SCCS Information: @(#) */
/* FILE: statn.F   SID: 2.2   DATE OF SID: 4/20/96   RELEASE: 2 */

/*     %---------------------------------------------% */
/*     | Initialize statistic and timing information | */
/*     | for complex nonsymmetric Arnoldi code.      | */
/*     %---------------------------------------------% */
/* Subroutine */ int zstatn_()
{

/*     %--------------------------------% */
/*     | See stat.doc for documentation | */
/*     %--------------------------------% */

/*     %-----------------------% */
/*     | Executable Statements | */
/*     %-----------------------% */
/*     %--------------------------------% */
/*     | See stat.doc for documentation | */
/*     %--------------------------------% */

/* \SCCS Information: @(#) */
/* FILE: stat.h   SID: 2.2   DATE OF SID: 11/16/95   RELEASE: 2 */


    timing_1.nopx = 0;
    timing_1.nbx = 0;
    timing_1.nrorth = 0;
    timing_1.nitref = 0;
    timing_1.nrstrt = 0;
    timing_1.tcaupd = (float)0.;
    timing_1.tcaup2 = (float)0.;
    timing_1.tcaitr = (float)0.;
    timing_1.tceigh = (float)0.;
    timing_1.tcgets = (float)0.;
    timing_1.tcapps = (float)0.;
    timing_1.tcconv = (float)0.;
    timing_1.titref = (float)0.;
    timing_1.tgetv0 = (float)0.;
    timing_1.trvec = (float)0.;
/*     %----------------------------------------------------% */
/*     | User time including reverse communication overhead | */
/*     %----------------------------------------------------% */
    timing_1.tmvopx = (float)0.;
    timing_1.tmvbx = (float)0.;
    return 0;

/*     %---------------% */
/*     | End of zstatn | */
/*     %---------------% */

} /* zstatn_ */

