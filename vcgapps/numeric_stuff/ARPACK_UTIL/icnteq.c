/* icnteq.f -- translated by f2c (version 19991025).
   You must link the resulting object file with the libraries:
	-lf2c -lm   (in that order)
*/

#include "f2c.h"


/* ----------------------------------------------------------------------- */

/*     Count the number of elements equal to a specified integer value. */

integer icnteq_(n, array, value)
integer *n, *array, *value;
{
    /* System generated locals */
    integer ret_val, i__1;

    /* Local variables */
    static integer i__, k;



    /* Parameter adjustments */
    --array;

    /* Function Body */
    k = 0;
    i__1 = *n;
    for (i__ = 1; i__ <= i__1; ++i__) {
	if (array[i__] == *value) {
	    ++k;
	}
/* L10: */
    }
    ret_val = k;

    return ret_val;
} /* icnteq_ */

