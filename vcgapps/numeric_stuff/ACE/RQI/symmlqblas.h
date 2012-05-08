
#include "f2c.h"

 /* Subroutine */ int daxpy_(
							integer  *n,
							doublereal *da, doublereal *dx,
							integer  *incx,
							doublereal *dy,
							integer  *incy);
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
 /* Subroutine */ int dcopy_(
							integer  *n,
							doublereal *dx,
							integer  *incx,
							doublereal *dy,
							integer  *incy
							);


/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
doublereal ddot_(
				integer  *n,
				doublereal *dx,
				integer  *incx,
				doublereal *dy,
				integer  *incy
				);


/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
doublereal dnrm2_(
					integer  *n,
					doublereal *dx,
					integer  *incx
					);
