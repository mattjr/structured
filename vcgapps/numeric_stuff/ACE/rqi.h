#ifndef _RQI_H_
#define _RQI_H_

void      y2x(
				double  **xvecs,		/* pointer to list of x-vectors */
				int       ndims,		/* number of divisions to make (# xvecs) */
				int       nmyvtxs,		/* number of vertices I own (lenght xvecs) */
				double   *wsqrt		/* sqrt of vertex weights */
			);


void      x2y(
				double  **yvecs,		/* pointer to list of y-vectors */
				int       ndims,		/* number of divisions to make (# yvecs) */
				int       nmyvtxs,		/* number of vertices I own (lenght yvecs) */
				double   *wsqrt 		/* sqrt of vertex weights */
			);
#endif
