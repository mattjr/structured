/*
 * Copyright 1997, Regents of the University of Minnesota
 *
 * metis.h
 *
 * This file includes all necessary header files
 *
 * Started 8/27/94
 * George
 *
 * $Id: metis.h 2435 2005-09-19 14:16:44Z levy $
 */


#include <stdio.h>
#ifdef __STDC__
#include <stdlib.h>
#else
#include <malloc.h>
#endif

#ifndef WIN32
#include <strings.h>
#endif

#include <string.h>
#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <time.h>

#ifdef DMALLOC
#include <dmalloc.h>
#endif

#include <numeric_stuff/METIS/defs.h>
#include <numeric_stuff/METIS/struct.h>
#include <numeric_stuff/METIS/macros.h>
#include <numeric_stuff/METIS/rename.h>
#include <numeric_stuff/METIS/proto.h>

