//
// structured - Tools for the Generation and Visualization of Large-scale
// Three-dimensional Reconstructions from Image Data. This software includes
// source code from other projects, which is subject to different licensing,
// see COPYING for details. If this project is used for research see COPYING
// for making the appropriate citations.
// Copyright (C) 2013 Matthew Johnson-Roberson <mattkjr@gmail.com>
//
// This file is part of structured.
//
// structured is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// structured is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with structured.  If not, see <http://www.gnu.org/licenses/>.
//


#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include <stdio.h>
#include <stdlib.h>
/* The semaphore key is an arbitrary long integer which serves as an
   external identifier by which the semaphore is known to any program
   that wishes to use it. */

#define KEY (1492)

int main()
{
   int id; /* Number by which the semaphore is known within a program */

   /* The next thing is an argument to the semctl() function. Semctl()
      does various things to the semaphore depending on which arguments
      are passed. We will use it to make sure that the value of the
      semaphore is initially 0. */

        union semun {
                int val;
                struct semid_ds *buf;
                ushort * array;
        } argument;

   argument.val = 1;

   /* Create the semaphore with external key KEY if it doesn't already
      exists. Give permissions to the world. */

   id = semget(KEY, 1, 0666 | IPC_CREAT);

   /* Always check system returns. */

   if(id < 0)
   {
      fprintf(stderr, "Unable to obtain semaphore.\n");
      exit(0);
   }

   /* What we actually get is an array of semaphores. The second
      argument to semget() was the array dimension - in our case
      1. */

   /* Set the value of the number 0 semaphore in semaphore array
      # id to the value 0. */

   if( semctl(id, 0, SETVAL, argument) < 0)
   {
      fprintf( stderr, "Cannot set semaphore value.\n");
   }
   else
   {
      fprintf(stderr, "Semaphore %d initialized.\n", KEY);
   }
}
