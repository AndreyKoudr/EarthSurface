/*
BSD 2-Clause License

Copyright (c) 2020, Andrey Kudryavtsev (andrewkoudr@hotmail.com)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include "Vector.h"

/**
  Some matrix operations needed for this project
*/
                              // transpose matrix, no bound controls is performed
template <class T> 
void transpose(T *matrix, int n1, int n2, T *trans)
{
  for (int i = 0; i < n1; i++)
  {
    for (int j = 0; j < n2; j++)
    {
      trans[j * n1 + i] = matrix[i * n2 + j];
    }
  }
}
                              // fast multiplication with incremented addresses
template <class T> 
void fastM(T *x1, T *x2, T *x3, int n1, int n2, int n3, int m1, int m2, int m3)
{
  int m1len,m2len,m3len;
  T *P1,*P2,*P3,t;

  m1len = m1; m2len = m2; m3len = m3;

  for (int i = 0; i < n1; i++)
  {
    P3 = x3; P3 += (i * m3len - 1);
    for (int j = 0; j < n3; j++)
    {
      P3++;
      P1 = x1; P1 += (i * m1len - 1);
      P2 = x2; P2 += (j - m2len); 
      t = 0;
      for (int k = 0; k < n2; k++)
      { 
        P1++;
        P2 += m2len;
        t += (*P1) * (*P2);
      }
      *P3 = t;
    }
  }
}

