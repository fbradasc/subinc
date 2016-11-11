#pragma once

#ifndef constraint
#define constraint(v,min,max) (((v)<(min))?(min):(((v)>(max))?(max):(v)))
#endif

#ifndef linearmap
#define linearmap(x, imin, imax, omin, omax) (((x) - (imin)) * ((omax) - (omin)) / ((imax) - (imin)) + (omin))
#endif
