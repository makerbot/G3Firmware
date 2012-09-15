// Generate the SqrtTable.hh file for a given "resolution"
//
//   cc -o SqrtTable -I../shared/avrfix/ SqrtTable.c
//   ./SqrtTable > SqrtTable.hh

#include <stdio.h>
#include <math.h>

#define TEST_ON_PC
#include "avrfix.h"

#define SQRT_TABLE_SHIFT 6
#define RESOLUTION       (1 << SQRT_TABLE_SHIFT)

main()
{
     _Accum as;
     float fv, fs;
     int i;

     printf(
	  "#ifndef __SQRT_TABLE_H__\n"
	  "\n"
	  "#include \"Configuration.hh\"\n"
	  "\n"
	  "#define __SQRT_TABLE_H__\n"
	  "\n"
	  "// Square root lookup table for finding the square root of 1 + x for 0 <= x <= 3\n"
	  "//\n"
	  "//    sqrt(1.0 + x) = sqrt_table[(int)(x * SQRT_TABLE_RESOLUTION)]\n"
	  "//    sqrt(1.0 + x) = sqrt_table[(int)(x << SQRT_TABLE_SHIFT)]\n"
	  "\n"
	  "#define SQRT_TABLE_SHIFT      %d\n"
	  "#define SQRT_TABLE_RESOLUTION %d\n"
	  "\n"
	  "#ifdef FIXED\n"
	  "\n"
	  "#ifdef SMALL_4K_RAM\n"
	  "	typedef _Accum PROGMEM prog_FPTYPE;\n"
	  "	const static PROGMEM prog_FPTYPE sqrt_table[%d] = {\n"
	  "#else\n"
	  "	static FPTYPE sqrt_table%d] = {\n"
	  "#endif\n",
	  SQRT_TABLE_SHIFT, RESOLUTION, 1 + RESOLUTION * 3);

     for (i = 0; i <= 3 * RESOLUTION; i++)
     {
	  fv = (float)i / (float)RESOLUTION;
	  fs = sqrt(1.0 + fv);
	  as = ftok(fs);

	  printf("\t0x%08x%c // ftok(sqrt(1.0 + (float)%d/%d.0)) = ftok(sqrt(1.0 + %f)) = ftok(%f)\n",
		 as, (i != 3*RESOLUTION) ? ',' : ' ', i, RESOLUTION, fv, fs);
     }

     printf("};\n"
	    "\n"
	    "#else\n"
	    "\n"
	    "static float sqrt_table[%d] = {\n",
	    1 + RESOLUTION * 3);

     for (i = 0; i <= 3 * RESOLUTION; i++)
     {
	  fv = (float)i / (float)RESOLUTION;
	  fs = sqrt(1.0 + fv);

	  printf("\t%25.23f%c // sqrt(1.0 + (float)%d/%d.0) = sqrt(1.0 + %f))\n",
		 fs, (i != 3*RESOLUTION) ? ',' : ' ', i, RESOLUTION, fv);
     }

     printf("};\n"
	    "\n"
	    "#endif\n"
	    "\n"
	    "#endif\n");
}
